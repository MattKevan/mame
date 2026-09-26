// datarover_core.cpp — libdatarover core: framebuffer tap + lifecycle/pen/package.
//
// Scanout source: datarover_state::screen_update
// (src/mame/skeleton/datarover.cpp). The LCD panel is the "screen" device tag;
// scanout reads 2bpp pixels from guest DRAM at the Dino video-high-buffer
// base register (Dino MMIO 0x10c00000 + 0x030, masked to 0xfffffff0) with
// fallback base 0x003f6a00 when out of the 4 MiB DRAM window.
//
// Lifecycle mirrors the cli_frontend boot path (src/frontend/mame/clifront.cpp:
// start_execution) minus the CLI: select datarover840 by name, configure
// nvram/cfg/rom paths, build machine_config, run running_machine::run() on a
// worker thread (mame_machine_manager::execute() at src/frontend/mame/mame.cpp:235
// blocks — hence the thread). The manager is the real mame_machine_manager
// singleton, not a minimal subclass: the core video frame path calls
// emulator_info::draw_user_interface/periodic_check/frame_hook, which dereference
// mame_machine_manager::instance(), and running_machine::run() needs
// manager->http() non-null plus a real ui_manager from create_ui — a bare
// machine_manager base would null-deref in all three places. Plugins/Lua stay
// off via options; NVRAM save stays on so guest state persists across boots.
// Teardown is schedule_exit + join + manager delete. One live handle per
// process (the manager singleton binds one options set; a second concurrent
// create returns NULL).
//
// Pen injects into TOUCH_X/TOUCH_Y/TOUCH_BUTTON (datarover.cpp:4672-4679,
// IPT_LIGHTGUN_X/Y + IPT_BUTTON1) via ioport_field::set_value — the same
// injection point as the menu pulsePort (src/osd/sdl3/datarover_menu.mm) and
// the clickable views (src/emu/render.cpp:1317-1320).
//
// Package install ports the Python PCLink codec (tools/pclink_send.py:
// escape/CRC/packet/metadata) to C++ and writes the encoded wire to the
// guest's UART-A/RS-232 endpoint in-process through rs232_host_channel, which
// null_modem drains on the emulation thread (docs/pclink.md: Storeroom
// computer over Dino UART A 19200 8N1, exposed as MAME RS-232 port 1). The
// same handshake still falls back to the desktop PTY slave when a caller
// supplies one, so the Linux/macOS CLI harness keeps working.
//
// Threading contract: the emulation thread exclusively accesses MAME objects.
// Caller threads enqueue pen states and read locked framebuffer snapshots or a
// cached PTY path. destroy() requests exit and joins the worker before freeing
// the handle. Callers must finish other API calls before destroy(). Framebuffer
// snapshots remain valid until the next framebuffer call on that caller thread.

#include "libdatarover/datarover_core.h"

#include "emu.h"

#include "osdepend.h"
#include "main.h"
#include "frontend/mame/mame.h"
#include "emuopts.h"
#include "modules/lib/osdobj_common.h"
#include "modules/netdev/netdev_module.h"
#include "modules/osdmodule.h"
#include "drivenum.h"
#include "gamedrv.h"
#include "mconfig.h"
#include "ioport.h"
#include "dipty.h"
#include "dislot.h"
#include "frontend/mame/ui/menuitem.h"
#include "render.h"
#include "fileio.h"
#include "devices/bus/rs232/host_serial.h"
#include "devices/bus/rs232/null_modem.h"

#include <algorithm>
#include <atomic>
#include <array>
#include <fstream>
#include <functional>
#include <deque>
#include <sstream>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <fcntl.h>
#include <memory>
#include <mutex>
#include <poll.h>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <termios.h>
#include <unistd.h>

GAME_EXTERN(datarover840);

#if defined(OSD_NET_USE_SLIRP)
extern const module_type NETDEV_SLIRP;
#endif

namespace {

constexpr uint32_t DINO_MMIO_BASE = 0x10c00000U;
constexpr uint32_t DINO_VIDEO_HIGH_BUFFER_OFF = 0x030U;
constexpr uint32_t FALLBACK_BASE = 0x003f6a00U;
constexpr uint32_t DRAM_LIMIT = 0x00400000U;

// --- PCLink wire codec: C++ port of tools/pclink_send.py --------------------
// escape set exactly {0x0E,0x0F,0x10}, introducer 0x10, before framing (a pair
// may straddle a 256-byte boundary). Frames: raw BE u16 len + escaped bytes +
// raw BE u32 ~crc32(frame). Packet: CRC stream of (tag[4] + BE u32 len +
// payload). SPkg metadata: 0x404-byte WinPCLink layout (u32be size twice @0,
// 0x80000000 @24, filename char count @28, UTF-16BE name @32). Package stream:
// raw package + four NUL bytes as its own CRC stream (docs/pclink.md).
// CRC-32 is the zlib/IEEE polynomial (init/xorout 0xFFFFFFFF), implemented
// table-driven here so the core links nothing extra; it matches Python
// zlib.crc32 (0xCBF43926 for "123456789") and therefore the ~crc32 framing
// the unit tests pin in tests/test_pclink_regression.py.

const uint32_t *pclink_crc32_table()
{
	static uint32_t table[256];
	static const bool init = []()
	{
		for (uint32_t i = 0; i < 256; ++i)
		{
			uint32_t c = i;
			for (int k = 0; k < 8; ++k)
				c = (c & 1) ? (c >> 1) ^ 0xEDB88320U : (c >> 1);
			table[i] = c;
		}
		return true;
	}();
	(void)init;
	return table;
}

uint32_t pclink_crc32(const uint8_t *data, size_t len)
{
	const uint32_t *table = pclink_crc32_table();
	uint32_t crc = 0xFFFFFFFFU;
	for (size_t i = 0; i < len; ++i)
		crc = table[(crc ^ data[i]) & 0xFFU] ^ (crc >> 8);
	return crc ^ 0xFFFFFFFFU;
}

void pclink_put_be16(std::vector<uint8_t> &out, uint32_t v)
{
	out.push_back(static_cast<uint8_t>((v >> 8) & 0xff));
	out.push_back(static_cast<uint8_t>(v & 0xff));
}

void pclink_put_be32(std::vector<uint8_t> &out, uint32_t v)
{
	out.push_back(static_cast<uint8_t>((v >> 24) & 0xff));
	out.push_back(static_cast<uint8_t>((v >> 16) & 0xff));
	out.push_back(static_cast<uint8_t>((v >> 8) & 0xff));
	out.push_back(static_cast<uint8_t>(v & 0xff));
}

std::vector<uint8_t> pclink_encode_crc_stream(const uint8_t *data, size_t len)
{
	std::vector<uint8_t> escaped;
	escaped.reserve(len + len / 64 + 1);
	for (size_t i = 0; i < len; ++i)
	{
		const uint8_t v = data[i];
		if (v == 0x0e || v == 0x0f || v == 0x10)
			escaped.push_back(0x10);
		escaped.push_back(v);
	}
	std::vector<uint8_t> wire;
	wire.reserve(escaped.size() + (escaped.size() / 256 + 1) * 6);
	for (size_t start = 0; start < escaped.size(); start += 256)
	{
		const size_t n = std::min<size_t>(256, escaped.size() - start);
		pclink_put_be16(wire, static_cast<uint32_t>(n));
		wire.insert(wire.end(), escaped.begin() + start, escaped.begin() + start + n);
		pclink_put_be32(wire, ~pclink_crc32(escaped.data() + start, n));
	}
	return wire;
}

// --- PCLink wire decode: mirrors tools/pclink_send.py decode_crc_stream +
// decode_packet, needed for the Cnct/Pong handshake (the guest speaks first).
// Returns false on any truncation, bad length, CRC mismatch, or escape error.
struct pclink_packet
{
	char tag[4];
	std::vector<uint8_t> payload;
};
// Unescapes an encoded prefix. Returns 1 when the data ends mid-escape pair
// (more frames are needed), -1 on an invalid escape, 0 on success.
int unescape_stream(const std::vector<uint8_t> &encoded, std::vector<uint8_t> &stream)
{
	bool escaped = false;
	stream.clear();
	stream.reserve(encoded.size());
	for (uint8_t v : encoded)
	{
		if (escaped)
		{
			if (v != 0x0e && v != 0x0f && v != 0x10)
				return -1;
			stream.push_back(v);
			escaped = false;
		}
		else if (v == 0x10)
			escaped = true;
		else if (v == 0x0e || v == 0x0f)
			return -1;
		else
			stream.push_back(v);
	}
	return escaped ? 1 : 0;
}

// Decodes the FIRST complete packet in `wire`, reporting how many wire bytes it
// consumed. Trailing data is expected: the guest retries its connect request,
// and every retry it sent before the host started is already queued.
bool pclink_decode_packet(const uint8_t *wire, size_t wire_len, pclink_packet &out,
		size_t &consumed)
{
	std::vector<uint8_t> encoded;
	std::vector<uint8_t> stream;
	size_t pos = 0;
	while (pos < wire_len)
	{
		if (wire_len - pos < 2)
			return false;
		const size_t size = (static_cast<size_t>(wire[pos]) << 8) | wire[pos + 1];
		pos += 2;
		if (size < 1 || size > 256)
			return false;
		if (wire_len - pos < size + 4)
			return false;
		uint32_t crc = 0xFFFFFFFFU;
		const uint32_t *table = pclink_crc32_table();
		for (size_t i = 0; i < size; ++i)
			crc = table[(crc ^ wire[pos + i]) & 0xFFU] ^ (crc >> 8);
		crc ^= 0xFFFFFFFFU;
		uint32_t expected = (static_cast<uint32_t>(wire[pos + size]) << 24)
			| (static_cast<uint32_t>(wire[pos + size + 1]) << 16)
			| (static_cast<uint32_t>(wire[pos + size + 2]) << 8)
			| wire[pos + size + 3];
		if ((~crc & 0xFFFFFFFFU) != expected)
			return false;
		encoded.insert(encoded.end(), wire + pos, wire + pos + size);
		pos += size + 4;

		const int unescaped = unescape_stream(encoded, stream);
		if (unescaped < 0)
			return false;
		if (unescaped > 0 || stream.size() < 8)
			continue;
		const size_t declared = (static_cast<size_t>(stream[4]) << 24)
			| (static_cast<size_t>(stream[5]) << 16)
			| (static_cast<size_t>(stream[6]) << 8)
			| stream[7];
		if (stream.size() < declared + 8)
			continue;
		std::memcpy(out.tag, stream.data(), 4);
		out.payload.assign(stream.begin() + 8, stream.begin() + declared + 8);
		consumed = pos;
		return true;
	}
	return false;
}

std::vector<uint8_t> pclink_encode_packet(const char tag[4], const uint8_t *payload, size_t payload_len)
{
	std::vector<uint8_t> raw;
	raw.reserve(8 + payload_len);
	for (int i = 0; i < 4; ++i)
		raw.push_back(static_cast<uint8_t>(tag[i]));
	pclink_put_be32(raw, static_cast<uint32_t>(payload_len));
	if (payload_len)
		raw.insert(raw.end(), payload, payload + payload_len);
	return pclink_encode_crc_stream(raw.data(), raw.size());
}
std::vector<uint8_t> pclink_package_metadata(uint32_t size, const std::vector<uint8_t> &name16, uint32_t char_count)
{
	std::vector<uint8_t> meta(0x404, 0);
	auto put32 = [&meta](size_t off, uint32_t v)
	{
		meta[off] = static_cast<uint8_t>((v >> 24) & 0xff);
		meta[off + 1] = static_cast<uint8_t>((v >> 16) & 0xff);
		meta[off + 2] = static_cast<uint8_t>((v >> 8) & 0xff);
		meta[off + 3] = static_cast<uint8_t>(v & 0xff);
	};
	put32(0, size);
	put32(4, size);
	put32(24, 0x80000000U);
	put32(28, char_count);
	const size_t n = std::min(name16.size(), meta.size() - 32);
	std::memcpy(meta.data() + 32, name16.data(), n);
	return meta;
}

// --- Headless OSD stub ------------------------------------------------------
// Why a stub instead of sdl_osd_interface with -video none:
// sdl init calls SDL_InitSubSystem(SDL_INIT_VIDEO) and video_init() creates a
// real SDL window per numscreens even with -video none (only the Windows OSD
// skips positioning for a non-interactive renderer). A headless library cannot
// do that. This stub satisfies the osd_interface contract the core uses while
// creating no windows: no window list, update() renders nothing, and the guest
// advances via the normal scheduler timeslice in machine.run(). Sound "none"
// semantics (no_sound() true) match -sound none; everything else is a null
// sink like the modules/* none providers.

class core_headless_osd : public osd_interface
{
public:
	core_headless_osd(osd_options &options, bool network_enabled, bool audio_enabled)
		: m_options(options), m_network_enabled(network_enabled), m_audio_enabled(audio_enabled) { }
	~core_headless_osd() override { m_modules.exit(); }
	std::function<void(running_machine &)> frame_callback;
	std::function<void()> exit_callback;
	std::function<void(int)> network_status_callback;

	void init(running_machine &machine) override
	{
		m_machine = &machine;
		machine.add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate(&core_headless_osd::on_exit, this));
		if (m_network_enabled)
		{
#if defined(OSD_NET_USE_SLIRP)
			try
			{
				m_modules.register_module(NETDEV_SLIRP);
				m_network = &m_modules.select_module<netdev_module>(
						*this, m_options, OSD_NETDEV_PROVIDER, "slirp");
			}
			catch (...)
			{
				m_network = nullptr;
			}
			if (network_status_callback) network_status_callback(m_network ? 1 : -1);
#else
			osd_printf_error("DataRover network requested but libslirp support was not built\n");
			if (network_status_callback) network_status_callback(-1);
#endif
		}
		// No OSD window exists headless, so no render target is ever
		// created — but update_and_render derefs ui_target() (render.h:690
		// asserts non-null) and draws into its UI container on every
		// frame_update. Allocate a plain visible target: it gets a real
		// UI container, is never presented anywhere, and costs one
		// container allocation.
		render_target *target = machine.render().target_alloc(nullptr, 0);
		target->set_bounds(480, 320, 1.0F);
	}
	void update(bool skip_redraw) override { (void)skip_redraw; if (m_machine && frame_callback) frame_callback(*m_machine); }
	void input_update(bool relative_reset) override { (void)relative_reset; }
	void check_osd_inputs() override { }
	void set_verbose(bool print_verbose) override { m_verbose = print_verbose; }

	void init_debugger() override { }
	void wait_for_debugger(device_t &device, bool firststop) override
	{
		(void)device;
		(void)firststop;
	}

	bool no_sound() override { return !m_audio_enabled; }
	bool sound_external_per_channel_volume() override { return false; }
	bool sound_split_streams_per_source() override { return false; }
	uint32_t sound_get_generation() override { return 1; }
	osd::audio_info sound_get_information() override
	{
		osd::audio_info info;
		info.m_generation = 1;
		info.m_default_sink = m_audio_enabled ? 1 : 0;
		info.m_default_source = 0;
		if (m_audio_enabled)
		{
			osd::audio_info::node_info sink;
			sink.m_name = "DataRover";
			sink.m_display_name = "DataRover Speaker";
			sink.m_id = 1;
			sink.m_rate = { 48'000, 48'000, 48'000 };
			sink.m_port_names.emplace_back("Front Center");
			sink.m_port_positions.emplace_back(osd::channel_position::FC());
			sink.m_sinks = 1;
			sink.m_sources = 0;
			info.m_nodes.emplace_back(std::move(sink));
		}
		return info;
	}
	uint32_t sound_stream_sink_open(uint32_t node, std::string name, uint32_t rate) override
	{
		(void)node;
		(void)name;
		(void)rate;
		return m_audio_enabled ? 1 : 0;
	}
	uint32_t sound_stream_source_open(uint32_t node, std::string name, uint32_t rate) override
	{
		(void)node;
		(void)name;
		(void)rate;
		return 0;
	}
	void sound_stream_close(uint32_t id) override { (void)id; }
	void sound_stream_sink_update(uint32_t id, const int16_t *buffer, int samples_this_frame) override
	{
		if (m_audio_enabled && id == 1 && m_audio_write)
			m_audio_write(buffer, size_t(std::max(samples_this_frame, 0)));
	}
	void sound_stream_source_update(uint32_t id, int16_t *buffer, int samples_this_frame) override
	{
		(void)id;
		(void)buffer;
		(void)samples_this_frame;
	}
	void sound_stream_set_volumes(uint32_t id, const std::vector<float> &db) override
	{
		(void)id;
		(void)db;
	}
	void sound_begin_update() override { }
	void sound_end_update() override { }

	void customize_input_type_list(std::vector<input_type_entry> &typelist) override { (void)typelist; }

	void add_audio_to_recording(const int16_t *buffer, int samples_this_frame) override
	{
		(void)buffer;
		(void)samples_this_frame;
	}
	std::vector<ui::menu_item> get_slider_list() override { return std::vector<ui::menu_item>(); }

	osd_font::ptr font_alloc() override { return nullptr; }
	bool get_font_families(std::string const &font_path, std::vector<std::pair<std::string, std::string> > &result) override
	{
		(void)font_path;
		(void)result;
		return false;
	}

	bool execute_command(const char *command) override
	{
		(void)command;
		return false;
	}

	std::unique_ptr<osd::midi_input_port> create_midi_input(std::string_view name) override
	{
		(void)name;
		return nullptr;
	}
	std::unique_ptr<osd::midi_output_port> create_midi_output(std::string_view name) override
	{
		(void)name;
		return nullptr;
	}
	std::vector<osd::midi_port_info> list_midi_ports() override
	{
		return std::vector<osd::midi_port_info>();
	}

	std::unique_ptr<osd::network_device> open_network_device(int id, osd::network_handler &handler) override
	{
		return m_network_enabled && m_network ? m_network->open_device(id, handler) : nullptr;
	}
	std::vector<osd::network_device_info> list_network_devices() override
	{
		return m_network_enabled && m_network ? m_network->list_devices() : std::vector<osd::network_device_info>();
	}
	void set_audio_writer(std::function<void(const int16_t *, size_t)> writer)
	{
		m_audio_write = std::move(writer);
	}

private:
	void on_exit() { m_modules.exit(); m_network = nullptr; if (exit_callback) exit_callback(); m_machine = nullptr; }

	osd_options &m_options;
	bool m_network_enabled;
	bool m_audio_enabled;
	osd_module_manager m_modules;
	netdev_module *m_network = nullptr;
	std::function<void(const int16_t *, size_t)> m_audio_write;
	running_machine *m_machine = nullptr;
	bool m_verbose = false;
};

struct pcm_ring
{
	static constexpr size_t capacity = 96'000; // two seconds at 48 kHz
	std::array<int16_t, capacity> samples{};
	std::atomic<uint64_t> read_index{ 0 };
	std::atomic<uint64_t> write_index{ 0 };

	void push(const int16_t *source, size_t frames)
	{
		if (!source) return;
		uint64_t const write = write_index.load(std::memory_order_relaxed);
		uint64_t const read = read_index.load(std::memory_order_acquire);
		size_t const available = capacity - size_t(std::min<uint64_t>(write - read, capacity));
		size_t const count = std::min(frames, available);
		for (size_t index = 0; index < count; ++index)
			samples[(write + index) % capacity] = source[index];
		write_index.store(write + count, std::memory_order_release);
	}

	size_t pop(int16_t *destination, size_t frames)
	{
		if (!destination) return 0;
		uint64_t const read = read_index.load(std::memory_order_relaxed);
		uint64_t const write = write_index.load(std::memory_order_acquire);
		size_t const count = std::min<uint64_t>(frames, write - read);
		for (size_t index = 0; index < count; ++index)
			destination[index] = samples[(read + index) % capacity];
		read_index.store(read + count, std::memory_order_release);
		return count;
	}

	void clear()
	{
		read_index.store(write_index.load(std::memory_order_acquire), std::memory_order_release);
	}
};

struct datarover_core
{
	std::unique_ptr<osd_options> options;
	std::unique_ptr<core_headless_osd> osd;
	std::thread worker;
	std::mutex mutex;
	std::condition_variable ready_cv;
	bool ready = false;
	bool boot_failed = false;
	std::atomic<bool> stop_requested{ false };
	std::atomic<bool> cold_boot_requested{ false };
	std::atomic<bool> paused{ false }, save_requested{ false }, restart_requested{ false };
	std::atomic<int> save_status{ 0 };
	std::atomic<unsigned> option_mask{ 0 };
	std::atomic<uint64_t> frame_revision{ 0 };
	std::atomic<int> network_status{ 0 };
	pcm_ring audio;
	std::condition_variable control_cv;
	std::string checkpoint_path;
	std::string nvram_machine_path;
	bool restore_attempted = false;
	bool restored_checkpoint = false;
	bool restore_touch_pending = false;
	std::array<uint8_t, DATAROVER_FB_SIZE> restore_touch_frame{};
	std::chrono::steady_clock::time_point restore_touch_deadline{};
	unsigned applied_option = 0;
	ioport_field *option_button = nullptr;
	std::chrono::steady_clock::time_point last_checkpoint = std::chrono::steady_clock::now();
	bool active = false;
	bool frame_valid = false;
	std::array<uint8_t, DATAROVER_FB_SIZE> frame{};
	struct pen_event { int x, y; bool down; };
	std::deque<pen_event> pen_events;
	int last_pen_x = 0, last_pen_y = 0;
	// The guest samples touchscreen state from its event loop, so a host click
	// must remain down long enough to be observed even when its up event arrives
	// before the next guest poll.
	bool pen_is_down = false;
	bool pen_up_pending = false;
	std::chrono::steady_clock::time_point pen_down_since{};
	std::string slave_path;
	// In-process PCLink transport. Preferred over the PTY slave path wherever
	// openpty is unavailable (iOS) and used by the native shells on both
	// platforms; wired into the null_modem card on the emulation thread.
	std::shared_ptr<rs232_host_channel> host_serial;
	std::atomic<int> install_progress{ -1 };
	std::atomic<double> emulated_seconds{ 0.0 };
	int run_result = EMU_ERR_NONE;
	// Device/interface pointers resolved once on the worker thread after
	// boot (never string-looked-up on the caller thread: the tagmap
	// unordered_map races teardown and crashes in its deallocator).
	device_memory_interface *memintf = nullptr;
	ioport_field *pen_x = nullptr;
	ioport_field *pen_y = nullptr;
	ioport_field *pen_button = nullptr;
	// rs2321 slot + card resolved once on the worker thread at boot; the
	// caller thread must never subdevice()-lookup (same tagmap race).
	device_slot_interface *rs2321_slot = nullptr;
	device_t *rs2321_card = nullptr;
};

// One live handle per process: mame_machine_manager::instance() binds a single
// options set process-wide, so a second concurrent create must fail.
std::atomic<datarover_core *> s_live{ nullptr };

// Caller thread (same split as Lua automation / pulsePort): inject via
// ioport_field::set_value. Guest coords scale to the PORT_MINMAX(0,0xffff)
// axis exactly like the harness (floor(x * 0xffff / 479)). Analog set_value
// latches the adjoverride the live read consults, so the value holds until
void inject_pen(datarover_core *core, int x, int y, bool button)
{
	if (!core) return;
	std::lock_guard<std::mutex> lock(core->mutex);
	if (!core->active) return;
	if (x < 0) { x = core->last_pen_x; y = core->last_pen_y; }
	x = std::clamp(x, 0, DATAROVER_FB_WIDTH - 1);
	y = std::clamp(y, 0, DATAROVER_FB_HEIGHT - 1);
	core->last_pen_x = x; core->last_pen_y = y;
	// Coalesce motion, but retain button transitions between video frames.
	if (!core->pen_events.empty() && core->pen_events.back().down == button)
		core->pen_events.back() = {x, y, button};
	else {
		if (core->pen_events.size() >= 256) core->pen_events.pop_front();
		core->pen_events.push_back({x, y, button});
	}
}

// The guest only offers PCLink when it sees a Magic Bus accessory: without it
// the Storeroom computer reports "can't link to a computer" and never writes a
// byte, so no handshake can start. Seed the input configuration once so an app
// container behaves like the regression harness, which configures exactly this
// entry (tools/pclink_regression.py). MAME reads the game config independently
// of OPTION_READCONFIG, which only governs INI files.
void seed_magicbus_config(const char *cfg_dir)
{
	if (!cfg_dir || !*cfg_dir)
		return;
	const std::string path = std::string(cfg_dir) + "/datarover840.cfg";
	struct stat st{};
	if (::stat(path.c_str(), &st) == 0)
		return;
	std::ofstream out(path);
	if (!out)
		return;
	out << "<?xml version=\"1.0\"?>\n"
		"<mameconfig version=\"10\">\n"
		"    <system name=\"datarover840\">\n"
		"        <input>\n"
		"            <port tag=\":MAGICBUS_ACCESSORY\" type=\"CONFIG\" mask=\"1\" defvalue=\"1\" value=\"1\" />\n"
		"        </input>\n"
		"    </system>\n"
		"</mameconfig>\n";
}

// The caller passes the boot-cached slot + card: resolving "rs2321" via
// subdevice() on the caller thread races device-tree teardown
// (unordered_map deallocator) and faults exactly like the framebuffer path.
std::string pty_slave_path(device_slot_interface *slot, device_t *card)
{
	if (!slot || !card)
		return std::string();
	device_pty_interface *pty = dynamic_cast<device_pty_interface *>(card);
	if (!pty || !pty->is_slave_connected())
		return std::string();
	return pty->slave_name();
}

// Bounded write: the emulation thread drains the PTY master continuously, so
// this normally completes at once; the deadline only bites when the guest is
// not listening (wrong screen), turning a hang into a reported failure.
bool write_all_fd(int fd, const uint8_t *data, size_t len)
{
	size_t off = 0;
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
	while (off < len)
	{
		ssize_t n = ::write(fd, data + off, len - off);
		if (n > 0)
		{
			off += static_cast<size_t>(n);
			continue;
		}
		if (n < 0 && errno != EINTR && errno != EAGAIN)
			return false;
		if (std::chrono::steady_clock::now() >= deadline)
			return false;
		struct pollfd pfd{ fd, POLLOUT, 0 };
		::poll(&pfd, 1, 50);
	}
	return true;
}

void teardown_handle(datarover_core *core)
{
	if (core->worker.joinable())
	{
		{
			std::lock_guard<std::mutex> lock(core->mutex);
			core->stop_requested.store(true, std::memory_order_release);
		}
		core->control_cv.notify_all();
		core->worker.join();
	}
	delete mame_machine_manager::instance();
	datarover_core *expected = core;
	s_live.compare_exchange_strong(expected, nullptr, std::memory_order_acq_rel);
	delete core;
}

} // namespace

size_t datarover_framebuffer_size(void)
{
	return DATAROVER_FB_SIZE;
}

const uint8_t *datarover_framebuffer_bytes(void *machine)
{
	if (!machine)
		return nullptr;
	// Dual-mode tap: datarover_core* (iOS headless, worker-cached devices)
	// vs running_machine* (SDL parity shim, called on the emulation thread
	// where subdevice() is safe). Detect by probing the live registry: a
	// registered datarover_core* handle takes the cached path, anything
	// else is treated as a direct running_machine pointer.
	datarover_core *core = static_cast<datarover_core *>(machine);
	running_machine *m = nullptr;
	device_memory_interface *memintf = nullptr;
	// A plain load never mutates the registry (compare_exchange would
	// rewrite s_live on mismatch). Match => core handle; else the caller
	// passed a running_machine directly (SDL parity shim, emulation thread
	// where subdevice() is safe).
	if (s_live.load(std::memory_order_acquire) == core)
	{
		// The returned copy belongs to this calling thread. The emulator may
		// stop or produce another frame without invalidating these bytes.
		thread_local std::array<uint8_t, DATAROVER_FB_SIZE> snapshot;
		std::lock_guard<std::mutex> lock(core->mutex);
		if (!core->active || !core->frame_valid) return nullptr;
		snapshot = core->frame;
		return snapshot.data();
	}
	else
	{
		m = static_cast<running_machine *>(machine);
		if (device_t *cpu = m->root_device().subdevice("maincpu"))
			cpu->interface(memintf);
	}
	if (!m || !memintf)
		return nullptr;
	address_space &space = memintf->space(AS_PROGRAM);
	uint32_t base = space.read_dword(DINO_MMIO_BASE + DINO_VIDEO_HIGH_BUFFER_OFF) & 0xffff'fff0U;
	if (base > (DRAM_LIMIT - DATAROVER_FB_SIZE))
		base = FALLBACK_BASE;
	return static_cast<const uint8_t *>(space.get_read_ptr(base));
}

void *datarover_create_with_options(const char *nvram_dir, const char *cfg_dir, const char *rom_path,
		const datarover_create_options *create_options)
{
	const bool network_enabled = create_options
			&& create_options->struct_size >= sizeof(datarover_create_options)
			&& create_options->network_enabled;
	const bool audio_enabled = create_options
			&& create_options->struct_size >= sizeof(datarover_create_options)
			&& create_options->audio_output_enabled;
	auto core = std::make_unique<datarover_core>();
	core->options = std::make_unique<osd_options>();

	emu_options &opts = *core->options;
	try
	{
		opts.set_system_name("datarover840");
	}
	catch (...)
	{
		return nullptr;
	}
	const int prio = OPTION_PRIORITY_CMDLINE;
	if (nvram_dir && *nvram_dir)
		opts.set_value(OPTION_NVRAM_DIRECTORY, nvram_dir, prio);
	if (cfg_dir && *cfg_dir)
		opts.set_value(OPTION_CFG_DIRECTORY, cfg_dir, prio);
	seed_magicbus_config(cfg_dir);
	if (rom_path && *rom_path)
	{
		// OPTION_MEDIAPATH is a semicolon-separated search DIRECTORY list:
		// the loader opens <dir>/datarover840/magiccap-usa.image itself.
		// The iOS app passes the imported file (…/datarover840/<name>),
		// so normalize file paths to their parent set directory and strip
		// any trailing filename; directory input passes through unchanged.
		std::string media(rom_path);
		struct stat st{};
		if (::stat(media.c_str(), &st) == 0 && !S_ISDIR(st.st_mode))
		{
			const size_t slash = media.find_last_of("/\\");
			if (slash != std::string::npos)
				media.erase(slash);
			const size_t set_slash = media.find_last_of("/\\");
			if (set_slash != std::string::npos
				&& media.compare(set_slash + 1, std::string::npos, "datarover840") == 0)
				media.erase(set_slash);
		}
		if (!media.empty())
			opts.set_value(OPTION_MEDIAPATH, media.c_str(), prio);
	}
	// Headless defaults matching the regression harness (-video none -sound
	// none, no INI side effects, no Lua/plugins/debugger, no UI pauses,
	// real-time emulation with sleeping between frames).
	opts.set_value(OPTION_READCONFIG, 0, prio);
	opts.set_value(OPTION_WRITECONFIG, 0, prio);
	opts.set_value(OPTION_PLUGINS, 0, prio);
	opts.set_value(OPTION_CONSOLE, 0, prio);
	opts.set_value(OPTION_DEBUG, 0, prio);
	opts.set_value(OPTION_THROTTLE, 1, prio);
	opts.set_value(OPTION_SLEEP, 1, prio);
	opts.set_value(OPTION_SKIP_GAMEINFO, 1, OPTION_PRIORITY_MAXIMUM);
	// MAXIMUM priority: nothing downstream may re-arm startup screens.
	opts.set_value(OSDOPTION_VIDEO, OSDOPTVAL_NONE, OPTION_PRIORITY_MAXIMUM);
	opts.set_value(OSDOPTION_SOUND, OSDOPTVAL_NONE, OPTION_PRIORITY_MAXIMUM);
	if (network_enabled)
	{
		opts.set_value("networkprovider", "slirp", prio);
		if (::slot_option *ethernet = opts.find_slot_option("pccard1"))
			ethernet->specify("3c589");
		core->network_status.store(-1);
	}
	core->osd = std::make_unique<core_headless_osd>(
			*core->options, network_enabled, audio_enabled);
	// Serial card: the desktop harness uses an external PTY here and
	// install_package writes to that card's slave side — but iOS sandboxes
	// /dev/ptmx (the log's deny(1) file-read-data), so openpty fails, the
	// card never opens, and boot throws at device start. Default to
	// null_modem (same slot, no PTY): boot proceeds, package install
	// reports failure instead of crashing.
	if (::slot_option *rs2321 = opts.find_slot_option("rs2321"))
		rs2321->specify("null_modem");
	// The in-process PCLink channel exists before the machine starts; the card
	// receives it once the emulation thread has resolved the slot.
	core->host_serial = std::make_shared<rs232_host_channel>();
	core->osd->set_audio_writer([ring = &core->audio](const int16_t *samples, size_t frames)
		{
			ring->push(samples, frames);
		});
	core->osd->network_status_callback = [status = &core->network_status](int value)
		{
			status->store(value, std::memory_order_release);
		};

	core->checkpoint_path = std::string(cfg_dir && *cfg_dir ? cfg_dir : ".") + "/session.sta";
	core->nvram_machine_path = std::string(nvram_dir && *nvram_dir ? nvram_dir : ".") + "/datarover840";
	datarover_core *handle = core.release();
	datarover_core *expected = nullptr;
	if (!s_live.compare_exchange_strong(expected, handle, std::memory_order_acq_rel))
	{
		delete handle;
		return nullptr;
	}
	try
	{
		handle->worker = std::thread([handle]()
		{
			auto finished = [handle]() {
				std::lock_guard<std::mutex> lock(handle->mutex);
				handle->active = false;
				handle->frame_valid = false;
				handle->slave_path.clear();
				handle->pen_events.clear();
				if (!handle->ready) handle->boot_failed = true;
				handle->ready = true;
				handle->ready_cv.notify_all();
			};
			try
			{
				auto *manager = mame_machine_manager::instance(*handle->options, *handle->osd);
				manager->start_http_server();
				int index = driver_list::find("datarover840");
				if (index < 0) { finished(); return; }
				for (;;)
				{
					handle->restore_attempted = false;
					handle->restored_checkpoint = false;
					handle->restore_touch_pending = false;
					handle->memintf = nullptr;
					handle->pen_x = handle->pen_y = handle->pen_button = nullptr;
					handle->option_button = nullptr;
					handle->rs2321_slot = nullptr;
					handle->rs2321_card = nullptr;
					handle->pen_is_down = false;
					handle->pen_up_pending = false;
					machine_config config(driver_list::driver(index), *handle->options);
				running_machine machine(config, *manager);
				// Match mame_machine_manager::execute: Lua reads the manager's
				// machine during start(). Unregister before destruction on all exits.
				manager->set_machine(&machine);
				struct registration_guard {
					mame_machine_manager *manager;
					~registration_guard() { manager->set_machine(nullptr); }
				} registration{manager};
				handle->osd->exit_callback = finished;
				handle->osd->frame_callback = [handle](running_machine &m) {
					// Guest-visible schedule source for callers: device
					// behaviour is defined in emulated time, not host time.
					handle->emulated_seconds.store(m.time().as_double());
					if (m.phase() != machine_phase::RUNNING && handle->stop_requested.load()) { m.schedule_exit(); return; }
					// OSD updates also occur during ROM loading/startup UI. Neither
					// the address spaces nor input fields are ready at that point.
					if (m.phase() != machine_phase::RUNNING) return;
					if (!handle->restore_attempted) {
						handle->restore_attempted = true;
						emu_file file(OPEN_FLAG_READ);
						if (!file.open(handle->checkpoint_path)) {
							// Keep a rollback image: a truncated save must not leave
							// half-restored guest RAM or device state behind.
							std::stringstream backup(std::ios::in | std::ios::out | std::ios::binary);
							if (m.save().write_stream(backup) == STATERR_NONE
								&& m.save().read_file(file) != STATERR_NONE) {
								backup.seekg(0);
								m.save().read_stream(backup);
							} else {
								handle->restored_checkpoint = true;
							}
						}
					}
					if (handle->paused.load() || handle->stop_requested.load()) {
						// Never checkpoint a finger or Option control held down.
						if (handle->option_button) handle->option_button->clear_value();
						if (handle->pen_button) handle->pen_button->clear_value();
						handle->pen_is_down = false;
						handle->pen_up_pending = false;
						handle->applied_option = 0;
						std::lock_guard<std::mutex> lock(handle->mutex);
						handle->pen_events.clear();
					}
					const auto now = std::chrono::steady_clock::now();
					if (handle->save_requested.load() || handle->stop_requested.load()
						|| now - handle->last_checkpoint > std::chrono::seconds(60)) {
						if (m.scheduler().can_save()) {
							handle->save_requested.store(false);
							const auto temporary = handle->checkpoint_path + ".tmp";
							emu_file file(OPEN_FLAG_WRITE | OPEN_FLAG_CREATE | OPEN_FLAG_CREATE_PATHS);
							bool ok = !file.open(temporary);
							if (ok) ok = m.save().write_file(file) == STATERR_NONE;
							file.close();
							if (ok) ok = ::rename(temporary.c_str(), handle->checkpoint_path.c_str()) == 0;
							handle->last_checkpoint = now;
							handle->save_status.store(ok ? 2 : -1);
						}
					}
					if (handle->stop_requested.load()) { m.schedule_exit(); return; }
					// A native-shell restart must not call machine::soft_reset here:
					// the DataRover DAC stream can fault while its output buffer is
					// flushed from machine_reset. Exit this machine and construct a
					// fresh one on the same worker instead.
					if (handle->restart_requested.load(std::memory_order_acquire)) { m.schedule_exit(); return; }
					if (handle->paused.load() && !handle->save_requested.load()) {
						std::unique_lock<std::mutex> lock(handle->mutex);
						handle->control_cv.wait(lock, [handle] {
							return !handle->paused.load() || handle->stop_requested.load()
								|| handle->save_requested.load() || handle->restart_requested.load();
						});
						return;
					}
					if (!handle->memintf) {
						if (auto *cpu = m.root_device().subdevice("maincpu")) cpu->interface(handle->memintf);
						if (!handle->memintf || !handle->memintf->has_space(AS_PROGRAM)) return;
						if (auto *port = m.root_device().ioport("OPTION_BUTTON")) handle->option_button = port->field(1);
						if (auto *port = m.root_device().ioport("TOUCH_X")) handle->pen_x = port->field(0xffff);
						if (auto *port = m.root_device().ioport("TOUCH_Y")) handle->pen_y = port->field(0xffff);
						if (auto *port = m.root_device().ioport("TOUCH_BUTTON")) handle->pen_button = port->field(1);
						if (auto *rs = m.root_device().subdevice("rs2321")) {
							handle->rs2321_slot = dynamic_cast<device_slot_interface *>(rs);
							if (handle->rs2321_slot) handle->rs2321_card = handle->rs2321_slot->get_card_device();
							// Hand the in-process channel to the card on the
							// emulation thread: the device never looks it up
							// across threads and no lock covers MAME objects.
							if (auto *nm = dynamic_cast<null_modem_device *>(handle->rs2321_card))
								nm->set_host_channel(handle->host_serial);
						}
					}
					const unsigned option = handle->option_mask.load();
					if (option != handle->applied_option && handle->option_button) {
						if (option) handle->option_button->set_value(1);
						else handle->option_button->clear_value();
						handle->applied_option = option;
					}
					// Apply one queued state per frame so the guest observes both
					// edges of a quick tap, rather than down and up simultaneously.
					datarover_core::pen_event event{};
					bool have_event = false;
					{
						std::lock_guard<std::mutex> lock(handle->mutex);
						if (!handle->pen_events.empty()) {
							event = handle->pen_events.front();
							handle->pen_events.pop_front();
							have_event = true;
						}
					}
					if (have_event) {
						if (handle->pen_x) handle->pen_x->set_value(uint32_t(event.x) * 0xffffU / 479U);
						if (handle->pen_y) handle->pen_y->set_value(uint32_t(event.y) * 0xffffU / 319U);
						if (event.down) {
							if (handle->restored_checkpoint && !handle->restore_touch_pending) {
								std::lock_guard<std::mutex> lock(handle->mutex);
								handle->restore_touch_frame = handle->frame;
								handle->restore_touch_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
								handle->restore_touch_pending = true;
							}
							if (!handle->pen_is_down)
								handle->pen_down_since = std::chrono::steady_clock::now();
							handle->pen_is_down = true;
							handle->pen_up_pending = false;
							if (handle->pen_button) handle->pen_button->set_value(1);
						} else if (handle->pen_is_down) {
							// The guest reliably samples a pen press held for at least
							// half a second. Queue up, then release after that dwell even
							// if the host generated a quick click.
							handle->pen_up_pending = true;
						}
					}
					if (handle->pen_up_pending
							&& std::chrono::steady_clock::now() - handle->pen_down_since >= std::chrono::milliseconds(500)) {
						if (handle->pen_button) handle->pen_button->clear_value();
						handle->pen_is_down = false;
						handle->pen_up_pending = false;
					}
					if (handle->restore_touch_pending && std::chrono::steady_clock::now() >= handle->restore_touch_deadline) {
						bool unchanged;
						{
							std::lock_guard<std::mutex> lock(handle->mutex);
							unchanged = std::memcmp(handle->frame.data(), handle->restore_touch_frame.data(), handle->frame.size()) == 0;
						}
						handle->restore_touch_pending = false;
						if (unchanged) {
							handle->cold_boot_requested.store(true, std::memory_order_release);
							m.schedule_exit();
							return;
						}
					}
					auto &space = handle->memintf->space(AS_PROGRAM);
					uint32_t base = space.read_dword(DINO_MMIO_BASE + DINO_VIDEO_HIGH_BUFFER_OFF) & 0xfffffff0U;
					if (base > DRAM_LIMIT - DATAROVER_FB_SIZE) base = FALLBACK_BASE;
					const auto *bytes = static_cast<const uint8_t *>(space.get_read_ptr(base));
					const auto slave = pty_slave_path(handle->rs2321_slot, handle->rs2321_card);
					std::lock_guard<std::mutex> lock(handle->mutex);
					handle->frame_valid = bytes != nullptr;
					if (bytes && (handle->frame_revision.load() == 0 || std::memcmp(bytes, handle->frame.data(), handle->frame.size()) != 0)) {
						std::copy_n(bytes, handle->frame.size(), handle->frame.begin());
						handle->frame_revision.fetch_add(1);
					}
					handle->slave_path = slave;
					handle->active = true;
					handle->ready = true;
					handle->ready_cv.notify_all();
				};
					handle->run_result = machine.run(true);
					bool const explicit_restart = handle->restart_requested.exchange(false, std::memory_order_acq_rel);
					if (explicit_restart)
						handle->cold_boot_requested.store(true, std::memory_order_release);
					if (handle->cold_boot_requested.exchange(false, std::memory_order_acq_rel))
					{
						const auto stamp = std::chrono::duration_cast<std::chrono::milliseconds>(
								std::chrono::system_clock::now().time_since_epoch()).count();
						const char *backup_kind = explicit_restart ? ".restart-" : ".stuck-";
						const std::string backup_path = handle->checkpoint_path + backup_kind + std::to_string(stamp);
						const std::string nvram_backup_path = handle->nvram_machine_path + backup_kind + std::to_string(stamp);
						bool const checkpoint_backed_up = (::rename(handle->checkpoint_path.c_str(), backup_path.c_str()) == 0);
						bool const checkpoint_missing = !checkpoint_backed_up && errno == ENOENT;
						bool const nvram_backed_up = (::rename(handle->nvram_machine_path.c_str(), nvram_backup_path.c_str()) == 0);
						bool const nvram_missing = !nvram_backed_up && errno == ENOENT;
						if ((checkpoint_backed_up || checkpoint_missing) && (nvram_backed_up || nvram_missing))
						{
							std::lock_guard<std::mutex> lock(handle->mutex);
							handle->frame_valid = false;
							handle->active = false;
							handle->frame_revision.store(0, std::memory_order_release);
							handle->save_requested.store(false, std::memory_order_release);
							handle->save_status.store(0, std::memory_order_release);
							handle->option_mask.store(0, std::memory_order_release);
							handle->pen_events.clear();
							continue;
						}
						if (checkpoint_backed_up)
							::rename(backup_path.c_str(), handle->checkpoint_path.c_str());
						if (nvram_backed_up)
							::rename(nvram_backup_path.c_str(), handle->nvram_machine_path.c_str());
					}
					break;
				}
				finished();
			}
			catch (...) { finished(); }
		});
	}
	catch (...)
	{
		teardown_handle(handle);
		return nullptr;
	}
	{
		std::unique_lock<std::mutex> lock(handle->mutex);
		handle->ready_cv.wait_for(lock, std::chrono::seconds(120), [handle]() { return handle->ready; });
		if (!handle->ready || handle->boot_failed)
		{
			lock.unlock();
			teardown_handle(handle);
			return nullptr;
		}
	}
	return handle;
}

void *datarover_create(const char *nvram_dir, const char *cfg_dir, const char *rom_path)
{
	const datarover_create_options defaults{ sizeof(datarover_create_options), 0, 0 };
	return datarover_create_with_options(nvram_dir, cfg_dir, rom_path, &defaults);
}

void datarover_set_option(void *machine, int side, int pressed)
{
	if (!machine || side < 0 || side > 1) return;
	auto *core = static_cast<datarover_core *>(machine);
	if (pressed) core->option_mask.fetch_or(1U << side);
	else core->option_mask.fetch_and(~(1U << side));
}
void datarover_request_save(void *machine)
{
	if (!machine) return;
	auto *core = static_cast<datarover_core *>(machine);
	{
		std::lock_guard<std::mutex> lock(core->mutex);
		core->save_status.store(1);
		core->save_requested.store(true);
	}
	core->control_cv.notify_all();
}
void datarover_set_paused(void *machine, int paused)
{
	if (!machine) return;
	auto *core = static_cast<datarover_core *>(machine);
	if (paused) inject_pen(core, -1, -1, false);
	{
		std::lock_guard<std::mutex> lock(core->mutex);
		if (paused) {
			core->option_mask.store(0);
			core->save_status.store(1);
			core->save_requested.store(true);
		}
		core->paused.store(paused != 0);
	}
	core->control_cv.notify_all();
}
int datarover_save_status(void *machine)
{
	return machine ? static_cast<datarover_core *>(machine)->save_status.load() : -1;
}
uint64_t datarover_frame_revision(void *machine)
{
	return machine ? static_cast<datarover_core *>(machine)->frame_revision.load() : 0;
}
void datarover_restart(void *machine)
{
	if (!machine) return;
	auto *core = static_cast<datarover_core *>(machine);
	core->audio.clear();
	{
		std::lock_guard<std::mutex> lock(core->mutex);
		core->restart_requested.store(true);
	}
	core->control_cv.notify_all();
}

size_t datarover_audio_read(void *machine, int16_t *samples, size_t frame_capacity)
{
	if (!machine || !samples || !frame_capacity) return 0;
	return static_cast<datarover_core *>(machine)->audio.pop(samples, frame_capacity);
}

void datarover_audio_clear(void *machine)
{
	if (!machine) return;
	static_cast<datarover_core *>(machine)->audio.clear();
}

int datarover_network_status(void *machine)
{
	return machine ? static_cast<datarover_core *>(machine)->network_status.load(std::memory_order_acquire) : -1;
}

void datarover_destroy(void *machine)
{
	if (!machine)
		return;
	teardown_handle(static_cast<datarover_core *>(machine));
}

void datarover_pen_down(void *machine, int x, int y)
{
	if (!machine)
		return;
	inject_pen(static_cast<datarover_core *>(machine), x, y, true);
}

void datarover_pen_move(void *machine, int x, int y)
{
	if (!machine)
		return;
	inject_pen(static_cast<datarover_core *>(machine), x, y, true);
}

void datarover_pen_up(void *machine)
{
	if (!machine)
		return;
	inject_pen(static_cast<datarover_core *>(machine), -1, -1, false);
}

// Package install: port of tools/pclink_send.py main() handshake. The guest
// speaks first (ChMa magic + Cnct request); the host answers Cntd twice
// (Magic Cap requires both), sends SPkg metadata + package stream, waits for
// Pong, then sends GBye. Mirrors pclink_send.py:238-288 exactly, including
// raw PTY mode (configure_raw_pty) and the monotonic deadlines.
// Returns 0 on observed Pong + GBye write; 1 on any timeout/protocol/IO error.
namespace {
// One PCLink conversation needs only three primitives, so the handshake is
// written once and runs over whichever transport the platform can offer: the
// in-process channel (all native shells, the only option on iOS) or the
// desktop PTY slave (the CLI harness).
struct pclink_link
{
	virtual ~pclink_link() = default;
	virtual bool read_available(std::vector<uint8_t> &out) = 0;
	virtual bool write_all(const uint8_t *data, size_t len) = 0;
	virtual void wait(int ms) = 0;
};

bool read_available_fd(int fd, std::vector<uint8_t> &out)
{
	for (;;)
	{
		struct pollfd pfd{ fd, POLLIN, 0 };
		if (::poll(&pfd, 1, 0) <= 0)
			return true;
		uint8_t buf[65536];
		ssize_t n = ::read(fd, buf, sizeof(buf));
		if (n > 0)
			out.insert(out.end(), buf, buf + n);
		else if (n == 0)
			return true;
		else if (errno != EAGAIN && errno != EINTR)
			return false;
	}
}
bool set_raw_fd(int fd)
{
	struct termios attrs;
	if (::tcgetattr(fd, &attrs) != 0)
		return false;
	::cfmakeraw(&attrs);
	attrs.c_cflag |= CS8 | CREAD | CLOCAL;
	attrs.c_cc[VMIN] = 0;
	attrs.c_cc[VTIME] = 0;
	return ::tcsetattr(fd, TCSANOW, &attrs) == 0;
}

// PTY slave: the desktop harness path.
struct fd_link : pclink_link
{
	explicit fd_link(int fd) : m_fd(fd) {}
	~fd_link() override { if (m_fd >= 0) ::close(m_fd); }
	bool read_available(std::vector<uint8_t> &out) override { return read_available_fd(m_fd, out); }
	bool write_all(const uint8_t *data, size_t len) override { return write_all_fd(m_fd, data, len); }
	void wait(int ms) override
	{
		struct pollfd pfd{ m_fd, POLLIN, 0 };
		::poll(&pfd, 1, ms);
	}
	int m_fd;
};

// In-process channel: the guest UART drains it from the emulation thread.
// Writes are nonblocking and can accept a partial buffer, so a full queue is
// retried until the deadline rather than treated as a device failure.
struct channel_link : pclink_link
{
	explicit channel_link(std::shared_ptr<rs232_host_channel> ch) : m_channel(std::move(ch)) {}
	bool read_available(std::vector<uint8_t> &out) override
	{
		uint8_t buf[65536];
		for (;;)
		{
			// host_read drains the device→host queue; device_read would drain
			// the queue this side writes into.
			const size_t n = m_channel->host_read(buf, sizeof(buf));
			if (!n)
				return true;
			out.insert(out.end(), buf, buf + n);
		}
	}
	bool write_all(const uint8_t *data, size_t len) override
	{
		size_t off = 0;
		const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
		while (off < len)
		{
			const size_t n = m_channel->host_write(data + off, len - off);
			if (n)
			{
				off += n;
				continue;
			}
			if (!m_channel->is_open() || std::chrono::steady_clock::now() >= deadline)
				return false;
			std::this_thread::sleep_for(std::chrono::milliseconds(2));
		}
		return true;
	}
	void wait(int ms) override { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
	std::shared_ptr<rs232_host_channel> m_channel;
};
} // namespace
int datarover_install_package_named(void *machine, const uint8_t *data, size_t len,
		const char *filename_utf8)
{
	if (!machine || !data || len == 0 || !filename_utf8 || !*filename_utf8)
		return 1;
	datarover_core *core = static_cast<datarover_core *>(machine);
	// -1 means "nothing in flight"; the caller polls this while the blocking
	// handshake below runs on its own thread.
	auto fail = [core]() { core->install_progress.store(-1); return 1; };
	core->install_progress.store(0);
	std::unique_ptr<pclink_link> link;
	{
		std::lock_guard<std::mutex> lock(core->mutex);
		if (!core->active)
			return fail();
		if (core->host_serial)
		{
			// Reopen only a channel a previous transfer closed. A live channel
			// must keep its queue: the guest speaks first and its connect
			// request can already be waiting before the host starts.
			if (!core->host_serial->is_open())
				core->host_serial->reset();
			link = std::make_unique<channel_link>(core->host_serial);
		}
		else if (!core->slave_path.empty())
		{
			const int fd = ::open(core->slave_path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
			if (fd >= 0 && set_raw_fd(fd))
				link = std::make_unique<fd_link>(fd);
			else if (fd >= 0)
				::close(fd);
		}
	}
	if (!link)
		return fail();
	const char kChMa[4] = { 'C', 'h', 'M', 'a' };
	const char kCnct[4] = { 'C', 'n', 'c', 't' };
	const char kCntd[4] = { 'C', 'n', 't', 'd' };
	const char kSPkg[4] = { 'S', 'P', 'k', 'g' };
	const char kPing[4] = { 'P', 'i', 'n', 'g' };
	const char kPong[4] = { 'P', 'o', 'n', 'g' };
	const char kGBye[4] = { 'G', 'B', 'y', 'e' };
	std::vector<uint8_t> device_wire;
	const auto cnct_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
	for (;;)
	{
		if (!link->read_available(device_wire))
			return fail();
		if (device_wire.size() >= 4
			&& std::memcmp(device_wire.data(), kChMa, 4) == 0)
		{
			pclink_packet pkt;
			size_t consumed = 0;
			if (pclink_decode_packet(device_wire.data() + 4,
					device_wire.size() - 4, pkt, consumed)
				&& std::memcmp(pkt.tag, kCnct, 4) == 0)
			{
				// Drop the consumed request and keep whatever follows it.
				device_wire.erase(device_wire.begin(),
						device_wire.begin() + 4 + consumed);
				break;
			}
		}
		else
		{
			// Bytes can predate this transfer (the guest opens the link when
			// the user reaches the Storeroom computer, which may be before the
			// host is asked to install). Resynchronise on the guest's magic
			// instead of demanding it at offset zero.
			const auto magic = std::search(device_wire.begin(), device_wire.end(),
					kChMa, kChMa + 4);
			if (magic != device_wire.end())
				device_wire.erase(device_wire.begin(), magic);
		}
		if (std::chrono::steady_clock::now() >= cnct_deadline)
			return fail();
		link->wait(50);
	}
	core->install_progress.store(20);
	// WinPCLink sends this acknowledgement twice and Magic Cap requires both,
	// as complete packets (see the CLI harness's host-wire.bin). Duplicating
	// via a self-referential insert would be undefined, so copy first.
	const std::vector<uint8_t> cntd_once = pclink_encode_packet(kCntd, nullptr, 0);
	std::vector<uint8_t> cntd = cntd_once;
	cntd.insert(cntd.end(), cntd_once.begin(), cntd_once.end());
	if (!link->write_all(cntd.data(), cntd.size()))
		return fail();
	// UTF-8 filename -> UTF-16BE + WinPCLink character count (not byte count).
	std::vector<uint8_t> name16;
	uint32_t char_count = 0;
	for (size_t i = 0; filename_utf8[i];)
	{
		uint32_t cp;
		unsigned char c = static_cast<unsigned char>(filename_utf8[i]);
		size_t seqlen;
		if (c < 0x80) { cp = c; seqlen = 1; }
		else if ((c & 0xe0) == 0xc0) { cp = c & 0x1f; seqlen = 2; }
		else if ((c & 0xf0) == 0xe0) { cp = c & 0x0f; seqlen = 3; }
		else if ((c & 0xf8) == 0xf0) { cp = c & 0x07; seqlen = 4; }
		else break;
		for (size_t k = 1; k < seqlen; ++k)
		{
			unsigned char cc = static_cast<unsigned char>(filename_utf8[i + k]);
			if ((cc & 0xc0) != 0x80) { cp = 0xfffd; seqlen = k; break; }
			cp = (cp << 6) | (cc & 0x3f);
		}
		if (cp >= 0x10000) { cp -= 0x10000; name16.push_back(static_cast<uint8_t>(((0xd800 | (cp >> 10)) >> 8) & 0xff)); name16.push_back(static_cast<uint8_t>((0xd800 | (cp >> 10)) & 0xff)); name16.push_back(static_cast<uint8_t>(((0xdc00 | (cp & 0x3ff)) >> 8) & 0xff)); name16.push_back(static_cast<uint8_t>((0xdc00 | (cp & 0x3ff)) & 0xff)); char_count += 2; }
		else { name16.push_back(static_cast<uint8_t>((cp >> 8) & 0xff)); name16.push_back(static_cast<uint8_t>(cp & 0xff)); char_count += 1; }
		i += seqlen;
	}
	std::vector<uint8_t> meta = pclink_package_metadata(static_cast<uint32_t>(len), name16, char_count);
	std::vector<uint8_t> meta_wire = pclink_encode_packet(kSPkg, meta.data(), meta.size());
	if (!link->write_all(meta_wire.data(), meta_wire.size()))
		return fail();
	core->install_progress.store(30);
	std::vector<uint8_t> stream_in(len + 4, 0);
	std::memcpy(stream_in.data(), data, len);
	std::vector<uint8_t> stream = pclink_encode_crc_stream(stream_in.data(), stream_in.size());
	if (!link->write_all(stream.data(), stream.size()))
		return fail();
	core->install_progress.store(90);
	std::vector<uint8_t> ping = pclink_encode_packet(kPing, nullptr, 0);
	std::vector<uint8_t> pong = pclink_encode_packet(kPong, nullptr, 0);
	std::vector<uint8_t> gbye = pclink_encode_packet(kGBye, nullptr, 0);
	if (!link->write_all(ping.data(), ping.size()))
		return fail();
	bool pong_seen = false;
	const auto pong_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
	for (;;)
	{
		if (!link->read_available(device_wire))
			return fail();
		if (std::search(device_wire.begin(), device_wire.end(),
				pong.begin(), pong.end()) != device_wire.end())
		{
			pong_seen = true;
			break;
		}
		if (std::chrono::steady_clock::now() >= pong_deadline)
			break;
		link->wait(50);
	}
	if (!pong_seen)
		return fail();
	if (!link->write_all(gbye.data(), gbye.size()))
		return fail();
	core->install_progress.store(100);
	return 0;
}

int datarover_install_progress(void *machine)
{
	if (!machine)
		return -1;
	return static_cast<datarover_core *>(machine)->install_progress.load();
}

double datarover_emulated_seconds(void *machine)
{
	if (!machine)
		return 0.0;
	return static_cast<datarover_core *>(machine)->emulated_seconds.load();
}

int datarover_install_package(void *machine, const uint8_t *data, size_t len)
{
	return datarover_install_package_named(machine, data, len, "package.pkg");
}
