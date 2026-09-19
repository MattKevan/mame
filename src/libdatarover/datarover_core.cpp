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
// guest's UART-A/RS-232 endpoint in-process via the rs2321 PTY card's slave
// side (docs/pclink.md: Storeroom computer over Dino UART A 19200 8N1,
// exposed as MAME RS-232 port 1; harness uses -rs2321 pty).
//
// Threading contract: the emulation thread exclusively accesses MAME objects.
// Caller threads enqueue pen states and read locked framebuffer snapshots or
// a cached PTY path. destroy() requests exit and joins the worker before freeing
// the handle. Callers must finish other API calls before destroy(). Framebuffer
// snapshots remain valid until the next framebuffer call on that caller thread.

#include "libdatarover/datarover_core.h"

#include "emu.h"

#include "osdepend.h"
#include "main.h"
#include "frontend/mame/mame.h"
#include "emuopts.h"
#include "modules/lib/osdobj_common.h"
#include "drivenum.h"
#include "gamedrv.h"
#include "mconfig.h"
#include "ioport.h"
#include "dipty.h"
#include "dislot.h"
#include "frontend/mame/ui/menuitem.h"
#include "render.h"
#include "fileio.h"

#include <algorithm>
#include <atomic>
#include <array>
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
bool pclink_decode_packet(const uint8_t *wire, size_t wire_len, pclink_packet &out)
{
	std::vector<uint8_t> encoded;
	encoded.reserve(wire_len);
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
	}
	bool escaped = false;
	std::vector<uint8_t> stream;
	stream.reserve(encoded.size());
	for (uint8_t v : encoded)
	{
		if (escaped)
		{
			if (v != 0x0e && v != 0x0f && v != 0x10)
				return false;
			stream.push_back(v);
			escaped = false;
		}
		else if (v == 0x10)
			escaped = true;
		else if (v == 0x0e || v == 0x0f)
			return false;
		else
			stream.push_back(v);
	}
	if (escaped || stream.size() < 8)
		return false;
	const size_t declared = (static_cast<size_t>(stream[4]) << 24)
		| (static_cast<size_t>(stream[5]) << 16)
		| (static_cast<size_t>(stream[6]) << 8)
		| stream[7];
	if (stream.size() != declared + 8)
		return false;
	std::memcpy(out.tag, stream.data(), 4);
	out.payload.assign(stream.begin() + 8, stream.end());
	return true;
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
	core_headless_osd() = default;
	std::function<void(running_machine &)> frame_callback;
	std::function<void()> exit_callback;

	void init(running_machine &machine) override
	{
		m_machine = &machine;
		machine.add_notifier(MACHINE_NOTIFY_EXIT, machine_notify_delegate(&core_headless_osd::on_exit, this));
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

	bool no_sound() override { return true; }
	bool sound_external_per_channel_volume() override { return false; }
	bool sound_split_streams_per_source() override { return false; }
	uint32_t sound_get_generation() override { return 1; }
	osd::audio_info sound_get_information() override
	{
		osd::audio_info info;
		info.m_generation = 1;
		info.m_default_sink = 0;
		info.m_default_source = 0;
		return info;
	}
	uint32_t sound_stream_sink_open(uint32_t node, std::string name, uint32_t rate) override
	{
		(void)node;
		(void)name;
		(void)rate;
		return 0;
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
		(void)id;
		(void)buffer;
		(void)samples_this_frame;
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
		(void)id;
		(void)handler;
		return nullptr;
	}
	std::vector<osd::network_device_info> list_network_devices() override
	{
		return std::vector<osd::network_device_info>();
	}

private:
	void on_exit() { if (exit_callback) exit_callback(); m_machine = nullptr; }

	running_machine *m_machine = nullptr;
	bool m_verbose = false;
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
	std::atomic<bool> paused{ false }, save_requested{ false }, restart_requested{ false };
	std::atomic<int> save_status{ 0 };
	std::atomic<unsigned> option_mask{ 0 };
	std::atomic<uint64_t> frame_revision{ 0 };
	std::condition_variable control_cv;
	std::string checkpoint_path;
	bool restore_attempted = false;
	unsigned applied_option = 0;
	ioport_field *option_button = nullptr;
	std::chrono::steady_clock::time_point last_checkpoint = std::chrono::steady_clock::now();
	bool active = false;
	bool frame_valid = false;
	std::array<uint8_t, DATAROVER_FB_SIZE> frame{};
	struct pen_event { int x, y; bool down; };
	std::deque<pen_event> pen_events;
	int last_pen_x = 0, last_pen_y = 0;
	std::string slave_path;
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

void *datarover_create(const char *nvram_dir, const char *cfg_dir, const char *rom_path)
{
	auto core = std::make_unique<datarover_core>();
	core->options = std::make_unique<osd_options>();
	core->osd = std::make_unique<core_headless_osd>();

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
	// Serial card: the desktop harness uses an external PTY here and
	// install_package writes to that card's slave side — but iOS sandboxes
	// /dev/ptmx (the log's deny(1) file-read-data), so openpty fails, the
	// card never opens, and boot throws at device start. Default to
	// null_modem (same slot, no PTY): boot proceeds, package install
	// reports failure instead of crashing.
	if (::slot_option *rs2321 = opts.find_slot_option("rs2321"))
		rs2321->specify("null_modem");

	core->checkpoint_path = std::string(cfg_dir && *cfg_dir ? cfg_dir : ".") + "/session.sta";
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
							}
						}
					}
					if (handle->paused.load() || handle->stop_requested.load()) {
						// Never checkpoint a finger or Option control held down.
						if (handle->option_button) handle->option_button->clear_value();
						if (handle->pen_button) handle->pen_button->clear_value();
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
					if (handle->restart_requested.exchange(false)) m.schedule_soft_reset();
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
						if (handle->pen_button) { if (event.down) handle->pen_button->set_value(1); else handle->pen_button->clear_value(); }
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
	{
		std::lock_guard<std::mutex> lock(core->mutex);
		core->restart_requested.store(true);
	}
	core->control_cv.notify_all();
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
} // namespace
int datarover_install_package_named(void *machine, const uint8_t *data, size_t len,
		const char *filename_utf8)
{
	if (!machine || !data || len == 0 || !filename_utf8 || !*filename_utf8)
		return 1;
	datarover_core *core = static_cast<datarover_core *>(machine);
	std::string slave;
	{ std::lock_guard<std::mutex> lock(core->mutex);
	  if (!core->active) return 1;
	  slave = core->slave_path; }
	if (slave.empty())
		return 1;
	const int fd = ::open(slave.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0)
		return 1;
	if (!set_raw_fd(fd))
	{
		::close(fd);
		return 1;
	}
	const char kChMa[4] = { 'C', 'h', 'M', 'a' };
	const char kCnct[4] = { 'C', 'n', 'c', 't' };
	const char kCntd[4] = { 'C', 'n', 't', 'd' };
	const char kSPkg[4] = { 'S', 'P', 'k', 'g' };
	const char kPing[4] = { 'P', 'i', 'n', 'g' };
	const char kPong[4] = { 'P', 'o', 'n', 'g' };
	const char kGBye[4] = { 'G', 'B', 'y', 'e' };
	std::vector<uint8_t> device_wire;
	size_t connect_len = 0;
	const auto cnct_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
	for (;;)
	{
		if (!read_available_fd(fd, device_wire))
		{
			::close(fd);
			return 1;
		}
		if (device_wire.size() >= 4
			&& std::memcmp(device_wire.data(), kChMa, 4) == 0)
		{
			pclink_packet pkt;
			if (pclink_decode_packet(device_wire.data() + 4,
					device_wire.size() - 4, pkt)
				&& std::memcmp(pkt.tag, kCnct, 4) == 0)
			{
				connect_len = device_wire.size();
				break;
			}
		}
		if (std::chrono::steady_clock::now() >= cnct_deadline)
		{
			::close(fd);
			return 1;
		}
		struct pollfd pfd{ fd, POLLIN, 0 };
		::poll(&pfd, 1, 50);
	}
	std::vector<uint8_t> cntd = pclink_encode_packet(kCntd, nullptr, 0);
	cntd.insert(cntd.end(), cntd.begin(), cntd.begin() + cntd.size() / 2);
	if (!write_all_fd(fd, cntd.data(), cntd.size()))
	{
		::close(fd);
		return 1;
	}
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
	if (!write_all_fd(fd, meta_wire.data(), meta_wire.size()))
	{
		::close(fd);
		return 1;
	}
	std::vector<uint8_t> stream_in(len + 4, 0);
	std::memcpy(stream_in.data(), data, len);
	std::vector<uint8_t> stream = pclink_encode_crc_stream(stream_in.data(), stream_in.size());
	if (!write_all_fd(fd, stream.data(), stream.size()))
	{
		::close(fd);
		return 1;
	}
	std::vector<uint8_t> ping = pclink_encode_packet(kPing, nullptr, 0);
	std::vector<uint8_t> pong = pclink_encode_packet(kPong, nullptr, 0);
	std::vector<uint8_t> gbye = pclink_encode_packet(kGBye, nullptr, 0);
	if (!write_all_fd(fd, ping.data(), ping.size()))
	{
		::close(fd);
		return 1;
	}
	bool pong_seen = false;
	const auto pong_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
	for (;;)
	{
		if (!read_available_fd(fd, device_wire))
		{
			::close(fd);
			return 1;
		}
		if (device_wire.size() > connect_len
			&& std::search(device_wire.begin() + connect_len, device_wire.end(),
				pong.begin(), pong.end()) != device_wire.end())
		{
			pong_seen = true;
			break;
		}
		if (std::chrono::steady_clock::now() >= pong_deadline)
			break;
		struct pollfd pfd{ fd, POLLIN, 0 };
		::poll(&pfd, 1, 50);
	}
	if (!pong_seen)
	{
		::close(fd);
		return 1;
	}
	const bool ok = write_all_fd(fd, gbye.data(), gbye.size());
	::close(fd);
	return ok ? 0 : 1;
}

int datarover_install_package(void *machine, const uint8_t *data, size_t len)
{
	return datarover_install_package_named(machine, data, len, "package.pkg");
}
