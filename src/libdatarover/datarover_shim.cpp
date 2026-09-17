// datarover_shim.cpp — SDL shim over the libdatarover core ABI.
//
// Integration choice: compile datarover_core.cpp INTO the datarover binary
// (file-list-only addition in scripts/src/osd/sdl3.lua, same pattern as the
// mac-only datarover_menu.mm entry) and expose the ABI to an in-process
// parity dumper. A separate link target was rejected as invasive: main()
// lives in the OSD (src/osd/sdl3/sdlmain.cpp), the datarover SUBTARGET build
// links one emulator binary, and SOURCES-filtered driver projects only
// accept src/mame/** files (makedep write_sources_project only emits the
// src/mame/ subset) — so the OSD archive is the minimal home for the core.
//
// The dump path runs INSIDE the normal SDL OSD boot: osdsdl init arms it,
// and the per-frame hook counts MACHINE_NOTIFY_FRAME-equivalent video frames
// (video_manager::frame_update, the same point that fires Lua
// register_frame_done) for the datarover driver only. At the target frame
// it reads the guest 2bpp framebuffer with the same scanout math as the
// driver's screen_update (Dino video-high-buffer base register masked to
// 0xfffffff0, fallback base 0x003f6a00 outside the 4 MiB DRAM window —
// datarover.cpp screen_update) and writes the raw bytes to disk, then
// requests a clean exit. Reference and shim runs use identical ROM/NVRAM/cfg
// and the same frame count, so the gate proves the ABI scanout path reads
// the same guest memory the driver renders.
//
// Control is via environment, not new CLI flags (no emuopts table churn):
//   DATAROVER_CORE_DUMP=/path/to/fb.bin  arm + set the dump path
//   DATAROVER_CORE_FRAMES=N              target frame (default 600)
//
// Framebuffer-only scope (plan Task 3): Task 2's known gaps — Cntd
// double-ack omitted, PTY slave fd never set raw — do not block this gate
#include "emu.h"
#include "frontend/mame/mame.h"
#include "libdatarover/datarover_shim.h"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

constexpr uint32_t DINO_VIDEO_HIGH_BUFFER_ADDR = 0x10c00030U;
constexpr uint32_t DRAM_LIMIT = 0x00400000U;
constexpr uint32_t FALLBACK_BASE = 0x003f6a00U;
constexpr size_t FB_SIZE = 480 * 320 / 4; // 2bpp, matches FRAME_BYTES

std::atomic<int> s_frame_count{ 0 };
std::string s_dump_path;
int s_target_frames = 600;
bool s_armed = false;
bool s_dumped = false;

bool is_datarover(running_machine &machine)
{
	return std::strncmp(machine.system().name, "datarover", 9) == 0;
}

} // namespace

void datarover_shim_osd_init(void)
{
	const char *dump = std::getenv("DATAROVER_CORE_DUMP");
	if (!dump || !*dump)
		return;
	const char *frames = std::getenv("DATAROVER_CORE_FRAMES");
	if (frames && *frames)
		s_target_frames = std::atoi(frames);
	if (s_target_frames <= 0)
		s_target_frames = 600;
	s_dump_path = dump;
	s_frame_count.store(0);
	s_armed = true;
	s_dumped = false;
}

void datarover_shim_frame_hook(void)
{
	if (!s_armed || s_dumped)
		return;
	// The OSD frame hook runs without machine context; resolve the live
	// machine through the manager singleton, mirroring how the core video
	// frame path reaches emulator_info draw/periodic/frame hooks.
	running_machine *machine = mame_machine_manager::instance()->machine();
	if (machine == nullptr)
		return;
	if (!is_datarover(*machine))
		return;
	const int frames = s_frame_count.fetch_add(1) + 1;
	if (frames < s_target_frames)
		return;
	s_dumped = true;

	device_t *cpu = machine->root_device().subdevice("maincpu");
	uint32_t base = FALLBACK_BASE;
	if (cpu != nullptr)
	{
		device_memory_interface *memintf = nullptr;
		if (cpu->interface(memintf) && memintf->has_space(AS_PROGRAM))
		{
			address_space &space = memintf->space(AS_PROGRAM);
			base = space.read_dword(DINO_VIDEO_HIGH_BUFFER_ADDR) & 0xffff'fff0U;
			if (base > (DRAM_LIMIT - FB_SIZE))
				base = FALLBACK_BASE;
			FILE *f = std::fopen(s_dump_path.c_str(), "wb");
			if (f != nullptr)
			{
				for (size_t off = 0; off < FB_SIZE; off += 4)
				{
					const uint32_t word = space.read_dword(base + static_cast<uint32_t>(off));
					uint8_t be[4];
					be[0] = static_cast<uint8_t>((word >> 24) & 0xff);
					be[1] = static_cast<uint8_t>((word >> 16) & 0xff);
					be[2] = static_cast<uint8_t>((word >> 8) & 0xff);
					be[3] = static_cast<uint8_t>(word & 0xff);
					std::fwrite(be, 1, 4, f);
				}
				std::fclose(f);
			}
		}
	}
	machine->schedule_exit();
}
