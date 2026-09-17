// datarover_shim.cpp — SDL shim over the libdatarover core ABI.
//
// Integration choice: compile datarover_core.cpp INTO the datarover binary
// (file-list-only addition in scripts/src/osd/sdl3.lua) and expose the ABI
// to an in-process parity dumper. A separate link target was rejected as
// invasive: main() lives in the OSD (src/osd/sdl3/sdlmain.cpp), the
// datarover SUBTARGET build links one emulator binary, and SOURCES-filtered
// driver projects only accept src/mame/** files (makedep
// write_sources_project only emits the src/mame/ subset) — so the OSD
// archive is the minimal home for the core.
//
// The dump path runs INSIDE the normal SDL OSD boot: osdsdl init arms it,
// and the per-frame hook counts video frames (video_manager::frame_update,
// the same point that fires Lua register_frame_done) for the datarover
// driver only. At the target frame it reads the guest 2bpp framebuffer
// THROUGH THE CORE ABI — datarover_framebuffer_bytes() +
// datarover_framebuffer_size(), null-checked per the header contract — and
// writes the raw bytes to disk, then requests a clean exit. Reference and
// shim runs use identical ROM/NVRAM/cfg and the same frame count, so the
// gate proves the ABI tap returns the same guest memory the driver renders
// (the Lua reference side reads the same words via program:read_u32,
// packed little-endian to match the raw byte order).
//
// The OSD call sites pass the live running_machine directly (init takes it
// as a parameter; update reaches it via machine()) — the shim includes only
// emu.h plus the core ABI header, no frontend headers.
//
// Control is via environment, not new CLI flags (no emuopts table churn):
//   DATAROVER_CORE_DUMP=/path/to/fb.bin  arm + set the dump path
//   DATAROVER_CORE_FRAMES=N              target frame (default 600)
//
// Failure semantics: a NULL tap pointer (pre-boot / non-RAM-backed mapping
// per the header contract) or a short write prints to stderr and exits
// WITHOUT writing a dump file, so the gate reports a missing-dump error
// (exit 2) — never a silent PARITY OK.
//
// Framebuffer-only scope (plan Task 3): Task 2's known gaps — Cntd
// double-ack omitted, PTY slave fd never set raw — do not block this gate
// and no package delivery is claimed here.

#include "emu.h"

#include "../libdatarover/datarover_core.h"
#include "../libdatarover/datarover_shim.h"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

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

void datarover_shim_osd_init(running_machine &machine)
{
	if (!is_datarover(machine))
		return;
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

void datarover_shim_frame_hook(running_machine &machine)
{
	if (!s_armed || s_dumped)
		return;
	if (!is_datarover(machine))
		return;
	const int frames = s_frame_count.fetch_add(1) + 1;
	if (frames < s_target_frames)
		return;
	s_dumped = true;

	// The actual ABI proof: the dump goes through
	// datarover_framebuffer_bytes()/datarover_framebuffer_size(), not
	// around them. NULL is a legal tap result (pre-boot / non-RAM-backed
	// mapping, mirroring the scanout power-gated black-fill path) — treat
	// it as a gate error, never as empty output.
	const size_t size = datarover_framebuffer_size();
	const uint8_t *bytes = datarover_framebuffer_bytes(&machine);
	if (bytes == nullptr)
	{
		std::fprintf(stderr, "datarover_shim: framebuffer tap returned NULL at frame %d\n", frames);
		machine.schedule_exit();
		return;
	}
	FILE *f = std::fopen(s_dump_path.c_str(), "wb");
	if (f == nullptr)
	{
		std::fprintf(stderr, "datarover_shim: cannot open dump path %s\n", s_dump_path.c_str());
		machine.schedule_exit();
		return;
	}
	const size_t wrote = std::fwrite(bytes, 1, size, f);
	std::fclose(f);
	if (wrote != size)
		std::fprintf(stderr, "datarover_shim: short write (%zu of %zu bytes)\n", wrote, size);
	machine.schedule_exit();
}
