// datarover_shim.h — SDL-side entry points for the libdatarover parity gate.
//
// The shim is NOT a second main(): main() lives in the SDL OSD
// (src/osd/sdl3/sdlmain.cpp). These hooks run inside the normal OSD boot
// path for the datarover driver only, so the SDL frontend keeps working
// unchanged while the core ABI proves pixel parity.
#pragma once

// OSD init hook: arms the framebuffer dump when DATAROVER_CORE_DUMP is set.
// OSD frame hook: counts emulated frames; at the target it dumps the guest
// 2bpp framebuffer bytes to the dump path and requests a clean exit.
// Both are no-ops unless DATAROVER_CORE_DUMP names a writable file path.
void datarover_shim_osd_init(void);
void datarover_shim_frame_hook(void);
