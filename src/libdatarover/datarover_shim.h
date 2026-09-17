// datarover_shim.h — SDL-side entry points for the libdatarover parity gate.
//
// The shim is NOT a second main(): main() lives in the SDL OSD
// (src/osd/sdl3/sdlmain.cpp). These hooks run inside the normal OSD boot
// path for the datarover driver only, so the SDL frontend keeps working
// unchanged while the core ABI proves pixel parity.
//
// Both hooks take the live running_machine from the OSD call site
// (init receives it as a parameter; update reaches it via machine()) —
// no manager-singleton lookup, no frontend headers.
#pragma once

class running_machine;

// OSD init hook: arms the framebuffer dump when DATAROVER_CORE_DUMP is set.
// OSD frame hook: counts emulated frames; at the target it dumps the guest
// 2bpp framebuffer bytes via the core ABI to the dump path and requests a
// clean exit. Both are no-ops unless DATAROVER_CORE_DUMP names a path.
void datarover_shim_osd_init(running_machine &machine);
void datarover_shim_frame_hook(running_machine &machine);
