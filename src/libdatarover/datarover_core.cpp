// datarover_core.cpp — framebuffer tap for libdatarover (Task 1).
//
// Scanout source: datarover_state::screen_update
// (src/mame/skeleton/datarover.cpp). The LCD panel is the "screen" device tag;
// scanout reads 2bpp pixels from guest DRAM at the Dino video-high-buffer
// base register (Dino MMIO 0x10c00000 + 0x030, masked to 0xfffffff0) with
// fallback base 0x003f6a00 when out of the 4 MiB DRAM window.

#include "libdatarover/datarover_core.h"

#include "emu.h"

namespace {

constexpr uint32_t DINO_MMIO_BASE = 0x10c00000U;
constexpr uint32_t DINO_VIDEO_HIGH_BUFFER_OFF = 0x030U;
constexpr uint32_t FALLBACK_BASE = 0x003f6a00U;
constexpr uint32_t DRAM_LIMIT = 0x00400000U;

} // namespace

size_t datarover_framebuffer_size(void)
{
	return DATAROVER_FB_SIZE;
}

const uint8_t *datarover_framebuffer_bytes(void *machine)
{
	if (!machine)
		return nullptr;
	running_machine *const m = static_cast<running_machine *>(machine);
	// LCD panel device tag from the driver finder: m_screen(*this, "screen").
	if (!m->root_device().subdevice("screen"))
		return nullptr;
	device_t *const cpu = m->root_device().subdevice("maincpu");
	if (!cpu)
		return nullptr;
	device_memory_interface *memintf = nullptr;
	if (!cpu->interface(memintf) || !memintf->has_space(AS_PROGRAM))
		return nullptr;
	address_space &space = memintf->space(AS_PROGRAM);
	uint32_t base = space.read_dword(DINO_MMIO_BASE + DINO_VIDEO_HIGH_BUFFER_OFF) & 0xffff'fff0U;
	if (base > (DRAM_LIMIT - DATAROVER_FB_SIZE))
		base = FALLBACK_BASE;
	return static_cast<const uint8_t *>(space.get_read_ptr(base));
}

// Task 2: lifecycle lands here.
void *datarover_create(const char *, const char *, const char *)
{
	return nullptr;
}

// Task 2: lifecycle lands here.
void datarover_destroy(void *)
{
}

// Task 2: pen input lands here.
void datarover_pen_down(void *, int, int)
{
}

// Task 2: pen input lands here.
void datarover_pen_move(void *, int, int)
{
}

// Task 2: pen input lands here.
void datarover_pen_up(void *)
{
}

// Task 2: package install lands here.
int datarover_install_package(void *, const uint8_t *, size_t)
{
	return 0;
}
