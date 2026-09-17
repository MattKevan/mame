// datarover_core.h — portable C ABI for libdatarover.
#pragma once
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define DATAROVER_FB_WIDTH 480
#define DATAROVER_FB_HEIGHT 320
#define DATAROVER_FB_SIZE (480 * 320 / 4)  // 2bpp, matches FRAME_BYTES

// Framebuffer access (valid after machine boot; pointer owned by core).
const uint8_t *datarover_framebuffer_bytes(void *machine);
size_t datarover_framebuffer_size(void);
// Lifecycle + input + package + paths land in Task 2; declared here:
void *datarover_create(const char *nvram_dir, const char *cfg_dir, const char *rom_path);
void datarover_destroy(void *machine);
void datarover_pen_down(void *machine, int x, int y);
void datarover_pen_move(void *machine, int x, int y);
void datarover_pen_up(void *machine);
int datarover_install_package(void *machine, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
