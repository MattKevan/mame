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
// The pointer may be NULL: get_read_ptr() returns NULL for non-RAM-backed
// mappings (unmapped/ROM/device handlers) and pre-boot, mirroring the
// scanout power-gated black-fill path. The caller must null-check.
const uint8_t *datarover_framebuffer_bytes(void *machine);
size_t datarover_framebuffer_size(void);
// Lifecycle + input + package + paths land in Task 2; declared here:
void *datarover_create(const char *nvram_dir, const char *cfg_dir, const char *rom_path);
void datarover_destroy(void *machine);
void datarover_pen_down(void *machine, int x, int y);
void datarover_pen_move(void *machine, int x, int y);
void datarover_pen_up(void *machine);
int datarover_install_package(void *machine, const uint8_t *data, size_t len);
// Named variant: filename controls the guest-visible Storeroom name.
// Character count (not byte count) is recorded per WinPCLink; non-ASCII
// names are encoded UTF-16BE with a proper character count.
int datarover_install_package_named(void *machine, const uint8_t *data, size_t len, const char *filename_utf8);

#ifdef __cplusplus
}
#endif
