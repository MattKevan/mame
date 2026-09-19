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

// For a datarover_create handle, returns a snapshot owned by the calling
// thread, valid until its next framebuffer call. Returns NULL before a
// frame is available or after emulation stops. The SDL shim may instead
// pass a running_machine on its emulation thread and receives live RAM.
// Callers must null-check and must finish all API calls before destroy.
const uint8_t *datarover_framebuffer_bytes(void *machine);
size_t datarover_framebuffer_size(void);
// Lifecycle + input + package + paths land in Task 2; declared here:
void *datarover_create(const char *nvram_dir, const char *cfg_dir, const char *rom_path);
void datarover_destroy(void *machine);

// Thread-safe controls; changes are applied by the emulation worker.
void datarover_set_option(void *machine, int side, int pressed);
void datarover_set_paused(void *machine, int paused);
void datarover_request_save(void *machine);
// 0: not saved, 1: pending, 2: saved, -1: failed.
int datarover_save_status(void *machine);
uint64_t datarover_frame_revision(void *machine);
void datarover_restart(void *machine);

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
