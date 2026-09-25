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

typedef struct datarover_create_options
{
	uint32_t struct_size;
	int32_t network_enabled;
	int32_t audio_output_enabled;
} datarover_create_options;

// For a datarover_create handle, returns a snapshot owned by the calling
// thread, valid until its next framebuffer call. Returns NULL before a
// frame is available or after emulation stops. The SDL shim may instead
// pass a running_machine on its emulation thread and receives live RAM.
// Callers must null-check and must finish all API calls before destroy.
const uint8_t *datarover_framebuffer_bytes(void *machine);
size_t datarover_framebuffer_size(void);
// Lifecycle + input + package + paths land in Task 2; declared here:
void *datarover_create(const char *nvram_dir, const char *cfg_dir, const char *rom_path);
void *datarover_create_with_options(const char *nvram_dir, const char *cfg_dir, const char *rom_path,
		const datarover_create_options *options);
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
// Progress of the install running on the calling thread: 0-100, or -1 when no
// install is in flight. Phase-granular (handshake, metadata, stream, reply),
// so a UI can show meaningful movement during a multi-minute transfer.
int datarover_install_progress(void *machine);
// Emulated seconds since the machine started (0 before the first frame). Guest
// behaviour is scheduled in emulated time, so regressions and any guest clock
// work must use this rather than host wall time.
double datarover_emulated_seconds(void *machine);
// Nonblocking pull of mono signed PCM frames at 48 kHz.
size_t datarover_audio_read(void *machine, int16_t *samples, size_t frame_capacity);
void datarover_audio_clear(void *machine);
// 0: disabled, 1: provider initialized, -1: requested provider unavailable.
int datarover_network_status(void *machine);

#ifdef __cplusplus
}
#endif
