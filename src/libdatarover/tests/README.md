# Headless core lifecycle regression

`lifecycle.cpp` exercises the same `datarover_create` entry point as the iOS
app, rather than the SDL shim. It checks two boot/shutdown cycles, rejection of
a second live core, changing framebuffer content, snapshot lifetime across
frames and destruction, queued pen input, and a missing-ROM failure.

Build DataRoverCore for an arm64 iOS simulator in the companion
magic-cap-emulator Xcode project. Then, from this MAME checkout, set
`CORE_LIBRARY` to the resulting `libDataRoverCore.a`, `SIMULATOR_ID` to a booted
arm64 simulator, and `ROM_PATH` to your legally obtained MagicCap image:

```sh
xcrun --sdk iphonesimulator clang++ -std=c++20 \
  -target arm64-apple-ios16.0-simulator \
  -isysroot "$(xcrun --sdk iphonesimulator --show-sdk-path)" \
  -I src/libdatarover src/libdatarover/tests/lifecycle.cpp "$CORE_LIBRARY" \
  -framework Foundation -framework CoreMIDI -o /tmp/datarover-lifecycle
xcrun simctl spawn "$SIMULATOR_ID" /tmp/datarover-lifecycle "$ROM_PATH" \
  "$(mktemp -d /tmp/datarover-lifecycle.XXXXXX)"
```

Expected: exit status zero, two `PASS boot` lines and
`PASS missing ROM returns null`. Each successful boot runs for 15 seconds.
The test uses its own NVRAM/config directory and does not change app data.
A crash, nonzero exit, or hang is a failure; use a 180-second outer timeout in CI.
The test stresses input delivery but does not assert guest-visible tap behavior;
check that interactively in the app.

## Controls and checkpoints

Compile `controls.cpp` using the same command and library as above, replacing
`lifecycle.cpp` and the output executable name. Pass the ROM and a fresh scratch
directory. It checks pause CPU usage, stable paused frames, checkpoint creation,
restart, reload, shutdown while paused and corrupt-checkpoint fallback. Expected:
`PASS controls, pause, checkpoint, restart, resume and corrupt-save fallback`.
The test does not assert the guest-visible effect of the Option controls.

## In-process package install

Compile `install.cpp` the same way. It takes three arguments — ROM, a fresh
scratch directory, and a `.pkg` — and drives the guest the way the CLI harness
does: welcome tap, the three calibration targets, then the Hallway and
Storeroom. It starts the host-side install *before* tapping the Storeroom
computer, because that tap is what makes the guest send its `ChMa`/`Cnct`
opening exchange (docs/pclink.md).

```sh
xcrun simctl spawn "$SIMULATOR_ID" /tmp/datarover-install \
  "$ROM_PATH" "$(mktemp -d /tmp/datarover-install.XXXXXX)" "$PACKAGE"
```

Expected: `PASS in-process PCLink package install`, exit status zero, about a
minute, with the package counted in the guest's Storeroom storage. A fresh
scratch directory also proves the core seeds the Magic Bus accessory
configuration the guest needs before it will open a link at all; without that
seed the guest never transmits and the test fails after a 180-second timeout.
The test writes `shot-<frame>.raw` snapshots (480x320 2bpp) into the scratch
directory for failures, and destroys its own emulator state.

## Checkpoint scheduler boundary

`checkpoint.cpp` verifies that restoring a paused checkpoint does not strand
CPU execution in overdue screen timers. After the fresh ROM reaches its welcome
screen, the test saves, destroys and recreates the core, requires a touch-driven
frame change within two seconds, and requires restart to complete within three
seconds. Link it like `controls.cpp`, pass the ROM and a fresh scratch directory,
and use a 90-second process timeout (including shutdown). The companion
repository's `tools/test_core_checkpoint.sh` supplies the macOS link flags and
isolated scratch directory.

Expected: `PASS checkpoint restore, guest touch response and prompt restart`.
The old OSD-callback save/load implementation fails the prompt-restart check.

## Host battery inputs

`host_battery.cpp` exercises the actual native worker and Betty ADC path in the
IDT monitor. It includes the core translation unit with a test-only observer;
measurements are taken on the worker and copied under a mutex. The ordinary
library build has no observer or diagnostic ABI. The test covers every integer
percentage, clamping/unavailability, AC state, backup-cell health, isolation
from synthetic charging, an update queued while paused, a completed restart,
and checkpoint recreation without persisting host inputs.

Run companion `tools/test_core_host_battery.sh` with `CORE_LIBRARY` pointing to
the built macOS library, `CORE_BUILD_ARGUMENTS` to that target's C++
`common-args.resp` (the response file containing `-std=c++20` without Objective-C
ARC flags), and `ROM_PATH` to the release ROM. This keeps the fixture's MAME
headers and compile definitions identical to the app. The script creates
isolated state and enforces a 90-second timeout.

## Power-off wake

`power_wake.cpp` powers the guest off through its own power-button path, then
requires a tap to wake it, both live and after restoring a powered-down
checkpoint. It also requires that neither the wake nor later taps on inert
areas trigger stuck-restore recovery. Build it like `host_battery.cpp`; the
companion `tools/test_core_power_wake.sh` does this with a 120-second timeout.

## Host clock bridge

`host_clock.cpp` checks the calendar bridge protocol against fake guest memory,
with no MAME machine: queueing only at the idle loop, the mailbox date and time,
completion after the callback's epilogue, abandoning a callback the guest never
runs, and the ROM guards. Build it standalone with `clang++ -std=c++20`.
Expected: `PASS host clock queueing, completion, abandonment and rejection guards`.
