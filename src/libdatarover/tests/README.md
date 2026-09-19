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
