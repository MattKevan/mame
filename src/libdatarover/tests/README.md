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
