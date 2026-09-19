// install.cpp — headless in-process PCLink install regression.
//
// Drives the same guest navigation tools/pclink_regression.py performs from the
// CLI (welcome tap, three calibration points, open the Storeroom computer),
// then installs a package through datarover_install_package_named over the
// in-process rs232_host_channel. The guest's own reply is the acceptance: the
// core only reports success after the guest sent its connect request and the
// final Pong, so a passing run means the guest accepted the stream.
//
// Build and run as documented in this directory's README:
//   xcrun --sdk iphonesimulator clang++ -std=c++20 \
//     -target arm64-apple-ios16.0-simulator \
//     -isysroot "$(xcrun --sdk iphonesimulator --show-sdk-path)" \
//     -I src/libdatarover src/libdatarover/tests/install.cpp "$CORE_LIBRARY" \
//     -framework Foundation -framework CoreMIDI -o /tmp/datarover-install
//   xcrun simctl spawn "$SIMULATOR_ID" /tmp/datarover-install \
//     "$ROM_PATH" "$(mktemp -d /tmp/datarover-install.XXXXXX)" "$PACKAGE"
#include "datarover_core.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <vector>

namespace {
using clk = std::chrono::steady_clock;

// Guest frame schedule copied from the CLI harness's generated Lua, in frames
// at 60 Hz. The guest paces the transfer, so only these taps are timed; the
// install itself waits for the guest to speak (up to its own deadline).
struct tap
{
	int frame;
	int x;
	int y;
};
constexpr tap k_navigation[] = {
	{ 1220, 240, 160 }, // welcome screen: tap to continue
	{ 1420, 23, 23 },   // calibration: upper left
	{ 1620, 456, 296 }, // calibration: lower right
	{ 1820, 240, 160 }, // calibration: centre
	{ 2200, 440, 10 },  // dismiss the startup notice
	{ 2400, 452, 255 }, // hallway
	{ 2500, 60, 130 },  // storeroom
};
// Tapping the Storeroom computer is what makes the guest emit ChMa/Cnct, so the
// host must already be listening: docs/pclink.md puts "opens the Storeroom
// computer" after the host-side wait begins.
constexpr tap k_open_storeroom_computer = { 2630, 48, 155 };
constexpr int k_hold_frames = 20;

// Guest frames at 60 Hz become emulated seconds. Scheduling on emulated time
// is what makes this deterministic: datarover_create returns after the machine
// has already booted, so host-relative timing would drift by the boot time.
constexpr double frames_to_seconds(int frame) { return double(frame) / 60.0; }

void wait_emulated(void *core, double seconds)
{
	while (datarover_emulated_seconds(core) < seconds)
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

// Guest framebuffer at a given guest frame, written raw (480x320 2bpp, the
// layout EmulatorView decode) so a failing run can be inspected offline.
void dump_frame(void *core, const std::filesystem::path &base, int frame)
{
	const uint8_t *bytes = datarover_framebuffer_bytes(core);
	if (!bytes)
		return;
	std::ofstream out(base / ("shot-" + std::to_string(frame) + ".raw"), std::ios::binary);
	out.write(reinterpret_cast<const char *>(bytes), datarover_framebuffer_size());
}

std::string basename_of(const std::string &path)
{
	const size_t slash = path.find_last_of("/\\");
	return slash == std::string::npos ? path : path.substr(slash + 1);
}
} // namespace

int main(int argc, char **argv)
{
	if (argc != 4)
	{
		std::fprintf(stderr, "usage: install <rom> <scratch-dir> <package.pkg>\n");
		return 2;
	}
	std::filesystem::path base(argv[2]);
	std::filesystem::create_directories(base / "cfg");
	std::filesystem::create_directories(base / "nvram");

	void *core = datarover_create((base / "nvram").c_str(), (base / "cfg").c_str(), argv[1]);
	if (!core)
		return 3;

	const auto frame_deadline = clk::now() + std::chrono::seconds(180);
	while (!datarover_framebuffer_bytes(core) && clk::now() < frame_deadline)
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	if (!datarover_framebuffer_bytes(core))
	{
		std::puts("FAIL no frame after boot");
		datarover_destroy(core);
		return 4;
	}

	const int k_dump_frames[] = { 1180, 1900, 2300, 2500 };
	size_t dump_index = 0;
	const auto tap = [&](const struct tap &t) {
		wait_emulated(core, frames_to_seconds(t.frame));
		datarover_pen_down(core, t.x, t.y);
		wait_emulated(core, frames_to_seconds(t.frame + k_hold_frames));
		datarover_pen_up(core);
	};
	for (const struct tap &t : k_navigation)
	{
		while (dump_index < std::size(k_dump_frames) && k_dump_frames[dump_index] < t.frame)
		{
			wait_emulated(core, frames_to_seconds(k_dump_frames[dump_index]));
			dump_frame(core, base, k_dump_frames[dump_index]);
			++dump_index;
		}
		tap(t);
	}

	std::ifstream in(argv[3], std::ios::binary);
	const std::vector<uint8_t> package((std::istreambuf_iterator<char>(in)),
			std::istreambuf_iterator<char>());
	if (package.empty())
	{
		std::fprintf(stderr, "FAIL cannot read package %s\n", argv[3]);
		datarover_destroy(core);
		return 5;
	}
	const std::string name = basename_of(argv[3]);
	std::printf("installing %s (%zu bytes)\n", name.c_str(), package.size());

	int rc = 1;
	int last_progress = -1;
	std::atomic<bool> done{ false };
	std::thread installer([&]() {
		rc = datarover_install_package_named(core, package.data(), package.size(), name.c_str());
		done.store(true);
	});
	// The host is listening now; this tap is what makes the guest send ChMa.
	tap(k_open_storeroom_computer);
	// The handshake is guest-paced and may wait minutes; keep sampling the
	// screen so a timeout still yields evidence of where the guest was.
	for (int frame = 2900; !done.load(); frame += 200)
	{
		wait_emulated(core, frames_to_seconds(frame));
		if (done.load())
			break;
		dump_frame(core, base, frame);
		const int p = datarover_install_progress(core);
		if (p != last_progress && p >= 0)
		{
			last_progress = p;
			std::printf("progress %d%%\n", p);
			std::fflush(stdout);
		}
	}
	done.store(true);
	installer.join();
	dump_frame(core, base, 99999);
	datarover_destroy(core);

	if (rc != 0)
	{
		std::puts("FAIL guest did not complete the install");
		return 6;
	}
	std::puts("PASS in-process PCLink package install");
	return 0;
}
