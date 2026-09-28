// Checks option-struct compatibility and redirect-port plumbing into slirp.
#include "datarover_core.h"
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <thread>
namespace osd { uint16_t slirp_http_redirect_port(); }
namespace fs = std::filesystem;
using namespace std::chrono_literals;

static void require(bool ok, const char *what) {
	if (!ok) { std::fprintf(stderr, "FAIL %s\n", what); std::fflush(nullptr); std::_Exit(1); }
	std::printf("ok   %s\n", what);
}
static bool network_ready(void *core) {
	for (int i = 0; i < 250; ++i) {
		if (datarover_network_status(core) == 1) return true;
		std::this_thread::sleep_for(20ms);
	}
	return false;
}

int main(int argc, char **argv) {
	if (argc != 3) return 2;
	fs::path base(argv[2]);
	fs::create_directories(base / "cfg"); fs::create_directories(base / "nvram");

	datarover_create_options current{};
	current.struct_size = sizeof current;
	current.network_enabled = 1;
	current.http_redirect_port = 54321;
	void *core = datarover_create_with_options((base / "nvram").c_str(), (base / "cfg").c_str(), argv[1], &current);
	require(core, "boot with the current options");
	require(network_ready(core), "network ready with the current options");
	require(osd::slirp_http_redirect_port() == 54321, "redirect port reaches slirp");
	datarover_destroy(core);

	// A caller built before http_redirect_port existed passes the old size.
	datarover_create_options legacy{};
	legacy.struct_size = offsetof(datarover_create_options, http_redirect_port);
	legacy.network_enabled = 1;
	legacy.http_redirect_port = 999; // must be ignored: outside the old size
	core = datarover_create_with_options((base / "nvram").c_str(), (base / "cfg").c_str(), argv[1], &legacy);
	require(core, "boot with legacy-sized options");
	require(network_ready(core), "legacy-sized options still enable networking");
	require(osd::slirp_http_redirect_port() == 0, "legacy-sized options leave the redirect off");
	datarover_destroy(core);
	std::puts("PASS network options");
}
