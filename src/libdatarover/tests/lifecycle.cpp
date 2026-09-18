#include "datarover_core.h"
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <thread>

int main(int argc, char **argv) {
    if (argc != 3) return 2;
    const std::filesystem::path root(argv[2]);
    std::filesystem::create_directories(root / "nvram");
    std::filesystem::create_directories(root / "cfg");
    for (int run = 0; run < 2; ++run) {
        void *core = datarover_create((root / "nvram").c_str(), (root / "cfg").c_str(), argv[1]);
        if (!core) { std::fprintf(stderr, "FAIL boot %d\n", run); return 1; }
        if (datarover_create(nullptr, nullptr, argv[1])) return 3;
        std::array<uint8_t, DATAROVER_FB_SIZE> first{};
        const auto *bytes = datarover_framebuffer_bytes(core);
        if (!bytes) return 4;
        std::memcpy(first.data(), bytes, first.size());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (std::memcmp(first.data(), bytes, first.size())) return 5;
        bool changed = false;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
        while (std::chrono::steady_clock::now() < deadline) {
            bytes = datarover_framebuffer_bytes(core);
            if (!bytes) return 6;
            changed |= std::memcmp(first.data(), bytes, first.size()) != 0;
            datarover_pen_down(core, 240, 160);
            datarover_pen_move(core, 241, 161);
            datarover_pen_up(core);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::array<uint8_t, DATAROVER_FB_SIZE> last{};
        std::memcpy(last.data(), bytes, last.size());
        datarover_destroy(core);
        if (std::memcmp(last.data(), bytes, last.size())) return 7;
        std::printf("PASS boot %d: frame changed=%d, snapshot stable, input and destroy completed\n", run, changed);
        std::fflush(stdout);
        if (!changed) return 8;
    }
    void *missing = datarover_create((root / "nvram").c_str(), (root / "cfg").c_str(), "/missing-datarover-rom");
    if (missing) { datarover_destroy(missing); return 9; }
    std::puts("PASS missing ROM returns null");
}
