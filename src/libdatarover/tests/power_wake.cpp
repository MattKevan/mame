// Compile this translation unit instead of the core archive's core object.
// The observer runs on the worker; only copied measurements reach this thread.
template<class Core, class Machine> void observe_power(Core &, Machine &);
#define DATAROVER_CORE_TEST_OBSERVER(core, machine) observe_power(core, machine)
#include "../datarover_core.cpp"
#include <filesystem>

namespace fs = std::filesystem;

namespace {
struct measurement {
 bool halted = false;
 bool vcc = true;
 unsigned revision = 0;
};
std::mutex measurement_mutex;
measurement latest;
// Test-only power button presses, counted in worker ticks: >0 held.
std::atomic<int> press_ticks{ 0 };
// Lose a queued clock update, like a guest that stops draining its run queue.
std::atomic<bool> stall_clock{ false }, clock_stalled{ false };
}

template<class Core, class Machine> void observe_power(Core &core, Machine &machine)
{
 if (int ticks = press_ticks.load()) {
  auto *field = machine.root_device().ioport("POWER_BUTTON")->field(1);
  if (ticks > 1) field->set_value(1); else field->clear_value();
  press_ticks.store(ticks - 1);
 }
 auto *cpu = machine.root_device().subdevice("maincpu");
 if (stall_clock.load() && core.clock_bridge.in_flight() && !clock_stalled.load()) {
  // Drop the bridge's entry, which it appends last, so the callback never runs.
  auto &space = core.memintf->space(AS_PROGRAM);
  uint32_t const packed = space.read_dword(datarover_host_clock::user_queue);
  space.write_dword(datarover_host_clock::user_queue, packed - (12U << 16));
  clock_stalled.store(true);
 }
 measurement current;
 current.halted = cpu->execute().suspended(SUSPEND_REASON_HALT);
 current.vcc = core.memintf->space(AS_PROGRAM).read_dword(0x10c001c4) & 1;
 std::lock_guard<std::mutex> lock(measurement_mutex);
 current.revision = latest.revision + 1;
 latest = current;
}

measurement snapshot() {
 std::lock_guard<std::mutex> lock(measurement_mutex);
 return latest;
}
void require(bool value, const char *message) {
 if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::fflush(nullptr); std::_Exit(1); }
}
template<class Predicate> bool within(std::chrono::milliseconds limit, Predicate predicate) {
 auto deadline = std::chrono::steady_clock::now() + limit;
 do {
  if (predicate()) return true;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
 } while (std::chrono::steady_clock::now() < deadline);
 return false;
}
bool powered_off() { auto s = snapshot(); return s.halted && !s.vcc; }
bool running() { auto s = snapshot(); return !s.halted && s.vcc; }
bool cold_booted(const fs::path &cfg) {
 for (auto &entry : fs::directory_iterator(cfg))
  if (entry.path().filename().string().find(".stuck-") != std::string::npos) return true;
 return false;
}
void power_off() {
 // The guest's own power-off path: hold the button, as a user would.
 // A guest still finishing its wake ignores the button, so press again.
 for (int attempt = 0; attempt < 4; ++attempt) {
  press_ticks.store(12);
  if (within(std::chrono::seconds(10), powered_off)) return;
 }
 require(false, "guest did not power itself off");
}
void tap(void *core, int x, int y) {
 datarover_pen_down(core, x, y);
 std::this_thread::sleep_for(std::chrono::milliseconds(60));
 datarover_pen_up(core);
}

int main(int argc, char **argv)
{
 if (argc != 3) return 2;
 fs::path base(argv[2]);
 fs::create_directories(base / "cfg");
 fs::create_directories(base / "nvram");
 auto create = [&] { return datarover_create((base / "nvram").c_str(), (base / "cfg").c_str(), argv[1]); };

 void *core = create();
 require(core, "initial boot");
 std::this_thread::sleep_for(std::chrono::seconds(25));
 require(running(), "guest running at the welcome screen");

 // A powered-off guest only wakes on its power button. A tap must press it.
 power_off();
 tap(core, 240, 160);
 require(within(std::chrono::seconds(5), running), "tap did not wake a powered-off guest");
 std::puts("PASS tap wakes a powered-off guest");

 // The shell's checkpoint can capture that powered-off state.
 power_off();
 datarover_set_paused(core, 1);
 require(within(std::chrono::seconds(3), [&] { return datarover_save_status(core) == 2; }), "checkpoint save");
 datarover_destroy(core);

 core = create();
 require(core, "restore boot");
 require(within(std::chrono::seconds(3), powered_off), "restored guest is powered off");
 tap(core, 240, 160);
 require(within(std::chrono::seconds(5), running), "tap did not wake a restored powered-off guest");
 // The guest shows its wake progress for several seconds; a wake is proof of
 // life, so stuck-restore recovery must not cold boot it.
 std::this_thread::sleep_for(std::chrono::seconds(5));
 require(!cold_booted(base / "cfg"), "wake tap triggered stuck-restore recovery");
 // Taps on inert screen areas are normal input, not evidence of a hang.
 for (int i = 0; i < 2; ++i) {
  tap(core, 470, 310);
  std::this_thread::sleep_for(std::chrono::seconds(4));
 }
 require(!cold_booted(base / "cfg"), "later inert taps triggered stuck-restore recovery");
 std::puts("PASS restored powered-off guest wakes without stuck recovery");

 // A guest that never runs a queued clock update must not hold pause, its
 // checkpoint or shutdown.
 stall_clock.store(true);
 datarover_set_host_clock(core, 1, std::chrono::duration_cast<std::chrono::milliseconds>(
   std::chrono::system_clock::now().time_since_epoch()).count());
 require(within(std::chrono::seconds(10), [] { return clock_stalled.load(); }), "clock update was not queued");
 datarover_set_paused(core, 1);
 require(within(std::chrono::seconds(10), [&] { return datarover_save_status(core) == 2; }),
   "pause did not checkpoint with a stalled clock update");
 auto const stopping = std::chrono::steady_clock::now();
 datarover_destroy(core);
 require(std::chrono::steady_clock::now() - stopping < std::chrono::seconds(10),
   "shutdown waited on a stalled clock update");
 std::puts("PASS stalled clock update does not block pause, checkpoint or shutdown");
 std::puts("PASS power wake");
}
