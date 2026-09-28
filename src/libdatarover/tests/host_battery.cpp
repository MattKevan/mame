// Compile this translation unit instead of the core archive's core object.
// The observer runs on the worker; only copied measurements reach this thread.
template<class Core, class Machine> void observe_battery(Core &, Machine &);
#define DATAROVER_CORE_TEST_OBSERVER(core, machine) observe_battery(core, machine)
#include "../datarover_core.cpp"
#include <filesystem>

namespace {
struct measurement {
 unsigned sample = 0;
 unsigned main = 0, backup = 0, ac = 0;
 double seconds = 0;
 bool restored = false;
 unsigned revision = 0;
 unsigned generation = 0;
};
std::mutex measurement_mutex;
measurement latest;
}

template<class Core, class Machine> void observe_battery(Core &core, Machine &machine)
{
 auto &space = core.memintf->space(AS_PROGRAM);
 // Keep the guest charger enabled so any unwanted synthetic accumulation
 // would be visible after disabling mirroring.
 space.write_dword(0x10c00184, space.read_dword(0x10c00184) | 2);
 auto adc = [&](unsigned channel) {
  space.write_dword(0x10c00080, 0x54000000 | channel);
  space.write_dword(0x10c00080, 0x58000000);
  return (space.read_dword(0x10c00088) >> 5) & 0x3ff;
 };
 measurement current;
 current.sample = machine.root_device().ioport("HOST_BATTERY")->read();
 current.main = adc(24);
 current.backup = adc(28);
 current.ac = (space.read_dword(0x10c001c4) >> 30) & 1;
 current.seconds = machine.time().as_double();
 current.restored = core.restored_checkpoint;
 std::lock_guard<std::mutex> lock(measurement_mutex);
 current.revision = latest.revision + 1;
 current.generation = latest.generation + (current.seconds < latest.seconds ? 1 : 0);
 latest = current;
}

measurement snapshot() {
 std::lock_guard<std::mutex> lock(measurement_mutex);
 return latest;
}
void require(bool value, const char *message) {
 if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::fflush(nullptr); std::_Exit(1); }
}
template<class Predicate> void await(Predicate predicate, const char *message) {
 auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
 do {
  if (predicate()) return;
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
 } while (std::chrono::steady_clock::now() < deadline);
 require(false, message);
}
void expect(void *core, int percentage, int ac) {
 unsigned before = snapshot().revision;
 datarover_set_host_battery(core, percentage, ac);
 unsigned sample = percentage < 0 ? 0 : unsigned(std::min(percentage,100)+1) | (ac ? 128 : 0);
 await([&]{ auto s=snapshot(); return s.revision>before && s.sample==sample; }, "worker applied queued sample");
 auto s=snapshot();
 // Mirroring never drops the guest below its 320-count low-power point:
 // below that Magic Cap turns off communications, which breaks networking
 // and PCLink. The floor is 330 counts (about 35%).
 require(s.main == (percentage<0 ? 200 : std::max(330, 80 + (std::min(percentage,100)*720+50)/100)), "main ADC mapping");
 require(s.backup == (percentage<0 ? 300 : 1000), "backup health/restoration");
 require(s.ac == unsigned(percentage>=0 && ac!=0), "AC state/restoration");
}
int main(int argc, char **argv) {
 require(argc==3, "usage: host_battery ROM_PATH SCRATCH_STATE");
 namespace fs=std::filesystem;
 fs::path base(argv[2]); fs::create_directories(base/"cfg"); fs::create_directories(base/"nvram");
 std::ofstream(base/"cfg/datarover840.cfg") << R"(<mameconfig version="10"><system name="datarover840"><input>
<port tag=":BOOT_MODE" type="CONFIG" mask="8" defvalue="8" value="0" />
<port tag=":BATTERY" type="CONFIG" mask="3" defvalue="0" value="1" />
<port tag=":BATTERY" type="CONFIG" mask="12" defvalue="0" value="8" />
</input></system></mameconfig>)";
 auto create=[&]{return datarover_create((base/"nvram").c_str(), (base/"cfg").c_str(), argv[1]);};
 void *core=create(); require(core, "initial boot");
 expect(core,-1,1);
 for(int percent=0; percent<=100; ++percent) expect(core,percent,percent%2);
 expect(core,1000,1);
 expect(core,37,1);
 double seconds=snapshot().seconds;
 await([&]{return snapshot().seconds >= seconds+2.0;}, "mirrored charging interval");
 expect(core,-1,0);
 expect(core,42,0);
 datarover_set_paused(core,1);
 await([&]{return datarover_save_status(core)==2;}, "pause checkpoint saved");
 datarover_set_host_battery(core,73,1);
 datarover_set_paused(core,0);
 await([&]{auto s=snapshot(); return s.sample==202 && s.main==606 && s.ac==1;}, "sample queued while paused applied on resume");
 unsigned revision=snapshot().revision;
 unsigned generation=snapshot().generation;
 datarover_restart(core);
 await([&]{
  for(auto &entry:fs::directory_iterator(base/"cfg"))
   if(entry.path().filename().string().find(".restart-")!=std::string::npos) return true;
  return false;
 }, "restart executed");
 await([&]{auto s=snapshot(); return s.generation>generation && s.revision>revision && s.sample==202 && s.main==606 && s.ac==1;}, "host selection survived restart");
 datarover_set_paused(core,1);
 await([&]{return datarover_save_status(core)==2;}, "second checkpoint saved");
 datarover_destroy(core);
 revision=snapshot().revision;
 core=create(); require(core, "checkpoint recreate");
 await([&]{auto s=snapshot(); return s.revision>revision && s.restored;}, "checkpoint restored");
 expect(core,-1,0);
 expect(core,18,1);
 datarover_destroy(core);
 std::puts("PASS: native queued battery 0..100, clamp, unavailable, AC, charge isolation, pause, restart and checkpoint restore");
}
