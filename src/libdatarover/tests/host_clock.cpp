// Bridge protocol against a fake guest memory; no MAME machine is needed.
#include "../host_clock.h"
#include <cassert>
#include <cstdint>
#include <iostream>
#include <unordered_map>

using namespace datarover_host_clock;

namespace {
constexpr uint32_t idle_pc = 0x13c3b270, busy_pc = 0x13d37820;
constexpr uint32_t callback_pc = mapped_base + 0x120;
constexpr int64_t sample = 1790512496000LL;

struct guest {
 std::unordered_map<uint32_t, uint32_t> memory;
 unsigned writes = 0, invalidations = 0;
 guest() {
  for (auto guard : rom_guards) memory[guard.address] = guard.word;
  memory[0x10730] = 0x20000; // live intrinsic table
  memory[0x29eec] = 0x24f34; // date indexical
  memory[0x24f30] = 0x52e0014d;
  memory[user_queue] = 0; // empty run queue
 }
 auto reader() { return [this](uint32_t a) { return memory[a]; }; }
 auto writer() { return [this](uint32_t a, uint32_t v) { ++writes; memory[a] = v; }; }
 auto invalidator() { return [this](uint32_t, uint32_t) { ++invalidations; }; }
 unsigned queued() const { return memory.at(user_queue) >> 16; }
 // The ROM dequeues the entry before calling it.
 void dequeue() { memory[user_queue] &= 0xffff; }
};

result sync(bridge &b, guest &g, uint32_t pc, int64_t ms = sample) {
 return b.synchronize(pc, ms, g.reader(), g.writer(), g.invalidator());
}
}

int main()
{
 {
  guest g; bridge b;
  assert(sync(b, g, idle_pc) == result::unsupported); // not initialized
  b.initialize(g.writer());
  assert(g.memory[mapped_base] == callback && g.memory[mailbox_enabled] == 0);

  // Only a safe idle boundary may queue.
  g.writes = 0;
  assert(sync(b, g, busy_pc) == result::pending && !b.in_flight() && g.writes == 0);

  assert(sync(b, g, idle_pc) == result::pending && b.in_flight());
  assert(g.queued() == 12 && g.memory[user_queue + 4] == mapped_base);
  assert(g.memory[mailbox_enabled] == 1 && g.memory[mailbox_state] == 1);
  int64_t const total = sample + 40587LL * 86400000LL;
  assert(g.memory[mailbox_date] == uint32_t(total / 86400000));
  assert(g.memory[mailbox_time] == uint32_t(total % 86400000));
  assert(g.invalidations == g.writes);

  // Completion is reported only after the callback's epilogue has returned.
  g.dequeue(); g.memory[mailbox_state] = 3;
  assert(sync(b, g, callback_pc) == result::pending && b.in_flight());
  assert(sync(b, g, idle_pc) == result::synced && !b.in_flight());
 }
 {
  // A guest that stops draining its queue (for example, it powers itself
  // off) must not hold lifecycle operations forever.
  guest g; bridge b; b.initialize(g.writer());
  assert(sync(b, g, idle_pc) == result::pending && b.in_flight());
  assert(b.abandon(g.reader(), g.writer()) && !b.in_flight());
  assert(g.memory[mailbox_enabled] == 0);
  // The stale entry is still queued, so a retry waits for it to drain.
  assert(sync(b, g, idle_pc) == result::pending && !b.in_flight());
  g.dequeue();
  assert(sync(b, g, idle_pc) == result::pending && b.in_flight());
  assert(g.memory[mailbox_enabled] == 1);
 }
 {
  // A callback already inside the ROM setter is left to return.
  guest g; bridge b; b.initialize(g.writer());
  assert(sync(b, g, idle_pc) == result::pending);
  g.dequeue(); g.memory[mailbox_state] = 2;
  assert(!b.abandon(g.reader(), g.writer()) && b.in_flight());
  assert(g.memory[mailbox_enabled] == 1);
 }
 {
  guest g; bridge b; b.initialize(g.writer());
  g.memory[rom_guards[0].address] ^= 1;
  g.writes = 0;
  assert(sync(b, g, idle_pc) == result::unsupported && g.writes == 0);
  g.memory[rom_guards[0].address] ^= 1;
  assert(sync(b, g, idle_pc, -2208988800001LL) == result::unsupported);
  g.memory[0x24f30] = 0x62e0014c; // date indexical not live yet
  assert(sync(b, g, idle_pc) == result::pending && !b.in_flight());
 }
 std::cout << "PASS host clock queueing, completion, abandonment and rejection guards\n";
}
