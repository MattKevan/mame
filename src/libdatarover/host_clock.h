// Retail Magic Cap calendar synchronization through its native user run queue.
#pragma once
#include <cstdint>

namespace datarover_host_clock {

enum class result { pending, synced, unsupported };

inline constexpr uint32_t mapped_base = 0x01000000, mapped_bytes = 0x1000;
inline constexpr uint32_t callback = mapped_base + 0x100;
inline constexpr uint32_t mailbox_date = mapped_base + 0x200, mailbox_time = mapped_base + 0x204;
inline constexpr uint32_t mailbox_state = mapped_base + 0x208, mailbox_enabled = mapped_base + 0x20c;
inline constexpr uint32_t user_queue = 0x108b4;

struct rom_guard { uint32_t address, word; };
inline constexpr rom_guard rom_guards[] = {
	{ 0x13d37804, 0x27bdff48 }, { 0x13d37840, 0x3c050526 },
	{ 0x13d378a8, 0x240f01c0 }, { 0x13d378f0, 0x3c032000 },
	{ 0x13d37904, 0x0cfa5914 }, { 0x13cbdf9c, 0x94870000 },
	{ 0x13cbdfa8, 0x2c6200c0 }, { 0x13cbe31c, 0x0cf09524 },
	{ 0x13cbe37c, 0x8e7c0004 }, { 0x13cbe380, 0x8e610000 },
	{ 0x13c3b250, 0x3c08b0c0 }
};

// Descriptor ABI: word0 entry PC, word1 gp. The callback runs on the real guest
// user queue, preserving its stack/return address. No guest heap is overwritten.
// Mailbox state: 1 queued, 2 running (inside the ROM setter when enabled),
// 3 returning. A disabled callback skips the setter and only returns.
inline constexpr uint32_t callback_words[] = {
	0x27bdffe0, 0xafbf001c, 0x3c080100, 0x24090002, 0xad090208,
	0x8d09020c, 0x11200007, 0x00000000, 0x8d040200, 0x8d050204,
	0x3c1913d3, 0x37397804, 0x0320f809, 0x00000000,
	0x3c080100, 0x24090003, 0xad090208, 0x8fbf001c, 0x03e00008, 0x27bd0020
};

class bridge
{
	bool m_initialized = false, m_in_flight = false, m_cancelled = false;

public:
	bool in_flight() const { return m_in_flight; }

	// Call once per new running_machine after mapping host-owned backing
	// storage, and before the guest executes. A restored checkpoint may still
	// hold a queued entry; it then finds this callback, disabled, in place.
	// Save/pause/restart/stop are gated while in_flight(), including epilogue.
	template <class Write32> void initialize(Write32 write)
	{
		write(mapped_base, callback);
		write(mapped_base + 4, 0xdfe0);
		for (unsigned i = 0; i < sizeof(callback_words) / sizeof(callback_words[0]); ++i)
			write(callback + i * 4, callback_words[i]);
		write(mailbox_state, 0);
		write(mailbox_enabled, 0);
		m_initialized = true;
		m_in_flight = false;
		m_cancelled = false;
	}

	template <class Write32> void cancel(Write32 write)
	{
		if (m_in_flight) {
			m_cancelled = true;
			write(mailbox_enabled, 0);
		}
	}

	// Stop waiting for a guest that no longer drains its run queue, e.g. one
	// that powered itself off. The queued entry stays behind disabled, and a
	// retry waits until the guest has consumed it. A callback already running
	// the ROM setter is left to return; that returns false.
	template <class Read32, class Write32> bool abandon(Read32 read, Write32 write)
	{
		if (!m_in_flight) return true;
		if (read(mailbox_state) == 2) return false;
		write(mailbox_enabled, 0);
		m_in_flight = false;
		m_cancelled = false;
		return true;
	}

	// While draining a request the caller must keep supplying THAT request's
	// host wall-clock sample plus monotonic elapsed time. New generations wait.
	template <class Read32, class Write32, class Invalidate>
	result synchronize(uint32_t pc, int64_t local_unix_milliseconds,
			Read32 read, Write32 write, Invalidate invalidate)
	{
		if (!m_initialized) return result::unsupported;
		auto store = [&](uint32_t a, uint32_t v) { write(a, v); invalidate(a, 4); };
		auto refresh = [&] {
			int64_t const total = local_unix_milliseconds + 40587LL * 86400000LL;
			store(mailbox_date, uint32_t(total / 86400000));
			store(mailbox_time, uint32_t(total % 86400000));
		};
		bool const valid = local_unix_milliseconds >= -2208988800000LL
				&& local_unix_milliseconds <= 253402300799999LL;
		if (m_in_flight) {
			auto const state = read(mailbox_state);
			if (state == 3 && (pc < mapped_base || pc >= mapped_base + mapped_bytes)) {
				// The marker precedes the return epilogue; only report after leaving it.
				m_in_flight = false;
				return result::synced;
			}
			if (state == 1 && !m_cancelled && valid) refresh();
			return result::pending;
		}
		if (!valid) return result::unsupported;
		for (auto &g : rom_guards)
			if (read(g.address) != g.word) return result::unsupported;
		if (pc < 0x13c3b250 || pc >= 0x13c3b440 || (pc & 3)) return result::pending;
		// The intrinsic table and date indexical must be live before initialization
		// can run. A ROM-backed Time object is valid: the real setter shadows it.
		auto const intrinsic = read(0x10730), ref = read(0x29eec);
		if (intrinsic < 0x10000 || ref < 0x180 || ref >= 0x400000 || (ref & 3)) return result::pending;
		if ((read(ref - 4) & 0xffff) != 0x14d) return result::pending;
		auto const packed = read(user_queue);
		unsigned const count = packed >> 16;
		if (count >= 0xc0 || count % 12) return result::pending;
		for (unsigned off = 0; off < count; off += 12)
			if (read(user_queue + 4 + off) == mapped_base) return result::pending;
		refresh();
		store(mailbox_enabled, 1);
		store(mailbox_state, 1);
		store(user_queue + 4 + count, mapped_base);
		store(user_queue + 8 + count, 0);
		store(user_queue + 12 + count, 0);
		// Publish the count last, matching AddRunQueueEntry's 12-byte entry layout.
		store(user_queue, ((count + 12) << 16) | (packed & 0xffff));
		m_in_flight = true;
		m_cancelled = false;
		return result::pending;
	}
};

} // namespace datarover_host_clock
