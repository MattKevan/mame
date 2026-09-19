// license:BSD-3-Clause
// copyright-holders:DataRover contributors
#ifndef MAME_BUS_RS232_HOST_SERIAL_H
#define MAME_BUS_RS232_HOST_SERIAL_H

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>

// Shared ownership lets a host transfer finish safely after the device stops.
// Only these queues cross threads; serial registers and timers remain owned by
// the emulation thread. Writes are nonblocking and may accept a partial buffer.
class rs232_host_channel
{
public:
	static constexpr std::size_t capacity = 65536;

	std::size_t host_write(const uint8_t *data, std::size_t count)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		return append(m_to_device, data, count);
	}

	std::size_t host_read(uint8_t *data, std::size_t count)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		return consume(m_to_host, data, count);
	}

	std::size_t device_read(uint8_t *data, std::size_t count)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		return consume(m_to_device, data, count);
	}

	void device_write(uint8_t data)
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		// The guest cannot retry an already transmitted UART byte. Abort on
		// overflow instead of returning a silently corrupted protocol stream.
		if (m_open && !append(m_to_host, &data, 1))
			close_locked();
	}

	bool is_open() const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_open;
	}

	void close()
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		close_locked();
	}

	// Start a fresh transfer: drop any unconsumed bytes and reopen. A failed
	// or aborted transfer leaves the channel closed, so every new host-side
	// install must reset before it writes.
	void reset()
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		m_to_device.clear();
		m_to_host.clear();
		m_open = true;
	}

private:
	std::size_t append(std::deque<uint8_t> &queue, const uint8_t *data, std::size_t count)
	{
		if (!m_open || !count)
			return 0;
		count = std::min(count, capacity - queue.size());
		queue.insert(queue.end(), data, data + count);
		return count;
	}

	std::size_t consume(std::deque<uint8_t> &queue, uint8_t *data, std::size_t count)
	{
		count = std::min(count, queue.size());
		for (std::size_t i = 0; i < count; ++i)
		{
			data[i] = queue.front();
			queue.pop_front();
		}
		return count;
	}

	void close_locked()
	{
		m_open = false;
		m_to_device.clear();
		m_to_host.clear();
	}

	mutable std::mutex m_mutex;
	std::deque<uint8_t> m_to_device;
	std::deque<uint8_t> m_to_host;
	bool m_open = true;
};

#endif // MAME_BUS_RS232_HOST_SERIAL_H
