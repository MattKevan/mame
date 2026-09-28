// license:BSD-3-Clause
// copyright-holders:Danila Sukharev
/***************************************************************************

    3Com EtherLink III 3C589 PC Card

    The programming model follows the 3Com EtherLink III Parallel Tasking
    ISA, EISA, Micro Channel and PCMCIA Adapter Drivers Technical Reference.
    The CIS is the 3C589 tuple sequence published in chapter 7 of that
    reference.

***************************************************************************/

#include "emu.h"
#include "3c589.h"

#include <algorithm>

#define LOG_COMMANDS (1U << 1)
#define LOG_CIS      (1U << 2)
#define LOG_SETUP    (1U << 3)

#define VERBOSE 0
#include "logmacro.h"


DEFINE_DEVICE_TYPE(
		ETHERLINK_III_PCCARD,
		etherlink_iii_pccard_device,
		"3c589",
		"3Com EtherLink III 3C589 PC Card")


etherlink_iii_pccard_device::etherlink_iii_pccard_device(
		machine_config const &mconfig,
		char const *tag,
		device_t *owner,
		u32 clock)
	: device_t(mconfig, ETHERLINK_III_PCCARD, tag, owner, clock)
	, device_pccard_interface(mconfig, *this)
	, device_network_interface(mconfig, *this, 10)
{
}


void etherlink_iii_pccard_device::device_start()
{
	save_item(NAME(m_configuration_option));
	save_item(NAME(m_configuration_status));
	save_item(NAME(m_window));
	save_item(NAME(m_configuration_control));
	save_item(NAME(m_address_configuration));
	save_item(NAME(m_resource_configuration));
	save_item(NAME(m_eeprom_command));
	save_item(NAME(m_eeprom_data));
	save_item(NAME(m_media_status));
	save_item(NAME(m_net_diagnostic));
	save_item(NAME(m_interrupt_enable));
	save_item(NAME(m_status_enable));
	save_item(NAME(m_pending));
	save_item(NAME(m_receive_filter));
	save_item(NAME(m_receive_threshold));
	save_item(NAME(m_transmit_threshold));
	save_item(NAME(m_transmit_start));
	save_item(NAME(m_station_address));
	save_item(NAME(m_eeprom));
	save_item(NAME(m_transmit_data));
	save_item(NAME(m_receive_data));
	save_item(NAME(m_transmit_length));
	save_item(NAME(m_transmit_header_words));
	save_item(NAME(m_receive_position));
	save_item(NAME(m_transmit_status));
	save_item(NAME(m_transmit_interrupt));
	save_item(NAME(m_receiver_enabled));
	save_item(NAME(m_transmitter_enabled));
	save_item(NAME(m_statistics_enabled));
}


void etherlink_iii_pccard_device::device_reset()
{
	m_configuration_option = 0;
	m_configuration_status = 0;
	m_window = 0;
	m_configuration_control = 0;
	m_address_configuration = 0;
	m_resource_configuration = 0;
	m_eeprom_command = 0;
	m_eeprom_data = 0;
	m_media_status = 0;
	m_net_diagnostic = 0x0002; // original 3C589 ASIC revision
	m_interrupt_enable = 0;
	m_status_enable = 0;
	m_pending = 0;
	m_receive_filter = 0;
	m_receive_threshold = 0;
	m_transmit_threshold = 0;
	m_transmit_start = 0;
	m_station_address = { 0x02, 0x60, 0x8c, 0x12, 0x34, 0x56 };
	m_eeprom.fill(0);
	m_eeprom[0] = 0x0260;
	m_eeprom[1] = 0x8c12;
	m_eeprom[2] = 0x3456;
	m_eeprom[3] = 0x9058; // 3C589-TP/COMBO product ID
	m_eeprom[7] = 0x6d50; // 3Com manufacturer ID
	m_transmit_data.clear();
	m_receive_data.clear();
	m_transmit_length = 0;
	m_transmit_header_words = 0;
	m_receive_position = 0;
	m_transmit_status = 0;
	m_transmit_interrupt = false;
	m_receiver_enabled = false;
	m_transmitter_enabled = false;
	m_statistics_enabled = false;
	set_mac(m_station_address.data());
	// Signal card presence here rather than in device_start(): the resulting
	// cd1/cd2 callbacks reach owning-driver logic (e.g. DataRover's
	// PCCARD*_BATTERY ioport reads) that is only safe once ioports have
	// finished configuration, which happens between device start and reset.
	set_present(true);
	update_irq();
}


void etherlink_iii_pccard_device::set_present(bool present)
{
	m_cd1_cb(present ? 0 : 1);
	m_cd2_cb(present ? 0 : 1);
	m_bvd1_cb(1);
	m_bvd2_cb(1);
	m_wp_cb(0);
}


void etherlink_iii_pccard_device::update_irq()
{
	static constexpr u16 INTERRUPT_SOURCES =
			ADAPTER_FAILURE | TX_COMPLETE | TX_AVAILABLE | RX_COMPLETE
			| RX_EARLY | INT_REQUESTED | STATS_FULL;
	if (m_pending & m_interrupt_enable & m_status_enable & INTERRUPT_SOURCES)
	{
		m_pending |= INT_LATCH;
	}

	bool const asserted = bool(m_pending & INT_LATCH);
	m_configuration_status = (m_configuration_status & ~0x02)
			| (asserted ? 0x02 : 0x00);

	// In PC Card I/O mode BVD1 is the active-low IREQ signal.
	m_bvd1_cb(asserted ? 0 : 1);
}


void etherlink_iii_pccard_device::execute_command(u16 command)
{
	LOGMASKED(LOG_COMMANDS, "command %04x window %u\n", command, m_window);
	u16 const parameter = command & 0x07ff;
	switch (command >> 11)
	{
	case 0: // TotalReset
		device_reset();
		break;
	case 1: // SelectWindow
		m_window = parameter & 7;
		break;
	case 3: // RxDisable
		m_receiver_enabled = false;
		break;
	case 4: // RxEnable
		m_receiver_enabled = true;
		break;
	case 5: // RxReset
		discard_receive();
		break;
	case 8: // RxDiscard
		discard_receive();
		break;
	case 9: // TxEnable
		m_transmitter_enabled = true;
		break;
	case 10: // TxDisable
		m_transmitter_enabled = false;
		break;
	case 11: // TxReset
		m_transmit_data.clear();
		m_transmit_length = 0;
		m_transmit_header_words = 0;
		m_transmit_status = 0;
		m_transmit_interrupt = false;
		m_pending &= ~(TX_COMPLETE | TX_AVAILABLE);
		break;
	case 12: // FakeIntr
		m_pending |= INT_REQUESTED;
		break;
	case 13: // AckIntr
		m_pending &= ~(parameter & (INT_LATCH | TX_AVAILABLE
				| RX_EARLY | INT_REQUESTED));
		if (parameter & TX_AVAILABLE)
			m_transmit_threshold = 0x07ff;
		break;
	case 14: // SetIntrEnb
		m_interrupt_enable = parameter;
		break;
	case 15: // SetStatusEnb
		m_status_enable = parameter;
		break;
	case 16: // SetRxFilter
		m_receive_filter = parameter & 0x0f;
		break;
	case 17: // SetRxThreshold
		m_receive_threshold = parameter;
		break;
	case 18: // SetTxThreshold
		m_transmit_threshold = parameter & ~u16(3);
		if ((m_transmit_threshold <= 1792)
				&& (FIFO_BYTES > m_transmit_threshold))
			m_pending |= TX_AVAILABLE;
		break;
	case 19: // SetTxStart
		m_transmit_start = parameter;
		break;
	case 21: // StatsEnable
		m_statistics_enabled = true;
		break;
	case 22: // StatsDisable
		m_statistics_enabled = false;
		break;
	default:
		break;
	}
	update_irq();
}


u16 etherlink_iii_pccard_device::register_r(u8 offset, u16 mem_mask)
{
	if (offset == 0x0e)
	{
		u16 const visible = m_status_enable ? (m_pending & (m_status_enable | INT_LATCH | INT_REQUESTED)) : m_pending;
		return (u16(m_window) << 13) | visible;
	}

	switch (m_window)
	{
	case 0:
		switch (offset)
		{
		case 0x00: return 0x6d50;
		case 0x02: return 0x9058;
		case 0x04: return m_configuration_control;
		case 0x06: return m_address_configuration;
		case 0x08: return m_resource_configuration;
		case 0x0a: return m_eeprom_command;
		case 0x0c: return m_eeprom_data;
		}
		break;

	case 1:
		switch (offset)
		{
		case 0x00:
		case 0x02:
			// PIO reads advance only the byte lanes the host accessed.
			return fifo_r(mem_mask);
		case 0x08:
			return m_receive_data.empty()
					? 0x8000
					: u16((m_receive_data.size()
							- std::min<std::size_t>(
									m_receive_position,
									m_receive_data.size()))
							& 0x07ff);
		case 0x0a:
			return u16(m_transmit_status) << 8; // timer is low byte
		case 0x0c:
			return FIFO_BYTES;
		}
		break;

	case 2:
		if (offset < 6)
			return u16(m_station_address[offset])
					| (u16(m_station_address[offset + 1]) << 8);
		break;

	case 3:
		if (offset == 0x00)
			return 0x0000; // 8 KiB, byte-wide FIFO
		if (offset == 0x02)
			return 0x0000; // 3:5 TX:RX FIFO partition
		break;

	case 4:
		switch (offset)
		{
		case 0x04: return 0;
		case 0x06:
			return m_net_diagnostic
					| (m_transmitter_enabled ? 0x0800 : 0)
					| (m_receiver_enabled ? 0x0400 : 0);
		case 0x08: return 0;
		case 0x0a:
			// Link beat present on a connected 10BASE-T network.
			return m_media_status | 0x0800;
		}
		break;

	case 6:
		return 0; // statistics clear as they are read
	}
	return 0;
}


void etherlink_iii_pccard_device::register_w(u8 offset, u16 data, u16 mem_mask)
{
	if (offset == 0x0e)
	{
		execute_command(data);
		return;
	}

	switch (m_window)
	{
	case 0:
		switch (offset)
		{
		case 0x04:
			COMBINE_DATA(&m_configuration_control);
			break;
		case 0x06:
			COMBINE_DATA(&m_address_configuration);
			break;
		case 0x08:
			COMBINE_DATA(&m_resource_configuration);
			break;
		case 0x0a:
			COMBINE_DATA(&m_eeprom_command);
			if ((m_eeprom_command & 0x00c0) == 0x0080)
				m_eeprom_data = m_eeprom[m_eeprom_command & 0x003f];
			break;
		case 0x0c:
			COMBINE_DATA(&m_eeprom_data);
			break;
		}
		break;

	case 1:
		if (offset <= 0x02)
			fifo_w(data, mem_mask);
		else if ((offset == 0x0a) && (mem_mask & 0xff00))
		{
			m_transmit_status = 0;
			m_pending &= ~TX_COMPLETE;
			update_irq();
		}
		break;

	case 2:
		if (offset < 6)
		{
			if (mem_mask & 0x00ff)
				m_station_address[offset] = u8(data);
			if (mem_mask & 0xff00)
				m_station_address[offset + 1] = u8(data >> 8);
			set_mac(m_station_address.data());
		}
		break;

	case 4:
		if (offset == 0x06)
			COMBINE_DATA(&m_net_diagnostic);
		else if (offset == 0x0a)
			COMBINE_DATA(&m_media_status);
		break;

	default:
		break;
	}
}


u16 etherlink_iii_pccard_device::fifo_r(u16 mem_mask)
{
	u16 result = 0;
	if ((mem_mask & 0x00ff) && (m_receive_position < m_receive_data.size()))
		result |= m_receive_data[m_receive_position++];
	if ((mem_mask & 0xff00) && (m_receive_position < m_receive_data.size()))
		result |= u16(m_receive_data[m_receive_position++]) << 8;
	return result;
}


void etherlink_iii_pccard_device::fifo_w(u16 data, u16 mem_mask)
{
	auto append = [this](u8 value)
	{
		if (!m_transmit_header_words)
		{
			m_transmit_length = value;
			m_transmit_interrupt = false;
			m_transmit_header_words = 1;
			return;
		}
		if (m_transmit_header_words == 1)
		{
			m_transmit_interrupt = BIT(value, 7);
			m_transmit_length = (m_transmit_length | (u16(value) << 8)) & 0x07ff;
			m_transmit_header_words = 2;
			return;
		}
		if (m_transmit_header_words < 4)
		{
			++m_transmit_header_words;
			return;
		}
		m_transmit_data.push_back(value);
		if (m_transmit_data.size() >= ((m_transmit_length + 3) & ~3U))
			finish_transmit();
	};

	if (mem_mask & 0x00ff)
		append(u8(data));
	if (mem_mask & 0xff00)
		append(u8(data >> 8));
}


void etherlink_iii_pccard_device::finish_transmit()
{
	bool const transmitted = m_transmitter_enabled && m_transmit_length
			&& (m_transmit_length <= m_transmit_data.size());
	if (transmitted)
		send(m_transmit_data.data(), m_transmit_length);

	m_transmit_data.clear();
	m_transmit_length = 0;
	m_transmit_header_words = 0;
	if (transmitted && m_transmit_interrupt)
	{
		// Complete plus "interrupt requested" in the TX status stack.
		m_transmit_status = 0xc0;
		m_pending |= TX_COMPLETE;
	}
	m_transmit_interrupt = false;
	update_irq();
}


bool etherlink_iii_pccard_device::receive_filter_accepts(
		u8 const *frame,
		unsigned length) const
{
	if (!m_receiver_enabled || (length < 6))
		return false;
	if (m_receive_filter & 0x08)
		return true;
	if ((m_receive_filter & 0x04)
			&& std::all_of(frame, frame + 6, [](u8 value) { return value == 0xff; }))
		return true;
	if ((m_receive_filter & 0x02) && BIT(frame[0], 0))
		return true;
	return (m_receive_filter & 0x01)
			&& std::equal(m_station_address.begin(), m_station_address.end(), frame);
}


int etherlink_iii_pccard_device::recv_start_cb(u8 *buf, int length)
{
	if (!m_receive_data.empty()
			|| !receive_filter_accepts(buf, length)
			|| (length > 0x07ff))
		return 0;

	m_receive_data.assign(buf, buf + length);
	m_receive_position = 0;
	return length;
}


void etherlink_iii_pccard_device::recv_complete_cb(int result)
{
	if (!result || m_receive_data.empty())
		return;

	m_pending |= RX_COMPLETE;
	update_irq();
}


void etherlink_iii_pccard_device::discard_receive()
{
	m_receive_data.clear();
	m_receive_position = 0;
	m_pending &= ~(RX_COMPLETE | RX_EARLY);
	update_irq();
}


u16 etherlink_iii_pccard_device::io_r(u8 offset, u16 mem_mask)
{
	offset &= 0x0e;
	u16 const value = register_r(offset, mem_mask);
	return (value & mem_mask) | ~mem_mask;
}


void etherlink_iii_pccard_device::io_w(u8 offset, u16 data, u16 mem_mask)
{
	register_w(offset & 0x0e, data, mem_mask);
}


u16 etherlink_iii_pccard_device::read_memory(offs_t offset, u16 mem_mask)
{
	return io_r(u8(offset * 2), mem_mask);
}


void etherlink_iii_pccard_device::write_memory(
		offs_t offset,
		u16 data,
		u16 mem_mask)
{
	io_w(u8(offset * 2), data, mem_mask);
}


u16 etherlink_iii_pccard_device::read_reg(offs_t offset, u16 mem_mask)
{
	// Glacier multiplexes PC Card attribute and I/O cycles into this host
	// window.  Once COR selects an I/O configuration, low offsets are the
	// 3C589's 16-byte I/O register block rather than the start of the CIS.
	if (BIT(m_configuration_option, 0) && (offset < 8))
		return io_r(u8(offset * 2), mem_mask);

	// Original 3C589 CIS.  The full VERS_1 text is shortened, but the
	// manufacturer, function, configuration and I/O resource tuples are the
	// values published by 3Com.
	static constexpr std::array<u8, 77> CIS{
			0x01, 0x02, 0xff, 0xff,
			0x17, 0x03, 0x43, 0x02, 0xff,
			0x20, 0x04, 0x01, 0x01, 0x89, 0x05,
			0x21, 0x02, 0x06, 0x00,
			0x15, 0x20, 0x04, 0x01,
			'3', 'C', 'o', 'm', ' ', 'C', 'o', 'r', 'p', 'o', 'r', 'a', 't', 'i', 'o', 'n', 0x00,
			'3', 'C', '5', '8', '9', 0x00,
			'T', 'P', '/', 'B', 'N', 'C', 0x00,
			0x1a, 0x05, 0x01, 0x03, 0x00, 0x20, 0x01,
			0x1b, 0x0e, 0xc1, 0x01, 0x1d, 0x71, 0x55, 0x1e,
			0x26, 0x05, 0xe7, 0x26, 0x64, 0x20, 0xff, 0xff,
			0xff };

	u8 value = 0xff;
	if (offset < CIS.size())
	{
		value = CIS[offset];
		LOGMASKED(LOG_CIS, "CIS read %04x = %02x mask %04x\n",
				u32(offset), value, mem_mask);
	}
	else if (offset == CONFIG_BASE)
		value = m_configuration_option;
	else if (offset == (CONFIG_BASE + 1))
		value = m_configuration_status;
	return (mem_mask & 0x00ff) ? (0xff00 | value) : 0xffff;
}


void etherlink_iii_pccard_device::write_reg(
		offs_t offset,
		u16 data,
		u16 mem_mask)
{
	if (BIT(m_configuration_option, 0) && (offset < 8))
	{
		io_w(u8(offset * 2), data, mem_mask);
		return;
	}

	if (!(mem_mask & 0x00ff))
		return;

	if (offset == CONFIG_BASE)
	{
		LOGMASKED(LOG_SETUP, "COR write %02x (old %02x)\n",
				u8(data), m_configuration_option);
		u8 const old_value = m_configuration_option;
		m_configuration_option = u8(data);
		if (BIT(m_configuration_option, 7))
			device_reset();
		else if (BIT(old_value, 7))
			m_window = 0;
	}
}
