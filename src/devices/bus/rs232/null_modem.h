// license:BSD-3-Clause
// copyright-holders:smf,Carl
#ifndef MAME_BUS_RS232_NULL_MODEM_H
#define MAME_BUS_RS232_NULL_MODEM_H

#pragma once

#include "rs232.h"
#include "imagedev/bitbngr.h"
#include "diserial.h"
#include "host_serial.h"

#include <memory>

class null_modem_device : public device_t,
	public device_serial_interface,
	public device_rs232_port_interface
{
public:
	null_modem_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	virtual void input_txd(int state) override { device_serial_interface::rx_w(state); }
	virtual void input_rts(int state) override { m_rts = state; }
	virtual void input_dtr(int state) override { m_dtr = state; }

	void update_serial(int state);

	// Configure on the emulation thread before the machine starts. The host
	// retains the shared channel and never calls device methods across threads.
	// Re-applying the serial configuration asserts the control lines the guest
	// waits for, which only become true once a host channel exists.
	void set_host_channel(std::shared_ptr<rs232_host_channel> channel)
	{
		m_host_channel = std::move(channel);
		update_serial(0);
	}

protected:
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

	virtual void tra_callback() override;
	virtual void tra_complete() override;
	virtual void rcv_complete() override;

private:
	TIMER_CALLBACK_MEMBER(update_queue);
	void update_input_buffer();

	required_device<bitbanger_device> m_stream;
	std::shared_ptr<rs232_host_channel> m_host_channel;

	required_ioport m_rs232_txbaud;
	required_ioport m_rs232_rxbaud;
	required_ioport m_rs232_databits;
	required_ioport m_rs232_parity;
	required_ioport m_rs232_stopbits;
	required_ioport m_flow;

	uint8_t m_input_buffer[1000];
	uint32_t m_input_count;
	uint32_t m_input_index;
	emu_timer *m_timer_poll;
	int m_rts;
	int m_dtr;
	int m_xoff;
	bool m_cr;
};

DECLARE_DEVICE_TYPE(NULL_MODEM, null_modem_device)

#endif // MAME_BUS_RS232_NULL_MODEM_H
