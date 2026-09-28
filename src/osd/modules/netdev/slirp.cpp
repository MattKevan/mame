// license:BSD-3-Clause
// copyright-holders:Danila Sukharev
/***************************************************************************

    libslirp user-mode network

    Provides a rootless Ethernet backend on the conventional 10.0.2.0/24
    user network.  The guest sees the host at 10.0.2.2 and DNS at 10.0.2.3.

***************************************************************************/

#include "netdev_module.h"

#include "modules/osdmodule.h"

#if defined(OSD_NET_USE_SLIRP)

#include "netdev_common.h"
#include "slirp_redirect.h"

#include "osdcore.h"

#include <slirp/libslirp.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <poll.h>
#include <string>
#include <unordered_set>
#include <vector>


namespace osd {

namespace {
std::atomic<uint16_t> s_http_redirect_port{ 0 };
} // anonymous namespace

void set_slirp_http_redirect_port(uint16_t port) { s_http_redirect_port.store(port); }
uint16_t slirp_http_redirect_port() { return s_http_redirect_port.load(); }

namespace {

constexpr unsigned ETHERNET_MIN_FRAME_NO_FCS = 60;
constexpr unsigned MAX_QUEUED_FRAMES = 64;


class netdev_slirp : public network_device_base
{
public:
	netdev_slirp(network_handler &handler);
	virtual ~netdev_slirp();

	virtual int send(void const *buf, int len) override;

	bool valid() const { return bool(m_slirp); }

protected:
	virtual int recv_dev(u8 **buf) override;

private:
	struct timer
	{
		SlirpTimerCb callback;
		void *opaque;
		s64 expires_ms = std::numeric_limits<s64>::max();
	};

	static slirp_ssize_t send_packet(
			void const *buf,
			std::size_t len,
			void *opaque);
	static void guest_error(char const *message, void *opaque);
	static s64 clock_get_ns(void *opaque);
	static void *timer_new(
			SlirpTimerCb callback,
			void *callback_opaque,
			void *opaque);
	static void timer_free(void *timer_opaque, void *opaque);
	static void timer_mod(
			void *timer_opaque,
			s64 expires_ms,
			void *opaque);
	static void register_poll_socket(slirp_os_socket socket, void *opaque);
	static void unregister_poll_socket(slirp_os_socket socket, void *opaque);
	static void notify(void *opaque);
	static int add_poll(
			slirp_os_socket socket,
			int events,
			void *opaque);
	static int get_revents(int index, void *opaque);

	static s64 monotonic_ns();
	void pump();
	void pump_timers();

	Slirp *m_slirp = nullptr;
	SlirpCb m_callbacks{};
	std::deque<std::vector<u8>> m_packets;
	std::vector<u8> m_current;
	std::vector<pollfd> m_pollfds;
	std::unordered_set<timer *> m_timers;
};


s64 netdev_slirp::monotonic_ns()
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
}


netdev_slirp::netdev_slirp(network_handler &handler)
	: network_device_base(handler)
{
	SlirpConfig config{};
	config.version = SLIRP_CONFIG_VERSION_MAX;
	config.restricted = 0;
	config.in_enabled = true;
	inet_pton(AF_INET, "10.0.2.0", &config.vnetwork);
	inet_pton(AF_INET, "255.255.255.0", &config.vnetmask);
	inet_pton(AF_INET, "10.0.2.2", &config.vhost);
	config.in6_enabled = false;
	config.vhostname = "mame";
	inet_pton(AF_INET, "10.0.2.15", &config.vdhcp_start);
	inet_pton(AF_INET, "10.0.2.3", &config.vnameserver);
	config.if_mtu = 1500;
	config.if_mru = 1500;
	config.disable_host_loopback = false;
	config.enable_emu = false;
	config.disable_dns = false;
	config.disable_dhcp = false;

	m_callbacks.send_packet = &netdev_slirp::send_packet;
	m_callbacks.guest_error = &netdev_slirp::guest_error;
	m_callbacks.clock_get_ns = &netdev_slirp::clock_get_ns;
	m_callbacks.timer_new = &netdev_slirp::timer_new;
	m_callbacks.timer_free = &netdev_slirp::timer_free;
	m_callbacks.timer_mod = &netdev_slirp::timer_mod;
	m_callbacks.notify = &netdev_slirp::notify;
	m_callbacks.register_poll_socket = &netdev_slirp::register_poll_socket;
	m_callbacks.unregister_poll_socket =
			&netdev_slirp::unregister_poll_socket;

	m_slirp = slirp_new(&config, &m_callbacks, this);
#if defined(SLIRP_HAS_HTTP_REDIRECT)
	if (m_slirp)
		slirp_set_http_redirect_port(m_slirp, slirp_http_redirect_port());
#endif
	if (!m_slirp)
		osd_printf_error("libslirp network could not be initialized\n");
}


netdev_slirp::~netdev_slirp()
{
	if (m_slirp)
		slirp_cleanup(m_slirp);
	for (timer *const item : m_timers)
		delete item;
}


slirp_ssize_t netdev_slirp::send_packet(
		void const *buf,
		std::size_t len,
		void *opaque)
{
	netdev_slirp &device = *static_cast<netdev_slirp *>(opaque);
	if (!len || (device.m_packets.size() >= MAX_QUEUED_FRAMES))
		return len;
	std::size_t const accepted = len;

	// libslirp's ARP path may supply the 64-byte on-wire minimum with a
	// zero CRC placeholder.  MAME network devices exchange frames without
	// the FCS, so remove that placeholder before presenting the frame.
	if ((len == 64)
			&& std::all_of(
					static_cast<u8 const *>(buf) + 60,
					static_cast<u8 const *>(buf) + 64,
					[](u8 value) { return value == 0; }))
	{
		len = 60;
	}

	std::vector<u8> frame(
			static_cast<u8 const *>(buf),
			static_cast<u8 const *>(buf) + len);
	frame.resize(std::max<std::size_t>(len, ETHERNET_MIN_FRAME_NO_FCS), 0);
	device.m_packets.emplace_back(std::move(frame));
	return accepted;
}


void netdev_slirp::guest_error(char const *message, void *opaque)
{
	osd_printf_warning("libslirp guest error: %s\n", message);
}


s64 netdev_slirp::clock_get_ns(void *opaque)
{
	return monotonic_ns();
}


void *netdev_slirp::timer_new(
		SlirpTimerCb callback,
		void *callback_opaque,
		void *opaque)
{
	netdev_slirp &device = *static_cast<netdev_slirp *>(opaque);
	timer *const result = new timer{ callback, callback_opaque };
	device.m_timers.emplace(result);
	return result;
}


void netdev_slirp::timer_free(void *timer_opaque, void *opaque)
{
	netdev_slirp &device = *static_cast<netdev_slirp *>(opaque);
	timer *const item = static_cast<timer *>(timer_opaque);
	device.m_timers.erase(item);
	delete item;
}


void netdev_slirp::timer_mod(
		void *timer_opaque,
		s64 expires_ms,
		void *opaque)
{
	static_cast<timer *>(timer_opaque)->expires_ms = expires_ms;
}


void netdev_slirp::register_poll_socket(slirp_os_socket socket, void *opaque)
{
}


void netdev_slirp::unregister_poll_socket(slirp_os_socket socket, void *opaque)
{
}


void netdev_slirp::notify(void *opaque)
{
}


int netdev_slirp::add_poll(
		slirp_os_socket socket,
		int events,
		void *opaque)
{
	netdev_slirp &device = *static_cast<netdev_slirp *>(opaque);
	short native_events = 0;
	if (events & SLIRP_POLL_IN)
		native_events |= POLLIN;
	if (events & SLIRP_POLL_OUT)
		native_events |= POLLOUT;
	if (events & SLIRP_POLL_PRI)
		native_events |= POLLPRI;
	device.m_pollfds.push_back(pollfd{ socket, native_events, 0 });
	return device.m_pollfds.size() - 1;
}


int netdev_slirp::get_revents(int index, void *opaque)
{
	netdev_slirp &device = *static_cast<netdev_slirp *>(opaque);
	short const native_events = device.m_pollfds[index].revents;
	int events = 0;
	if (native_events & POLLIN)
		events |= SLIRP_POLL_IN;
	if (native_events & POLLOUT)
		events |= SLIRP_POLL_OUT;
	if (native_events & POLLPRI)
		events |= SLIRP_POLL_PRI;
	if (native_events & POLLERR)
		events |= SLIRP_POLL_ERR;
	if (native_events & POLLHUP)
		events |= SLIRP_POLL_HUP;
	return events;
}


void netdev_slirp::pump_timers()
{
	s64 const now_ms = monotonic_ns() / 1'000'000;
	std::vector<timer *> due;
	for (timer *const item : m_timers)
	{
		if (item->expires_ms <= now_ms)
		{
			item->expires_ms = std::numeric_limits<s64>::max();
			due.emplace_back(item);
		}
	}

	for (timer *const item : due)
	{
		if (m_timers.find(item) != m_timers.end())
			item->callback(item->opaque);
	}
}


void netdev_slirp::pump()
{
	if (!m_slirp)
		return;

	pump_timers();
	m_pollfds.clear();
	u32 timeout = 0;
	slirp_pollfds_fill_socket(m_slirp, &timeout, &netdev_slirp::add_poll, this);
	int const result = ::poll(m_pollfds.data(), m_pollfds.size(), 0);
	slirp_pollfds_poll(
			m_slirp,
			result < 0,
			&netdev_slirp::get_revents,
			this);
	pump_timers();
}


int netdev_slirp::send(void const *buf, int len)
{
	if (!m_slirp || (len <= 0))
		return 0;

	if (len < ETHERNET_MIN_FRAME_NO_FCS)
	{
		std::vector<u8> frame(
				static_cast<u8 const *>(buf),
				static_cast<u8 const *>(buf) + len);
		frame.resize(ETHERNET_MIN_FRAME_NO_FCS, 0);
		slirp_input(m_slirp, frame.data(), frame.size());
	}
	else
	{
		slirp_input(
				m_slirp,
				static_cast<u8 const *>(buf),
				len);
	}
	pump();
	return len;
}


int netdev_slirp::recv_dev(u8 **buf)
{
	pump();
	if (m_packets.empty())
		return 0;

	m_current = std::move(m_packets.front());
	m_packets.pop_front();
	*buf = m_current.data();
	return m_current.size();
}


class slirp_module : public osd_module, public netdev_module
{
public:
	slirp_module()
		: osd_module(OSD_NETDEV_PROVIDER, "slirp")
	{
	}

	virtual int init(osd_interface &osd, osd_options const &options) override
	{
		m_description = "libslirp user network (10.0.2.0/24, host 10.0.2.2)";
		return 0;
	}

	virtual std::unique_ptr<network_device> open_device(
			int id,
			network_handler &handler) override
	{
		if (id != 0)
			return nullptr;
		auto result = std::make_unique<netdev_slirp>(handler);
		if (!result->valid())
			return nullptr;
		return result;
	}

	virtual std::vector<network_device_info> list_devices() override
	{
		return { network_device_info{ 0, m_description } };
	}

private:
	std::string m_description;
};

} // anonymous namespace

} // namespace osd


MODULE_DEFINITION(NETDEV_SLIRP, osd::slirp_module)

#endif // defined(OSD_NET_USE_SLIRP)
