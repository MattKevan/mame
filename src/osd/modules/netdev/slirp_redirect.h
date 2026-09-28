// Process-wide port-80 redirect for the libslirp provider. Set before the
// network device opens; 0 disables. Only one emulated machine runs per process.
#pragma once
#include <cstdint>

namespace osd {
void set_slirp_http_redirect_port(uint16_t port);
uint16_t slirp_http_redirect_port();
} // namespace osd
