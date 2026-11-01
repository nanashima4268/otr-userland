// SPDX-License-Identifier: GPL-2.0-only
// The otrd service on any FX3 USB backend: the real libusb one (tools/otrd)
// or a fake board (tests/e2e_tests). Runs until `stop` is set.
#ifndef OTRD_SERVICE_H
#define OTRD_SERVICE_H

#include "fx3_device.h"

#include <atomic>
#include <functional>
#include <string>

namespace otr {

struct OtrdOptions final {
    std::string runtime_dir;
    std::string instance;   // empty: the board serial
    std::string fpga;       // bitstream to load (development)
    bool group = false;
};

// Returns the process exit status. on_ready is called with the instance once
// the endpoints are published.
int otrd_serve(const OtrdOptions& options, fx3::UsbBackend& usb, const std::atomic<bool>& stop,
               const std::function<void(const std::string&)>& on_ready = nullptr) noexcept;

}  // namespace otr

#endif  // OTRD_SERVICE_H
