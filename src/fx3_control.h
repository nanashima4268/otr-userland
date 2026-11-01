// SPDX-License-Identifier: GPL-2.0-only
// Control-transfer seam to the FX3. The libusb implementation and the test
// fake both provide it; the backend never talks to libusb directly.
#ifndef OTR_FX3_CONTROL_H
#define OTR_FX3_CONTROL_H

#include "px4/error.h"
#include "px4/transport.h"

#include <cstddef>
#include <cstdint>

namespace otr::fx3 {

using px4::userland::ByteView;
using px4::userland::MutableByteView;
using px4::userland::Result;

class Fx3Control {
public:
    virtual ~Fx3Control() noexcept = default;
    // Vendor, device-recipient OUT request with an optional data stage.
    virtual Result<void> control_out(std::uint8_t request, std::uint16_t value,
                                     std::uint16_t index, ByteView data) noexcept = 0;
    // Vendor, device-recipient IN request. Returns the bytes actually received.
    virtual Result<std::size_t> control_in(std::uint8_t request, std::uint16_t value,
                                           std::uint16_t index,
                                           MutableByteView data) noexcept = 0;
};

}  // namespace otr::fx3

#endif  // OTR_FX3_CONTROL_H
