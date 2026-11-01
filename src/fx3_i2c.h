// SPDX-License-Identifier: GPL-2.0-only
// px4-userland BridgeI2cMaster over the OTR FX3 I2C_RUN / I2C_RESULT requests.
// One instance per receiver: each demodulator sits on its own TCA9546A
// channel at 0x64 (D-24), and the FX3 firmware selects the mux channel.
#ifndef OTR_FX3_I2C_H
#define OTR_FX3_I2C_H

#include "bridge_i2c.h"
#include "fx3_control.h"
#include "fx3_protocol.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace otr::fx3 {

class Fx3I2cMaster final : public px4::userland::BridgeI2cMaster {
public:
    // `bus_lock` serializes I2C_RUN/I2C_RESULT pairs across all channels,
    // because the FX3 keeps one result buffer.
    Fx3I2cMaster(Fx3Control& control, std::mutex& bus_lock, std::uint8_t channel) noexcept
        : control_(control), bus_lock_(bus_lock), channel_(channel)
    {
    }

    Result<void> request(px4::userland::BridgeI2cRequest* requests,
                         std::size_t count) noexcept override;

    // Last failure as reported by the firmware (for diagnostics).
    I2cStatus last_status() const noexcept { return last_status_; }
    std::uint8_t last_failed_op() const noexcept { return last_failed_op_; }

private:
    Fx3Control& control_;
    std::mutex& bus_lock_;
    std::uint8_t channel_;
    I2cStatus last_status_ = I2cStatus::ok;
    std::uint8_t last_failed_op_ = 0U;
    std::array<std::uint8_t, kI2cMaxProgram> program_{};
    std::array<std::uint8_t, kI2cResultHeader + kI2cMaxRead> result_{};
};

}  // namespace otr::fx3

#endif  // OTR_FX3_I2C_H
