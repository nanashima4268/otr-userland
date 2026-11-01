// SPDX-License-Identifier: GPL-2.0-only
// Board-level control of the OTR-4TS3-U1 over the FX3 vendor requests:
// GET_INFO, GPIO, FPGA load/registers, STREAM, and the per-channel
// CH_ENABLE bits shared by the four receivers (docs/design.md sections 4, 5).
#ifndef OTR_BOARD_H
#define OTR_BOARD_H

#include "fx3_control.h"
#include "fx3_protocol.h"
#include "fpga_regs.h"

#include <array>
#include <cstdint>
#include <mutex>

namespace otr {

using px4::userland::Error;
using px4::userland::Result;

struct BoardInfo final {
    std::uint16_t protocol = 0U;
    std::uint8_t fw_major = 0U;
    std::uint8_t fw_minor = 0U;
    bool fpga_configured = false;
    bool stream_configured = false;
    fx3::FpgaSource fpga_source = fx3::FpgaSource::none;
    bool flash_present = false;
    std::uint32_t dma_buffer_size = 0U;
    std::array<std::uint8_t, 8> flash_uid{};
};

// GPIO request bits (FX3 otr_fw.h)
inline constexpr std::uint32_t gpio_ch_pwr(unsigned ch) noexcept { return 1U << ch; }
inline constexpr std::uint32_t gpio_ch_rst_n(unsigned ch) noexcept { return 1U << (4U + ch); }

class OtrBoard final {
public:
    explicit OtrBoard(fx3::Fx3Control& control) noexcept : control_(control) {}

    Result<BoardInfo> info() noexcept;
    Result<void> set_gpio(std::uint32_t value, std::uint32_t mask) noexcept;

    Result<std::uint32_t> fpga_read(fpga::Reg reg) noexcept;
    Result<std::uint32_t> fpga_read(std::uint8_t address) noexcept;
    Result<void> fpga_write(fpga::Reg reg, std::uint32_t value) noexcept;
    // Checks ID and the major version of a configured FPGA.
    Result<void> check_fpga() noexcept;
    // Loads a bitstream over FPGA_LOAD (development override, D-49).
    Result<void> load_fpga(const std::uint8_t* data, std::size_t size) noexcept;

    // CH_ENABLE bit for one receiver; the register is shared, so the cached
    // mask is updated under a lock.
    Result<void> set_channel_enabled(std::uint8_t channel, bool enabled) noexcept;
    Result<void> disable_all_channels() noexcept;
    // D-50: with the FX3 stream stopped, realign the FPGA writer.
    Result<void> path_reset() noexcept;
    Result<void> set_stream(bool on) noexcept;

private:
    fx3::Fx3Control& control_;
    std::mutex channel_mutex_;
    std::uint32_t channel_mask_ = 0U;
};

}  // namespace otr

#endif  // OTR_BOARD_H
