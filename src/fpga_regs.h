// SPDX-License-Identifier: GPL-2.0-only
// OTR-4TS3-U1 FPGA control registers v1.0, accessed with FPGA_REG (0xB5)
// (docs/design.md section 5; FPGA side: otr-4ts3-u1 fpga/rtl/spi_regs.v).
#ifndef OTR_FPGA_REGS_H
#define OTR_FPGA_REGS_H

#include <cstdint>

namespace otr::fpga {

inline constexpr std::uint32_t kId = 0x4F545231U;          // "OTR1"
inline constexpr std::uint16_t kVersionMajor = 1U;

enum class Reg : std::uint8_t {
    id = 0x00U,
    version = 0x01U,      // [31:16] major, [15:0] minor
    ch_enable = 0x02U,    // bits 0-3, all 0 after reset
    status = 0x03U,       // bit 0: FX3 FLAGA
    drop_cnt0 = 0x04U,    // 0x04-0x07, 16 bits, wraps
    gpio_out = 0x08U,
    scratch = 0x09U,
    pkt_cnt0 = 0x0CU,     // 0x0C-0x0F, 32 bits, wraps
};

inline constexpr std::uint8_t drop_cnt(unsigned ch) noexcept
{
    return static_cast<std::uint8_t>(static_cast<unsigned>(Reg::drop_cnt0) + ch);
}

inline constexpr std::uint8_t pkt_cnt(unsigned ch) noexcept
{
    return static_cast<std::uint8_t>(static_cast<unsigned>(Reg::pkt_cnt0) + ch);
}

}  // namespace otr::fpga

#endif
