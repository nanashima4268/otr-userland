// SPDX-License-Identifier: GPL-2.0-only
#include "otr_board.h"

#include <algorithm>

namespace otr {

namespace {

constexpr std::uint8_t req(fx3::Request r) noexcept { return static_cast<std::uint8_t>(r); }

std::uint32_t get_le32(const std::uint8_t* p) noexcept
{
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8U) |
           (static_cast<std::uint32_t>(p[2]) << 16U) | (static_cast<std::uint32_t>(p[3]) << 24U);
}

void put_le32(std::uint8_t* p, std::uint32_t v) noexcept
{
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >> 8U);
    p[2] = static_cast<std::uint8_t>(v >> 16U);
    p[3] = static_cast<std::uint8_t>(v >> 24U);
}

}  // namespace

Result<BoardInfo> OtrBoard::info() noexcept
{
    std::array<std::uint8_t, fx3::kInfoLength> b{};
    const auto got = control_.control_in(req(fx3::Request::get_info), 0U, 0U,
                                         fx3::MutableByteView{b.data(), b.size()});
    if (!got) return Result<BoardInfo>::failure(got.error());
    if (got.value() < 28U || b[0] != 'O' || b[1] != 'T' || b[2] != 'R' || b[3] != '1')
        return Result<BoardInfo>::failure(Error::PROTOCOL_ERROR);
    BoardInfo info;
    info.protocol = static_cast<std::uint16_t>(b[4] | (b[5] << 8U));
    info.fw_minor = b[6];
    info.fw_major = b[7];
    info.fpga_configured = b[8] != 0U;
    info.stream_configured = b[9] != 0U;
    info.fpga_source = static_cast<fx3::FpgaSource>(b[10] <= 2U ? b[10] : 0U);
    info.flash_present = b[11] != 0U;
    info.dma_buffer_size = get_le32(&b[12]);
    std::copy(b.begin() + 20, b.begin() + 28, info.flash_uid.begin());
    if (info.protocol != fx3::kProtocolVersion)
        return Result<BoardInfo>::failure(Error::VERSION_MISMATCH);
    return Result<BoardInfo>::success(info);
}

Result<void> OtrBoard::set_gpio(std::uint32_t value, std::uint32_t mask) noexcept
{
    return control_.control_out(req(fx3::Request::gpio), static_cast<std::uint16_t>(value),
                                static_cast<std::uint16_t>(mask), fx3::ByteView{nullptr, 0U});
}

Result<std::uint32_t> OtrBoard::fpga_read(std::uint8_t address) noexcept
{
    std::array<std::uint8_t, 4> b{};
    const auto got = control_.control_in(req(fx3::Request::fpga_reg), address, 0U,
                                         fx3::MutableByteView{b.data(), b.size()});
    if (!got) return Result<std::uint32_t>::failure(got.error());
    if (got.value() != 4U) return Result<std::uint32_t>::failure(Error::PROTOCOL_ERROR);
    return Result<std::uint32_t>::success(get_le32(b.data()));
}

Result<std::uint32_t> OtrBoard::fpga_read(fpga::Reg reg) noexcept
{
    return fpga_read(static_cast<std::uint8_t>(reg));
}

Result<void> OtrBoard::fpga_write(fpga::Reg reg, std::uint32_t value) noexcept
{
    std::array<std::uint8_t, 4> b{};
    put_le32(b.data(), value);
    return control_.control_out(req(fx3::Request::fpga_reg), static_cast<std::uint8_t>(reg), 0U,
                                fx3::ByteView{b.data(), b.size()});
}

Result<void> OtrBoard::check_fpga() noexcept
{
    const auto id = fpga_read(fpga::Reg::id);
    if (!id) return Result<void>::failure(id.error());
    if (id.value() != fpga::kId) return Result<void>::failure(Error::PROTOCOL_ERROR);
    const auto version = fpga_read(fpga::Reg::version);
    if (!version) return Result<void>::failure(version.error());
    if ((version.value() >> 16U) != fpga::kVersionMajor)
        return Result<void>::failure(Error::VERSION_MISMATCH);
    return Result<void>::success();
}

Result<void> OtrBoard::load_fpga(const std::uint8_t* data, std::size_t size) noexcept
{
    constexpr std::size_t kChunk = 4096U;
    if (data == nullptr || size == 0U) return Result<void>::failure(Error::INVALID_ARGUMENT);
    const std::uint8_t load = req(fx3::Request::fpga_load);
    auto r = control_.control_out(load, 0U, 0U, fx3::ByteView{nullptr, 0U});
    for (std::size_t off = 0U; r && off < size; off += kChunk) {
        const std::size_t n = std::min(kChunk, size - off);
        r = control_.control_out(load, 1U, 0U, fx3::ByteView{data + off, n});
    }
    // END reports CDONE: a failed DATA stage shows up here.
    const auto end = control_.control_out(load, 2U, 0U, fx3::ByteView{nullptr, 0U});
    if (!r) return r;
    return end;
}

Result<void> OtrBoard::set_channel_enabled(std::uint8_t channel, bool enabled) noexcept
{
    if (channel >= fx3::kI2cChannels) return Result<void>::failure(Error::INVALID_ARGUMENT);
    std::lock_guard<std::mutex> lock(channel_mutex_);
    const std::uint32_t next = enabled ? (channel_mask_ | (1U << channel))
                                       : (channel_mask_ & ~(1U << channel));
    const auto r = fpga_write(fpga::Reg::ch_enable, next);
    if (r) channel_mask_ = next;
    return r;
}

Result<void> OtrBoard::disable_all_channels() noexcept
{
    std::lock_guard<std::mutex> lock(channel_mutex_);
    const auto r = fpga_write(fpga::Reg::ch_enable, 0U);
    if (r) channel_mask_ = 0U;
    return r;
}

Result<void> OtrBoard::path_reset() noexcept
{
    const auto w = fpga_write(fpga::Reg::ctrl, fpga::kCtrlPathReset);
    if (!w) return w;
    // The reset completes within a packet time (a few microseconds); each
    // register read is a USB control transfer, so a few polls are plenty.
    for (int i = 0; i < 16; ++i) {
        const auto st = fpga_read(fpga::Reg::status);
        if (!st) return Result<void>::failure(st.error());
        if ((st.value() & fpga::kStatusPathResetPending) == 0U) return Result<void>::success();
    }
    return Result<void>::failure(Error::TIMEOUT);
}

Result<void> OtrBoard::set_stream(bool on) noexcept
{
    return control_.control_out(req(fx3::Request::stream), on ? 1U : 0U, 0U,
                                fx3::ByteView{nullptr, 0U});
}

}  // namespace otr
