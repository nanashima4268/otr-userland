// SPDX-License-Identifier: GPL-2.0-only
#include "fx3_i2c.h"

namespace otr::fx3 {

using px4::userland::BridgeI2cRequest;
using px4::userland::BridgeI2cRequestType;
using px4::userland::Error;

Result<void> Fx3I2cMaster::request(BridgeI2cRequest* requests, std::size_t count) noexcept
{
    if (requests == nullptr || count == 0U || channel_ >= kI2cChannels)
        return Result<void>::failure(Error::INVALID_ARGUMENT);

    // Encode the whole request list as one I2C program.
    std::size_t len = 0U;
    std::size_t total_read = 0U;
    for (std::size_t i = 0U; i < count; ++i) {
        const BridgeI2cRequest& r = requests[i];
        if (r.address > 0x7FU)
            return Result<void>::failure(Error::INVALID_ARGUMENT);
        std::size_t n = 0U;
        I2cOp op = I2cOp::write;
        if (r.type == BridgeI2cRequestType::write) {
            n = r.write_data.size;
            if (n != 0U && r.write_data.data == nullptr)
                return Result<void>::failure(Error::INVALID_ARGUMENT);
        } else if (r.type == BridgeI2cRequestType::read) {
            op = I2cOp::read;
            n = r.read_data.size;
            if (n == 0U || r.read_data.data == nullptr)
                return Result<void>::failure(Error::INVALID_ARGUMENT);
            total_read += n;
        } else {
            return Result<void>::failure(Error::INVALID_ARGUMENT);
        }
        const std::size_t need = kI2cOpHeader + (op == I2cOp::write ? n : 0U);
        if (n > 0xFFFFU || len + need > kI2cMaxProgram || total_read > kI2cMaxRead)
            return Result<void>::failure(Error::BUFFER_TOO_SMALL);
        program_[len + 0U] = static_cast<std::uint8_t>(op);
        program_[len + 1U] = r.address;
        program_[len + 2U] = static_cast<std::uint8_t>(n & 0xFFU);
        program_[len + 3U] = static_cast<std::uint8_t>(n >> 8U);
        for (std::size_t k = 0U; op == I2cOp::write && k < n; ++k)
            program_[len + kI2cOpHeader + k] = r.write_data.data[k];
        len += need;
    }

    std::lock_guard<std::mutex> lock(bus_lock_);
    Result<void> out = control_.control_out(static_cast<std::uint8_t>(Request::i2c_run),
                                            0U, channel_, ByteView{program_.data(), len});
    if (!out)
        return out;

    const std::size_t want = kI2cResultHeader + total_read;
    Result<std::size_t> in = control_.control_in(static_cast<std::uint8_t>(Request::i2c_result),
                                                 0U, channel_,
                                                 MutableByteView{result_.data(), want});
    if (!in)
        return Result<void>::failure(in.error());
    if (in.value() < kI2cResultHeader)
        return Result<void>::failure(Error::PROTOCOL_ERROR);

    last_status_ = static_cast<I2cStatus>(result_[0]);
    last_failed_op_ = result_[1];
    const std::size_t nread =
        static_cast<std::size_t>(result_[2]) | (static_cast<std::size_t>(result_[3]) << 8U);
    if (last_status_ == I2cStatus::nak_address || last_status_ == I2cStatus::nak_data ||
        last_status_ == I2cStatus::bus_error)
        return Result<void>::failure(Error::USB_IO);
    if (last_status_ != I2cStatus::ok)
        return Result<void>::failure(Error::PROTOCOL_ERROR);
    if (nread != total_read || in.value() != kI2cResultHeader + nread)
        return Result<void>::failure(Error::PROTOCOL_ERROR);

    // Scatter the read bytes back in request order.
    std::size_t pos = kI2cResultHeader;
    for (std::size_t i = 0U; i < count; ++i) {
        BridgeI2cRequest& r = requests[i];
        if (r.type != BridgeI2cRequestType::read)
            continue;
        for (std::size_t k = 0U; k < r.read_data.size; ++k)
            r.read_data.data[k] = result_[pos++];
    }
    return Result<void>::success();
}

}  // namespace otr::fx3
