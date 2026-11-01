// SPDX-License-Identifier: GPL-2.0-only
#include "otr_transport.h"

#include <algorithm>

namespace otr {

StreamConfig fx3_stream_shape(const StreamConfig& requested) noexcept
{
    StreamConfig shape = requested;
    shape.endpoint = kFx3TsEndpoint;
    std::size_t size = requested.transfer_size;
    if (size < kFx3DmaBuffer) size = kFx3DmaBuffer;
    size = ((size + kFx3DmaBuffer - 1U) / kFx3DmaBuffer) * kFx3DmaBuffer;
    shape.transfer_size = std::min(size, fx3::Fx3Device::kMaxTransferSize);
    shape.transfer_count = std::min(std::max<std::size_t>(requested.transfer_count, 1U),
                                    fx3::Fx3Device::kMaxTransfers);
    return shape;
}

Result<std::size_t> OtrStreamTransport::bulk_read(std::uint8_t endpoint,
                                                  px4::userland::MutableByteView data,
                                                  Timeout timeout,
                                                  px4::userland::BulkReadObservation* observation) noexcept
{
    return device_.bulk_read(endpoint, data, timeout, observation);
}

Result<std::size_t> OtrStreamTransport::bulk_write(std::uint8_t endpoint,
                                                   px4::userland::ByteView data,
                                                   Timeout timeout) noexcept
{
    return device_.bulk_write(endpoint, data, timeout);
}

Result<void> OtrStreamTransport::start_stream(const StreamConfig& config) noexcept
{
    // The FX3 stream is stopped here (D-50): realign the FPGA writer first.
    const auto reset = board_.path_reset();
    if (!reset) return reset;
    const auto on = board_.set_stream(true);
    if (!on) return on;
    const auto started = device_.start_stream(fx3_stream_shape(config));
    if (!started) (void)board_.set_stream(false);
    return started;
}

Result<StreamEvent> OtrStreamTransport::wait_stream(Timeout timeout) noexcept
{
    return device_.wait_stream(timeout);
}

Result<void> OtrStreamTransport::cancel_stream() noexcept
{
    return device_.cancel_stream();
}

Result<void> OtrStreamTransport::stop_stream() noexcept
{
    const auto stopped = device_.stop_stream();
    // STREAM 0 even after a failed stop: the FX3 must drop its partial buffer.
    const auto off = board_.set_stream(false);
    if (!stopped) return stopped;
    return off;
}

bool OtrStreamTransport::stream_active() const noexcept
{
    return device_.stream_active();
}

}  // namespace otr
