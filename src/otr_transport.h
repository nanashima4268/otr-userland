// SPDX-License-Identifier: GPL-2.0-only
// The Transport that px4-userland's Q3U4StreamDataPlane drives, on top of
// Fx3Device and the board controls.
//
// The data plane asks for the IT930x stream shape (EP 0x84, 188 x 816-byte
// transfers). The FX3 sends whole 16 KiB DMA buffers on EP 0x81, so the
// endpoint is replaced and the transfer size rounded up to a multiple of the
// DMA buffer (a short final buffer cannot then overflow a transfer).
//
// start_stream also performs the D-50 sequence: FPGA path reset (the FX3
// stream is stopped at that point), then STREAM 1. stop_stream ends with
// STREAM 0. CH_ENABLE belongs to the receivers (OtrFrontend).
#ifndef OTR_TRANSPORT_H
#define OTR_TRANSPORT_H

#include "fx3_device.h"
#include "otr_board.h"

namespace otr {

using px4::userland::StreamConfig;
using px4::userland::StreamEvent;
using px4::userland::Timeout;

inline constexpr std::uint8_t kFx3TsEndpoint = 0x81U;
inline constexpr std::size_t kFx3DmaBuffer = 16384U;

// Pure function, unit-tested: the shape actually used for a requested one.
StreamConfig fx3_stream_shape(const StreamConfig& requested) noexcept;

class OtrStreamTransport final : public px4::userland::Transport {
public:
    OtrStreamTransport(fx3::Fx3Device& device, OtrBoard& board) noexcept
        : device_(device), board_(board)
    {
    }

    Result<std::size_t> bulk_read(std::uint8_t endpoint, px4::userland::MutableByteView data,
                                  Timeout timeout,
                                  px4::userland::BulkReadObservation* observation =
                                      nullptr) noexcept override;
    Result<std::size_t> bulk_write(std::uint8_t endpoint, px4::userland::ByteView data,
                                   Timeout timeout) noexcept override;
    Result<void> start_stream(const StreamConfig& config) noexcept override;
    Result<StreamEvent> wait_stream(Timeout timeout) noexcept override;
    Result<void> cancel_stream() noexcept override;
    Result<void> stop_stream() noexcept override;
    bool stream_active() const noexcept override;

private:
    fx3::Fx3Device& device_;
    OtrBoard& board_;
};

}  // namespace otr

#endif  // OTR_TRANSPORT_H
