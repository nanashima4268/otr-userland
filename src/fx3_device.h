// SPDX-License-Identifier: GPL-2.0-only
// FX3 device: control requests (Fx3Control) plus the dtv-userland-core
// Transport stream contract on top of a small USB backend seam.
//
// Stream contract (the same as px4-userland's LibusbTransport):
//  - start_stream() submits `transfer_count` reads of `transfer_size`.
//  - wait_stream() returns completed transfers in submission order, not in
//    completion order. The returned data stays valid until the next
//    wait_stream()/cancel_stream()/stop_stream(); the next wait_stream()
//    resubmits that buffer first.
//  - A transfer shorter than transfer_size is reported as short_transfer.
//  - TIMEOUT means no transfer completed in time; the stream keeps running.
//  - DISCONNECTED ends the stream: the slot is not resubmitted.
#ifndef OTR_FX3_DEVICE_H
#define OTR_FX3_DEVICE_H

#include "fx3_control.h"
#include "px4/transport.h"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace otr::fx3 {

using px4::userland::Error;
using px4::userland::StreamConfig;
using px4::userland::StreamEvent;
using px4::userland::StreamEventKind;
using px4::userland::Timeout;

// Completion callbacks from the backend's event context.
class ReadCompletionSink {
public:
    virtual ~ReadCompletionSink() noexcept = default;
    virtual void on_read_complete(std::size_t slot, Error error,
                                  std::size_t length) noexcept = 0;
};

// Minimal USB backend: implemented with libusb (libusb_backend.cpp) and by
// a fake in the tests.
class UsbBackend {
public:
    virtual ~UsbBackend() noexcept = default;
    virtual Result<void> control_out(std::uint8_t request, std::uint16_t value,
                                     std::uint16_t index, ByteView data) noexcept = 0;
    virtual Result<std::size_t> control_in(std::uint8_t request, std::uint16_t value,
                                           std::uint16_t index,
                                           MutableByteView data) noexcept = 0;
    virtual Result<std::size_t> bulk_read(std::uint8_t endpoint, MutableByteView data,
                                          Timeout timeout) noexcept = 0;
    virtual Result<std::size_t> bulk_write(std::uint8_t endpoint, ByteView data,
                                           Timeout timeout) noexcept = 0;
    // Asynchronous bulk read into `buffer`. Completion (including
    // cancellation, reported as Error::NOT_READY) arrives exactly once
    // through the sink.
    virtual Result<void> submit_read(std::size_t slot, std::uint8_t endpoint,
                                     std::uint8_t* buffer, std::size_t size) noexcept = 0;
    virtual void cancel_read(std::size_t slot) noexcept = 0;
    virtual void set_sink(ReadCompletionSink* sink) noexcept = 0;
};

class Fx3Device final : public Fx3Control,
                        public px4::userland::Transport,
                        private ReadCompletionSink {
public:
    static constexpr std::size_t kMaxTransfers = 32U;
    static constexpr std::size_t kMaxTransferSize = 1U << 20U;

    explicit Fx3Device(UsbBackend& backend) noexcept;
    ~Fx3Device() noexcept override;
    Fx3Device(const Fx3Device&) = delete;
    Fx3Device& operator=(const Fx3Device&) = delete;

    // Fx3Control
    Result<void> control_out(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                             ByteView data) noexcept override;
    Result<std::size_t> control_in(std::uint8_t request, std::uint16_t value,
                                   std::uint16_t index, MutableByteView data) noexcept override;

    // Transport
    Result<std::size_t> bulk_read(std::uint8_t endpoint, MutableByteView output,
                                  Timeout timeout,
                                  px4::userland::BulkReadObservation* observation =
                                      nullptr) noexcept override;
    Result<std::size_t> bulk_write(std::uint8_t endpoint, ByteView input,
                                   Timeout timeout) noexcept override;
    Result<void> start_stream(const StreamConfig& config) noexcept override;
    Result<StreamEvent> wait_stream(Timeout timeout) noexcept override;
    Result<void> cancel_stream() noexcept override;
    Result<void> stop_stream() noexcept override;
    bool stream_active() const noexcept override;

private:
    enum class SlotState : std::uint8_t { idle, submitted, completed, delivered };
    struct Slot final {
        std::unique_ptr<std::uint8_t[]> buffer;
        SlotState state = SlotState::idle;
        std::uint64_t order = 0U;
        Error error = Error::OK;
        std::size_t length = 0U;
    };

    void on_read_complete(std::size_t slot, Error error, std::size_t length) noexcept override;
    Result<void> submit_locked(std::size_t index) noexcept;
    void cancel_all_locked() noexcept;

    UsbBackend& backend_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<Slot> slots_;
    StreamConfig config_{};
    bool active_ = false;
    bool stopping_ = false;
    std::size_t outstanding_ = 0U;
    std::uint64_t next_submit_ = 0U;
    std::uint64_t next_delivery_ = 0U;
    std::size_t delivered_ = kMaxTransfers;   // index of the slot handed out, or none
};

}  // namespace otr::fx3

#endif  // OTR_FX3_DEVICE_H
