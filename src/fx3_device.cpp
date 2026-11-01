// SPDX-License-Identifier: GPL-2.0-only
#include "fx3_device.h"

#include <chrono>

namespace otr::fx3 {

Fx3Device::Fx3Device(UsbBackend& backend) noexcept : backend_(backend)
{
    backend_.set_sink(this);
}

Fx3Device::~Fx3Device() noexcept
{
    (void)stop_stream();
    backend_.set_sink(nullptr);
}

Result<void> Fx3Device::control_out(std::uint8_t request, std::uint16_t value,
                                    std::uint16_t index, ByteView data) noexcept
{
    return backend_.control_out(request, value, index, data);
}

Result<std::size_t> Fx3Device::control_in(std::uint8_t request, std::uint16_t value,
                                          std::uint16_t index, MutableByteView data) noexcept
{
    return backend_.control_in(request, value, index, data);
}

Result<std::size_t> Fx3Device::bulk_read(std::uint8_t endpoint, MutableByteView output,
                                         Timeout timeout,
                                         px4::userland::BulkReadObservation* observation) noexcept
{
    Result<std::size_t> r = backend_.bulk_read(endpoint, output, timeout);
    if (observation != nullptr) {
        observation->completion_error = r ? Error::OK : r.error();
        observation->transferred = r ? r.value() : 0U;
    }
    return r;
}

Result<std::size_t> Fx3Device::bulk_write(std::uint8_t endpoint, ByteView input,
                                          Timeout timeout) noexcept
{
    return backend_.bulk_write(endpoint, input, timeout);
}

Result<void> Fx3Device::submit_locked(std::size_t index) noexcept
{
    Slot& s = slots_[index];
    s.state = SlotState::submitted;
    s.order = next_submit_++;
    s.error = Error::OK;
    s.length = 0U;
    ++outstanding_;
    Result<void> r = backend_.submit_read(index, config_.endpoint, s.buffer.get(),
                                          config_.transfer_size);
    if (!r) {
        // Keep the slot's place in the delivery order and report the error there.
        --outstanding_;
        s.state = SlotState::completed;
        s.error = r.error();
    }
    return r;
}

Result<void> Fx3Device::start_stream(const StreamConfig& config) noexcept
{
    if (config.transfer_count == 0U || config.transfer_count > kMaxTransfers ||
        config.transfer_size == 0U || config.transfer_size > kMaxTransferSize)
        return Result<void>::failure(Error::INVALID_ARGUMENT);
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_ || outstanding_ != 0U)
        return Result<void>::failure(Error::BUSY);
    config_ = config;
    slots_.clear();
    slots_.resize(config.transfer_count);
    for (Slot& s : slots_) {
        s.buffer.reset(new (std::nothrow) std::uint8_t[config.transfer_size]);
        if (!s.buffer) {
            slots_.clear();
            return Result<void>::failure(Error::INTERNAL);
        }
    }
    next_submit_ = 0U;
    next_delivery_ = 0U;
    delivered_ = kMaxTransfers;
    stopping_ = false;
    active_ = true;
    for (std::size_t i = 0U; i < slots_.size(); ++i) {
        Result<void> r = submit_locked(i);
        if (!r) {
            cancel_all_locked();
            active_ = false;
            return r;
        }
    }
    return Result<void>::success();
}

void Fx3Device::on_read_complete(std::size_t slot, Error error, std::size_t length) noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (slot >= slots_.size() || slots_[slot].state != SlotState::submitted)
        return;
    Slot& s = slots_[slot];
    s.state = SlotState::completed;
    s.error = error;
    s.length = length;
    --outstanding_;
    cv_.notify_all();
}

Result<StreamEvent> Fx3Device::wait_stream(Timeout timeout) noexcept
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (!active_ || stopping_)
        return Result<StreamEvent>::failure(Error::NOT_READY);

    // Return the previously delivered buffer to the device first.
    if (delivered_ != kMaxTransfers) {
        const std::size_t index = delivered_;
        delivered_ = kMaxTransfers;
        if (slots_[index].state == SlotState::delivered) {
            Result<void> r = submit_locked(index);
            if (!r)
                return Result<StreamEvent>::failure(r.error());
        }
    }

    auto ready_index = [this]() noexcept -> std::size_t {
        for (std::size_t i = 0U; i < slots_.size(); ++i)
            if (slots_[i].state == SlotState::completed && slots_[i].order == next_delivery_)
                return i;
        return kMaxTransfers;
    };

    std::size_t index = ready_index();
    if (index == kMaxTransfers) {
        const bool woke = cv_.wait_for(lock, std::chrono::milliseconds(timeout.milliseconds),
                                       [&]() noexcept {
                                           return stopping_ || !active_ ||
                                                  (index = ready_index()) != kMaxTransfers;
                                       });
        if (stopping_ || !active_)
            return Result<StreamEvent>::failure(Error::NOT_READY);
        if (!woke)
            return Result<StreamEvent>::failure(Error::TIMEOUT);
    }

    Slot& s = slots_[index];
    ++next_delivery_;
    if (s.error != Error::OK) {
        const Error e = s.error;
        if (e == Error::DISCONNECTED) {
            s.state = SlotState::idle;   // the device is gone: never resubmit
        } else {
            s.state = SlotState::delivered;
            delivered_ = index;          // resubmitted on the next wait
        }
        return Result<StreamEvent>::failure(e);
    }
    s.state = SlotState::delivered;
    delivered_ = index;
    const StreamEventKind kind = s.length < config_.transfer_size ? StreamEventKind::short_transfer
                                                                   : StreamEventKind::data;
    return Result<StreamEvent>::success(StreamEvent{kind, s.buffer.get(), s.length});
}

void Fx3Device::cancel_all_locked() noexcept
{
    for (std::size_t i = 0U; i < slots_.size(); ++i)
        if (slots_[i].state == SlotState::submitted)
            backend_.cancel_read(i);
}

Result<void> Fx3Device::cancel_stream() noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_)
        return Result<void>::failure(Error::NOT_READY);
    stopping_ = true;
    delivered_ = kMaxTransfers;
    cancel_all_locked();
    cv_.notify_all();
    return Result<void>::success();
}

Result<void> Fx3Device::stop_stream() noexcept
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (!active_ && outstanding_ == 0U)
        return Result<void>::success();
    stopping_ = true;
    delivered_ = kMaxTransfers;
    cancel_all_locked();
    cv_.notify_all();
    // Every submitted read reports exactly once (completed or cancelled).
    const bool drained = cv_.wait_for(lock, std::chrono::seconds(5),
                                      [this]() noexcept { return outstanding_ == 0U; });
    active_ = false;
    if (!drained)
        return Result<void>::failure(Error::TIMEOUT);   // buffers stay owned until drained
    slots_.clear();
    return Result<void>::success();
}

bool Fx3Device::stream_active() const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    return active_ && !stopping_;
}

}  // namespace otr::fx3
