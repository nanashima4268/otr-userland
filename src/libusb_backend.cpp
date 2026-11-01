// SPDX-License-Identifier: GPL-2.0-only
#include "libusb_backend.h"

#include <libusb.h>

#include <climits>
#include <new>

namespace otr::fx3 {

namespace {

constexpr std::uint8_t kVendorOut =
    LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;
constexpr std::uint8_t kVendorIn =
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE;

Error map_error(int rc) noexcept
{
    switch (rc) {
    case LIBUSB_ERROR_TIMEOUT:
        return Error::TIMEOUT;
    case LIBUSB_ERROR_NO_DEVICE:
        return Error::DISCONNECTED;
    case LIBUSB_ERROR_BUSY:
        return Error::BUSY;
    case LIBUSB_ERROR_NOT_FOUND:
        return Error::NOT_FOUND;
    case LIBUSB_ERROR_INVALID_PARAM:
        return Error::INVALID_ARGUMENT;
    case LIBUSB_ERROR_NOT_SUPPORTED:
        return Error::UNSUPPORTED;
    default:
        return Error::USB_IO;
    }
}

Error map_status(libusb_transfer_status status) noexcept
{
    switch (status) {
    case LIBUSB_TRANSFER_COMPLETED:
        return Error::OK;
    case LIBUSB_TRANSFER_CANCELLED:
        return Error::NOT_READY;
    case LIBUSB_TRANSFER_NO_DEVICE:
        return Error::DISCONNECTED;
    case LIBUSB_TRANSFER_TIMED_OUT:
        return Error::TIMEOUT;
    default:   // STALL, OVERFLOW, ERROR
        return Error::USB_IO;
    }
}

}  // namespace

Result<std::unique_ptr<LibusbBackend>> LibusbBackend::open(UsbIds ids) noexcept
{
    std::unique_ptr<LibusbBackend> self(new (std::nothrow) LibusbBackend());
    if (!self)
        return Result<std::unique_ptr<LibusbBackend>>::failure(Error::INTERNAL);
    int rc = libusb_init(&self->ctx_);
    if (rc != 0)
        return Result<std::unique_ptr<LibusbBackend>>::failure(map_error(rc));
    self->handle_ = libusb_open_device_with_vid_pid(self->ctx_, ids.vid, ids.pid);
    if (self->handle_ == nullptr)
        return Result<std::unique_ptr<LibusbBackend>>::failure(Error::NOT_FOUND);
    rc = libusb_claim_interface(self->handle_, 0);
    if (rc != 0)
        return Result<std::unique_ptr<LibusbBackend>>::failure(map_error(rc));
    self->claimed_ = true;
    for (auto& t : self->transfers_) {
        t = libusb_alloc_transfer(0);
        if (t == nullptr)
            return Result<std::unique_ptr<LibusbBackend>>::failure(Error::INTERNAL);
    }
    self->running_ = true;
    self->events_ = std::thread([p = self.get()]() noexcept { p->event_loop(); });
    return Result<std::unique_ptr<LibusbBackend>>::success(std::move(self));
}

LibusbBackend::~LibusbBackend() noexcept
{
    // Fx3Device::stop_stream() must have drained all transfers before this.
    running_ = false;
    if (handle_ != nullptr && claimed_)
        libusb_release_interface(handle_, 0);   // also wakes the event thread
    if (events_.joinable())
        events_.join();
    for (auto& t : transfers_)
        if (t != nullptr)
            libusb_free_transfer(t);
    if (handle_ != nullptr)
        libusb_close(handle_);
    if (ctx_ != nullptr)
        libusb_exit(ctx_);
}

void LibusbBackend::event_loop() noexcept
{
    while (running_) {
        timeval tv{0, 100000};   // 100 ms, so shutdown is noticed promptly
        libusb_handle_events_timeout_completed(ctx_, &tv, nullptr);
    }
}

Result<void> LibusbBackend::control_out(std::uint8_t request, std::uint16_t value,
                                        std::uint16_t index, ByteView data) noexcept
{
    if (data.size > 0xFFFFU)
        return Result<void>::failure(Error::INVALID_ARGUMENT);
    const int rc = libusb_control_transfer(handle_, kVendorOut, request, value, index,
                                           const_cast<unsigned char*>(data.data),
                                           static_cast<std::uint16_t>(data.size),
                                           kControlTimeoutMs);
    if (rc < 0)
        return Result<void>::failure(map_error(rc));
    if (static_cast<std::size_t>(rc) != data.size)
        return Result<void>::failure(Error::USB_IO);
    return Result<void>::success();
}

Result<std::size_t> LibusbBackend::control_in(std::uint8_t request, std::uint16_t value,
                                              std::uint16_t index, MutableByteView data) noexcept
{
    if (data.size > 0xFFFFU)
        return Result<std::size_t>::failure(Error::INVALID_ARGUMENT);
    const int rc = libusb_control_transfer(handle_, kVendorIn, request, value, index,
                                           data.data, static_cast<std::uint16_t>(data.size),
                                           kControlTimeoutMs);
    if (rc < 0)
        return Result<std::size_t>::failure(map_error(rc));
    return Result<std::size_t>::success(static_cast<std::size_t>(rc));
}

Result<std::size_t> LibusbBackend::bulk_read(std::uint8_t endpoint, MutableByteView data,
                                             Timeout timeout) noexcept
{
    if (data.size > INT_MAX)
        return Result<std::size_t>::failure(Error::INVALID_ARGUMENT);
    int done = 0;
    const int rc = libusb_bulk_transfer(handle_, endpoint, data.data,
                                        static_cast<int>(data.size), &done,
                                        timeout.milliseconds);
    if (rc != 0 && !(rc == LIBUSB_ERROR_TIMEOUT && done > 0))
        return Result<std::size_t>::failure(map_error(rc));
    return Result<std::size_t>::success(static_cast<std::size_t>(done));
}

Result<std::size_t> LibusbBackend::bulk_write(std::uint8_t endpoint, ByteView data,
                                              Timeout timeout) noexcept
{
    if (data.size > INT_MAX)
        return Result<std::size_t>::failure(Error::INVALID_ARGUMENT);
    int done = 0;
    const int rc = libusb_bulk_transfer(handle_, endpoint,
                                        const_cast<unsigned char*>(data.data),
                                        static_cast<int>(data.size), &done,
                                        timeout.milliseconds);
    if (rc != 0)
        return Result<std::size_t>::failure(map_error(rc));
    return Result<std::size_t>::success(static_cast<std::size_t>(done));
}

void LibusbBackend::transfer_callback(libusb_transfer* transfer) noexcept
{
    auto* self = static_cast<LibusbBackend*>(transfer->user_data);
    std::size_t slot = 0U;
    while (slot < self->transfers_.size() && self->transfers_[slot] != transfer)
        ++slot;
    ReadCompletionSink* sink = self->sink_.load();
    if (sink != nullptr && slot < self->transfers_.size())
        sink->on_read_complete(slot, map_status(transfer->status),
                               static_cast<std::size_t>(transfer->actual_length));
}

Result<void> LibusbBackend::submit_read(std::size_t slot, std::uint8_t endpoint,
                                        std::uint8_t* buffer, std::size_t size) noexcept
{
    if (slot >= transfers_.size() || size > INT_MAX)
        return Result<void>::failure(Error::INVALID_ARGUMENT);
    libusb_transfer* t = transfers_[slot];
    // No timeout: the FX3 sends only full 16 KiB buffers (D-43), so idle
    // receivers can legitimately be silent for a long time.
    libusb_fill_bulk_transfer(t, handle_, endpoint, buffer, static_cast<int>(size),
                              &LibusbBackend::transfer_callback, this, 0U);
    const int rc = libusb_submit_transfer(t);
    if (rc != 0)
        return Result<void>::failure(map_error(rc));
    return Result<void>::success();
}

void LibusbBackend::cancel_read(std::size_t slot) noexcept
{
    if (slot < transfers_.size())
        libusb_cancel_transfer(transfers_[slot]);
}

}  // namespace otr::fx3
