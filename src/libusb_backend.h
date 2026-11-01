// SPDX-License-Identifier: GPL-2.0-only
// libusb implementation of UsbBackend for the OTR-4TS3-U1 FX3.
// A dedicated thread runs libusb event handling so that synchronous control
// transfers (I2C) and the asynchronous TS stream can run side by side.
#ifndef OTR_LIBUSB_BACKEND_H
#define OTR_LIBUSB_BACKEND_H

#include "fx3_device.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

struct libusb_context;
struct libusb_device_handle;
struct libusb_transfer;

namespace otr::fx3 {

struct UsbIds final {
    std::uint16_t vid = 0x1209U;   // pid.codes test VID/PID during development (D-31)
    std::uint16_t pid = 0x0001U;
    // Empty: the first matching board. Otherwise the USB serial string, the
    // 16 hex digits of the boot flash unique ID (D-49), compared ignoring case.
    std::string serial;
};

inline constexpr std::uint8_t kTsEndpoint = 0x81U;   // EP1 IN, bulk
inline constexpr std::uint32_t kControlTimeoutMs = 1000U;

class LibusbBackend final : public UsbBackend {
public:
    // Opens the matching device and claims interface 0. More than one match
    // without a serial is BUSY (ambiguous).
    static Result<std::unique_ptr<LibusbBackend>> open(UsbIds ids) noexcept;
    ~LibusbBackend() noexcept override;
    LibusbBackend(const LibusbBackend&) = delete;
    LibusbBackend& operator=(const LibusbBackend&) = delete;

    const std::string& serial() const noexcept { return serial_; }

    Result<void> control_out(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                             ByteView data) noexcept override;
    Result<std::size_t> control_in(std::uint8_t request, std::uint16_t value,
                                   std::uint16_t index, MutableByteView data) noexcept override;
    Result<std::size_t> bulk_read(std::uint8_t endpoint, MutableByteView data,
                                  Timeout timeout) noexcept override;
    Result<std::size_t> bulk_write(std::uint8_t endpoint, ByteView data,
                                   Timeout timeout) noexcept override;
    Result<void> submit_read(std::size_t slot, std::uint8_t endpoint, std::uint8_t* buffer,
                             std::size_t size) noexcept override;
    void cancel_read(std::size_t slot) noexcept override;
    void set_sink(ReadCompletionSink* sink) noexcept override { sink_.store(sink); }

private:
    LibusbBackend() noexcept = default;
    static void transfer_callback(libusb_transfer* transfer) noexcept;
    void event_loop() noexcept;

    libusb_context* ctx_ = nullptr;
    libusb_device_handle* handle_ = nullptr;
    std::string serial_;
    bool claimed_ = false;
    std::atomic<bool> running_{false};
    std::thread events_;
    std::atomic<ReadCompletionSink*> sink_{nullptr};
    std::array<libusb_transfer*, Fx3Device::kMaxTransfers> transfers_{};
};

}  // namespace otr::fx3

#endif  // OTR_LIBUSB_BACKEND_H
