// SPDX-License-Identifier: GPL-2.0-only
// Four CXD2856ER/CXD2858ER receivers of the OTR-4TS3-U1.
//
// Modelled on px4-userland's Mlt5PeFrontend (itself derived from px4_drv
// pxmlt_device.c), with the board differences of docs/design.md 3.2:
//  - every receiver has its own I2C channel (TCA9546A, D-24), so the bus lock
//    is per receiver and every demodulator is at 0x64;
//  - power and reset are per channel FX3 GPIOs (D-06), sequenced like the
//    reference board: power on, 80 ms, release reset, 20 ms;
//  - capture start/stop sets the channel's FPGA CH_ENABLE bit (D-47, D-50).
// Every receiver accepts ISDB-T and ISDB-S. No LNB supply in v1 (D-30).
#ifndef OTR_FRONTEND_H
#define OTR_FRONTEND_H

#include "cxd2856er.h"
#include "cxd2858er.h"
#include "otr_board.h"
#include "px4/card_service.h"
#include "px4/tuner_service.h"

#include <array>
#include <cstdint>
#include <mutex>

namespace otr {

using px4::userland::BridgeI2cMaster;
using px4::userland::Cxd2856er;
using px4::userland::Cxd2856erSystem;
using px4::userland::Cxd2858er;
using px4::userland::Cxd285xDelay;

inline constexpr std::uint8_t kReceiverCount = 4U;
inline constexpr std::uint8_t kDemodAddress = 0x64U;   // D-24
inline constexpr std::uint32_t kPowerOnDelayMs = 80U;
inline constexpr std::uint32_t kResetReleaseDelayMs = 20U;

// Monotonic clock and sleep, shared by the drivers and the tuner service.
class OtrTime final : public Cxd285xDelay, public px4::userland::TunerServiceTime {
public:
    std::uint64_t monotonic_ms() noexcept override;
    void sleep_ms(std::uint32_t milliseconds) noexcept override;
};

enum class ReceiverState : std::uint8_t { closed, open, tuned, capturing };

// Power/reset of one channel; OtrBoard in production, a fake in tests.
class ChannelPower {
public:
    virtual ~ChannelPower() noexcept = default;
    virtual Result<void> set_channel_power(std::uint8_t channel, bool power,
                                           bool reset_released) noexcept = 0;
    virtual Result<void> set_channel_capture(std::uint8_t channel, bool enabled) noexcept = 0;
};

class BoardChannelPower final : public ChannelPower {
public:
    explicit BoardChannelPower(OtrBoard& board) noexcept : board_(board) {}
    Result<void> set_channel_power(std::uint8_t channel, bool power,
                                   bool reset_released) noexcept override;
    Result<void> set_channel_capture(std::uint8_t channel, bool enabled) noexcept override;

private:
    OtrBoard& board_;
};

class OtrFrontend final {
public:
    OtrFrontend(std::array<BridgeI2cMaster*, kReceiverCount> buses, ChannelPower& power,
                Cxd285xDelay& delay) noexcept;
    ~OtrFrontend() noexcept;
    OtrFrontend(const OtrFrontend&) = delete;
    OtrFrontend& operator=(const OtrFrontend&) = delete;

    Result<void> open_receiver(std::uint8_t receiver) noexcept;
    Result<void> close_receiver(std::uint8_t receiver) noexcept;
    Result<void> tune(std::uint8_t receiver, Cxd2856erSystem system,
                      std::uint32_t frequency_khz) noexcept;
    Result<void> select_satellite_slot(std::uint8_t receiver, std::uint8_t slot) noexcept;
    Result<void> select_satellite_tsid(std::uint8_t receiver, std::uint16_t tsid) noexcept;
    Result<bool> is_terrestrial_locked(std::uint8_t receiver) noexcept;
    Result<bool> is_satellite_locked(std::uint8_t receiver) noexcept;
    Result<void> start_capture(std::uint8_t receiver) noexcept;
    Result<void> stop_capture(std::uint8_t receiver) noexcept;
    ReceiverState receiver_state(std::uint8_t receiver) const noexcept;

private:
    struct Receiver final {
        Receiver(BridgeI2cMaster& bus, Cxd285xDelay& delay) noexcept
            : demod(bus, kDemodAddress, delay), tuner(demod, delay)
        {
        }
        Cxd2856er demod;
        Cxd2858er tuner;
        ReceiverState state = ReceiverState::closed;
        std::mutex bus_mutex;
    };

    Result<void> power_off(std::uint8_t receiver) noexcept;

    ChannelPower& power_;
    Cxd285xDelay& delay_;
    std::array<Receiver, kReceiverCount> receivers_;
    mutable std::mutex state_mutex_;
};

// TunerServiceBackend for the frontend (modelled on Mlt5PeTunerBackend).
class OtrTunerBackend final : public px4::userland::TunerServiceBackend {
public:
    explicit OtrTunerBackend(OtrFrontend& frontend) noexcept : frontend_(frontend) {}

    std::uint8_t receiver_count() const noexcept override { return kReceiverCount; }
    bool receiver_supports(std::uint8_t receiver,
                           px4::userland::ipc::System system) const noexcept override;
    bool selects_satellite_stream_before_tune() const noexcept override { return true; }
    bool requires_terrestrial_lock_settle() const noexcept override { return true; }

    Result<void> open_receiver(std::uint8_t receiver) noexcept override;
    Result<void> tune_terrestrial(std::uint8_t receiver, std::uint32_t frequency_khz,
                                  std::uint32_t timeout_ms) noexcept override;
    Result<void> tune_satellite(std::uint8_t receiver, std::uint32_t frequency_khz,
                                std::uint32_t timeout_ms) noexcept override;
    Result<px4::userland::TunerHzProgramming> tune_terrestrial_hz(
        std::uint8_t receiver, std::uint64_t frequency_hz,
        std::uint32_t timeout_ms) noexcept override;
    Result<px4::userland::TunerHzProgramming> tune_satellite_hz(
        std::uint8_t receiver, std::uint64_t frequency_hz,
        std::uint32_t timeout_ms) noexcept override;
    px4::userland::TunerStrictProfile strict_profile(
        std::uint8_t receiver, px4::userland::ipc::System system) const noexcept override;
    Result<bool> is_locked(std::uint8_t receiver,
                           px4::userland::ipc::System system) noexcept override;
    Result<void> select_satellite_slot(std::uint8_t receiver, std::uint8_t slot,
                                       std::uint32_t timeout_ms) noexcept override;
    Result<void> select_satellite_tsid(std::uint8_t receiver, std::uint16_t tsid,
                                       std::uint32_t timeout_ms) noexcept override;
    Result<void> close_receiver(std::uint8_t receiver) noexcept override;
    Result<void> start_capture(std::uint8_t receiver,
                               px4::userland::ipc::System system) noexcept override;
    Result<void> stop_capture(std::uint8_t receiver,
                              px4::userland::ipc::System system) noexcept override;
    Result<void> begin_tune_power(std::uint8_t receiver, px4::userland::ipc::System system,
                                  std::uint8_t lnb_voltage) noexcept override;
    Result<void> commit_tune_power(std::uint8_t receiver) noexcept override;
    Result<void> rollback_tune_power(std::uint8_t receiver) noexcept override;
    void mark_receiver_disconnected(std::uint8_t receiver) noexcept override;
    Result<void> shutdown() noexcept override;

private:
    OtrFrontend& frontend_;
};

// No card reader on the board (D-13): every card operation is UNSUPPORTED.
class AbsentCardBackend final : public px4::userland::CardServiceBackend {
public:
    Result<void> set_power(bool on) noexcept override;
    Result<void> initialize_uart() noexcept override;
    Result<bool> detect_card() noexcept override;
};

class AbsentCardProtocol final : public px4::userland::CardProtocolSession {
public:
    Result<void> initialize() noexcept override;
    Result<std::size_t> transmit(dtv::userland::ByteView apdu,
                                 dtv::userland::MutableByteView response) noexcept override;
    bool initialized() const noexcept override { return false; }
    const px4::userland::CardAtrBytes& atr() const noexcept override { return atr_; }
    void invalidate() noexcept override {}

private:
    px4::userland::CardAtrBytes atr_{};
};

}  // namespace otr

#endif  // OTR_FRONTEND_H
