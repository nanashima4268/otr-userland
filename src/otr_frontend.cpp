// SPDX-License-Identifier: GPL-2.0-only
// Receiver sequencing follows px4-userland's Mlt5PeFrontend/Mlt5PeTunerBackend
// (derived from px4_drv pxmlt_device.c, Copyright (c) 2018-2021 nns779).
#include "otr_frontend.h"
#include "px4_tuner_hz.h"

#include <chrono>
#include <thread>

namespace otr {

using px4::userland::TunerHzProgramming;
using px4::userland::TunerStrictProfile;
using px4::userland::ipc::System;

std::uint64_t OtrTime::monotonic_ms() noexcept
{
    return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::steady_clock::now().time_since_epoch())
                                          .count());
}

void OtrTime::sleep_ms(std::uint32_t milliseconds) noexcept
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

// ---- board channel power ----

Result<void> BoardChannelPower::set_channel_power(std::uint8_t channel, bool power,
                                                  bool reset_released) noexcept
{
    if (channel >= kReceiverCount) return Result<void>::failure(Error::INVALID_ARGUMENT);
    const std::uint32_t mask = gpio_ch_pwr(channel) | gpio_ch_rst_n(channel);
    const std::uint32_t value = (power ? gpio_ch_pwr(channel) : 0U) |
                                (reset_released ? gpio_ch_rst_n(channel) : 0U);
    return board_.set_gpio(value, mask);
}

Result<void> BoardChannelPower::set_channel_capture(std::uint8_t channel, bool enabled) noexcept
{
    return board_.set_channel_enabled(channel, enabled);
}

// ---- frontend ----

OtrFrontend::OtrFrontend(std::array<BridgeI2cMaster*, kReceiverCount> buses, ChannelPower& power,
                         Cxd285xDelay& delay) noexcept
    : power_(power), delay_(delay),
      receivers_{{Receiver(*buses[0], delay), Receiver(*buses[1], delay),
                  Receiver(*buses[2], delay), Receiver(*buses[3], delay)}}
{
}

OtrFrontend::~OtrFrontend() noexcept
{
    for (std::uint8_t r = 0U; r < kReceiverCount; ++r) (void)close_receiver(r);
}

Result<void> OtrFrontend::power_off(std::uint8_t receiver) noexcept
{
    // Reset first, then power, as on the reference board.
    const auto reset = power_.set_channel_power(receiver, true, false);
    const auto off = power_.set_channel_power(receiver, false, false);
    if (!reset) return reset;
    return off;
}

Result<void> OtrFrontend::open_receiver(std::uint8_t receiver) noexcept
{
    if (receiver >= kReceiverCount) return Result<void>::failure(Error::INVALID_ARGUMENT);
    Receiver& target = receivers_[receiver];
    std::lock_guard<std::mutex> bus(target.bus_mutex);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (target.state != ReceiverState::closed) return Result<void>::failure(Error::BUSY);
    }
    auto result = power_.set_channel_power(receiver, true, false);
    if (result) {
        delay_.sleep_ms(kPowerOnDelayMs);
        result = power_.set_channel_power(receiver, true, true);
    }
    if (result) {
        delay_.sleep_ms(kResetReleaseDelayMs);
        result = target.demod.initialize();
        if (result) {
            result = target.tuner.initialize();
            if (result) result = target.demod.configure_ts_output();
            if (!result) {
                target.tuner.terminate();
                target.demod.terminate();
            }
        }
    }
    if (!result) {
        (void)power_off(receiver);   // the original error remains the result
        return result;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    target.state = ReceiverState::open;
    return result;
}

Result<void> OtrFrontend::close_receiver(std::uint8_t receiver) noexcept
{
    if (receiver >= kReceiverCount) return Result<void>::failure(Error::INVALID_ARGUMENT);
    Receiver& target = receivers_[receiver];
    std::lock_guard<std::mutex> bus(target.bus_mutex);
    bool was_capturing = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (target.state == ReceiverState::closed) return Result<void>::success();
        was_capturing = target.state == ReceiverState::capturing;
        target.state = ReceiverState::closed;
    }
    if (was_capturing) (void)power_.set_channel_capture(receiver, false);
    // pxmlt_chrdev_release ignores both termination results.
    target.tuner.terminate();
    target.demod.terminate();
    return power_off(receiver);
}

Result<void> OtrFrontend::tune(std::uint8_t receiver, Cxd2856erSystem system,
                               std::uint32_t frequency_khz) noexcept
{
    if (receiver >= kReceiverCount ||
        (system != Cxd2856erSystem::isdb_t && system != Cxd2856erSystem::isdb_s))
        return Result<void>::failure(Error::INVALID_ARGUMENT);
    Receiver& target = receivers_[receiver];
    std::lock_guard<std::mutex> bus(target.bus_mutex);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (target.state == ReceiverState::closed) return Result<void>::failure(Error::NOT_READY);
        if (target.state == ReceiverState::capturing) return Result<void>::failure(Error::BUSY);
        target.state = ReceiverState::open;
    }
    auto result = target.demod.wakeup(system);
    if (result) {
        result = system == Cxd2856erSystem::isdb_t ? target.tuner.set_params_t(frequency_khz)
                                                   : target.tuner.set_params_s(frequency_khz);
    }
    if (result) result = target.demod.post_tune();
    if (result) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        target.state = ReceiverState::tuned;
    }
    return result;
}

Result<void> OtrFrontend::select_satellite_slot(std::uint8_t receiver, std::uint8_t slot) noexcept
{
    if (receiver >= kReceiverCount) return Result<void>::failure(Error::INVALID_ARGUMENT);
    if (receiver_state(receiver) == ReceiverState::closed)
        return Result<void>::failure(Error::NOT_READY);
    std::lock_guard<std::mutex> bus(receivers_[receiver].bus_mutex);
    return receivers_[receiver].demod.set_slot_isdbs(slot);
}

Result<void> OtrFrontend::select_satellite_tsid(std::uint8_t receiver, std::uint16_t tsid) noexcept
{
    if (receiver >= kReceiverCount) return Result<void>::failure(Error::INVALID_ARGUMENT);
    if (receiver_state(receiver) == ReceiverState::closed)
        return Result<void>::failure(Error::NOT_READY);
    std::lock_guard<std::mutex> bus(receivers_[receiver].bus_mutex);
    return receivers_[receiver].demod.set_tsid_isdbs(tsid);
}

Result<bool> OtrFrontend::is_terrestrial_locked(std::uint8_t receiver) noexcept
{
    if (receiver >= kReceiverCount) return Result<bool>::failure(Error::INVALID_ARGUMENT);
    Receiver& target = receivers_[receiver];
    std::lock_guard<std::mutex> bus(target.bus_mutex);
    if (target.demod.system() != Cxd2856erSystem::isdb_t)
        return Result<bool>::failure(Error::NOT_READY);
    const auto lock_state = target.demod.is_ts_locked_isdbt();
    if (!lock_state) return Result<bool>::failure(lock_state.error());
    // An "unlocked" report ends the wait at once (pxmlt_chrdev_check_lock).
    if (!lock_state.value().locked && lock_state.value().unlocked)
        return Result<bool>::failure(Error::TIMEOUT);
    return Result<bool>::success(lock_state.value().locked);
}

Result<bool> OtrFrontend::is_satellite_locked(std::uint8_t receiver) noexcept
{
    if (receiver >= kReceiverCount) return Result<bool>::failure(Error::INVALID_ARGUMENT);
    Receiver& target = receivers_[receiver];
    std::lock_guard<std::mutex> bus(target.bus_mutex);
    if (target.demod.system() != Cxd2856erSystem::isdb_s)
        return Result<bool>::failure(Error::NOT_READY);
    return target.demod.is_ts_locked_isdbs();
}

Result<void> OtrFrontend::start_capture(std::uint8_t receiver) noexcept
{
    if (receiver >= kReceiverCount) return Result<void>::failure(Error::INVALID_ARGUMENT);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (receivers_[receiver].state != ReceiverState::tuned)
        return Result<void>::failure(Error::NOT_READY);
    const auto enabled = power_.set_channel_capture(receiver, true);
    if (!enabled) return enabled;
    receivers_[receiver].state = ReceiverState::capturing;
    return enabled;
}

Result<void> OtrFrontend::stop_capture(std::uint8_t receiver) noexcept
{
    if (receiver >= kReceiverCount) return Result<void>::failure(Error::INVALID_ARGUMENT);
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (receivers_[receiver].state != ReceiverState::capturing) return Result<void>::success();
    receivers_[receiver].state = ReceiverState::tuned;
    // The FPGA discards what is left of this channel (D-50).
    return power_.set_channel_capture(receiver, false);
}

ReceiverState OtrFrontend::receiver_state(std::uint8_t receiver) const noexcept
{
    if (receiver >= kReceiverCount) return ReceiverState::closed;
    std::lock_guard<std::mutex> lock(state_mutex_);
    return receivers_[receiver].state;
}

// ---- tuner service backend ----

bool OtrTunerBackend::receiver_supports(std::uint8_t receiver, System system) const noexcept
{
    return receiver < kReceiverCount && (system == System::ISDB_T || system == System::ISDB_S);
}

Result<void> OtrTunerBackend::open_receiver(std::uint8_t receiver) noexcept
{
    return frontend_.open_receiver(receiver);
}

Result<void> OtrTunerBackend::tune_terrestrial(std::uint8_t receiver, std::uint32_t frequency_khz,
                                               std::uint32_t timeout_ms) noexcept
{
    (void)timeout_ms;   // bounded by the fixed I2C and sleep steps
    return frontend_.tune(receiver, Cxd2856erSystem::isdb_t, frequency_khz);
}

Result<void> OtrTunerBackend::tune_satellite(std::uint8_t receiver, std::uint32_t frequency_khz,
                                             std::uint32_t timeout_ms) noexcept
{
    (void)timeout_ms;
    return frontend_.tune(receiver, Cxd2856erSystem::isdb_s, frequency_khz);
}

Result<TunerHzProgramming> OtrTunerBackend::tune_terrestrial_hz(std::uint8_t receiver,
                                                                std::uint64_t frequency_hz,
                                                                std::uint32_t timeout_ms) noexcept
{
    const auto khz = px4::userland::px4_frequency_khz(frequency_hz, System::ISDB_T);
    if (!khz || !receiver_supports(receiver, System::ISDB_T))
        return Result<TunerHzProgramming>::failure(khz ? Error::UNSUPPORTED : khz.error());
    const auto tuned = tune_terrestrial(receiver, khz.value(), timeout_ms);
    if (!tuned) return Result<TunerHzProgramming>::failure(tuned.error());
    return Result<TunerHzProgramming>::success(px4::userland::px4_programming(khz.value()));
}

Result<TunerHzProgramming> OtrTunerBackend::tune_satellite_hz(std::uint8_t receiver,
                                                              std::uint64_t frequency_hz,
                                                              std::uint32_t timeout_ms) noexcept
{
    const auto khz = px4::userland::px4_frequency_khz(frequency_hz, System::ISDB_S);
    if (!khz || !receiver_supports(receiver, System::ISDB_S))
        return Result<TunerHzProgramming>::failure(khz ? Error::UNSUPPORTED : khz.error());
    const auto tuned = tune_satellite(receiver, khz.value(), timeout_ms);
    if (!tuned) return Result<TunerHzProgramming>::failure(tuned.error());
    return Result<TunerHzProgramming>::success(px4::userland::px4_programming(khz.value()));
}

TunerStrictProfile OtrTunerBackend::strict_profile(std::uint8_t receiver,
                                                   System system) const noexcept
{
    if (!receiver_supports(receiver, system)) return {};
    return px4::userland::px4_strict_profile(system, false);   // 0 V only (D-30)
}

Result<bool> OtrTunerBackend::is_locked(std::uint8_t receiver, System system) noexcept
{
    if (system == System::ISDB_T) return frontend_.is_terrestrial_locked(receiver);
    if (system == System::ISDB_S) return frontend_.is_satellite_locked(receiver);
    return Result<bool>::failure(Error::INVALID_ARGUMENT);
}

Result<void> OtrTunerBackend::select_satellite_slot(std::uint8_t receiver, std::uint8_t slot,
                                                    std::uint32_t timeout_ms) noexcept
{
    (void)timeout_ms;
    return frontend_.select_satellite_slot(receiver, slot);
}

Result<void> OtrTunerBackend::select_satellite_tsid(std::uint8_t receiver, std::uint16_t tsid,
                                                    std::uint32_t timeout_ms) noexcept
{
    (void)timeout_ms;
    return frontend_.select_satellite_tsid(receiver, tsid);
}

Result<void> OtrTunerBackend::close_receiver(std::uint8_t receiver) noexcept
{
    return frontend_.close_receiver(receiver);
}

Result<void> OtrTunerBackend::start_capture(std::uint8_t receiver, System system) noexcept
{
    (void)system;
    return frontend_.start_capture(receiver);
}

Result<void> OtrTunerBackend::stop_capture(std::uint8_t receiver, System system) noexcept
{
    (void)system;
    return frontend_.stop_capture(receiver);
}

Result<void> OtrTunerBackend::begin_tune_power(std::uint8_t receiver, System system,
                                               std::uint8_t lnb_voltage) noexcept
{
    if (!receiver_supports(receiver, system)) return Result<void>::failure(Error::INVALID_ARGUMENT);
    // No LNB supply in v1 (D-30): only 0 V is accepted.
    if (lnb_voltage != 0U) return Result<void>::failure(Error::UNSUPPORTED);
    return Result<void>::success();
}

Result<void> OtrTunerBackend::commit_tune_power(std::uint8_t receiver) noexcept
{
    (void)receiver;
    return Result<void>::success();
}

Result<void> OtrTunerBackend::rollback_tune_power(std::uint8_t receiver) noexcept
{
    (void)receiver;
    return Result<void>::success();
}

void OtrTunerBackend::mark_receiver_disconnected(std::uint8_t receiver) noexcept
{
    (void)receiver;
}

Result<void> OtrTunerBackend::shutdown() noexcept
{
    return Result<void>::success();
}

// ---- absent card ----

Result<void> AbsentCardBackend::set_power(bool on) noexcept
{
    (void)on;
    return Result<void>::failure(Error::UNSUPPORTED);
}

Result<void> AbsentCardBackend::initialize_uart() noexcept
{
    return Result<void>::failure(Error::UNSUPPORTED);
}

Result<bool> AbsentCardBackend::detect_card() noexcept
{
    return Result<bool>::failure(Error::UNSUPPORTED);
}

Result<void> AbsentCardProtocol::initialize() noexcept
{
    return Result<void>::failure(Error::UNSUPPORTED);
}

Result<std::size_t> AbsentCardProtocol::transmit(dtv::userland::ByteView apdu,
                                                 dtv::userland::MutableByteView response) noexcept
{
    (void)apdu;
    (void)response;
    return Result<std::size_t>::failure(Error::UNSUPPORTED);
}

}  // namespace otr
