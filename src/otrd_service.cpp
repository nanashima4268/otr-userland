// SPDX-License-Identifier: GPL-2.0-only
// otrd composition (modelled on px4-userland tools/px4d.cpp serve_core):
//   UsbBackend -> Fx3Device -> OtrBoard (GPIO, FPGA, STREAM)
//   Fx3I2cMaster x4 -> OtrFrontend -> OtrTunerBackend
//   OtrStreamTransport -> Q3U4StreamDataPlane (tags 1..4 -> receivers 0..3)
//   EnclosureFamily -> EnclosureContext (listener, control, data)
#include "otrd_service.h"

#include "fx3_i2c.h"
#include "otr_board.h"
#include "otr_frontend.h"
#include "otr_transport.h"
#include "px4/posix_tuner_nonce.h"
#include "px4/q3u4_stream.h"

#include <dtv/userland/host_listener_context/enclosure_context.h>
#include <dtv/userland/host_runtime/family.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifndef OTR_USERLAND_VERSION
#define OTR_USERLAND_VERSION "0.0.0-dev"
#endif
#ifndef DTV_USERLAND_CORE_VERSION
#define DTV_USERLAND_CORE_VERSION "unknown"
#endif

namespace otr {

namespace {

using px4::userland::Error;
using px4::userland::Result;
namespace wire = dtv::userland::wire;
namespace listener = dtv::userland::host_listener_context;
namespace host = dtv::userland::host_runtime;

std::string hex_serial(const std::array<std::uint8_t, 8>& uid)
{
    static const char digits[] = "0123456789abcdef";
    std::string s;
    for (std::uint8_t b : uid) {
        s.push_back(digits[b >> 4U]);
        s.push_back(digits[b & 0x0FU]);
    }
    return s;
}

template <std::size_t N>
bool set_text(wire::WString<N>& target, const std::string& value)
{
    if (value.size() > N) return false;
    target.length = static_cast<std::uint16_t>(value.size());
    std::copy(value.begin(), value.end(), target.bytes.begin());
    return true;
}

wire::ListResponse make_list(const std::string& serial, const wire::Epoch& epoch,
                             const wire::Producer& producer)
{
    wire::ListResponse list;
    list.epoch = epoch;
    list.producer = producer;
    list.snapshot.sequence = 1U;
    list.topology.state = wire::TopologyState::complete;
    list.topology.expected_function_count.present = true;
    list.topology.expected_function_count.value = 1U;
    list.enclosure.present = true;
    list.enclosure.value.id = 0U;
    list.enclosure.value.model.present = true;
    (void)set_text(list.enclosure.value.model.value.name, "OTR-4TS3-U1");
    list.enclosure.value.identity.serial_state = wire::IdentitySerialState::present;
    list.enclosure.value.identity.serial.present = true;
    (void)set_text(list.enclosure.value.identity.serial.value, serial);
    list.enclosure.value.identity.uniqueness = wire::Uniqueness::unique_in_scan;
    list.enclosure.value.limits.max_concurrent_streams.present = true;
    list.enclosure.value.limits.max_concurrent_streams.value = otr::kReceiverCount;

    list.functions.count = 1U;
    auto& function = list.functions.items[0];
    function.id = 1U;
    function.vid = 0x1209U;
    function.pid = 0x0001U;
    function.mode = wire::Mode::runtime;
    function.serial_state = wire::SerialState::present;
    function.serial.present = true;
    (void)set_text(function.serial.value, serial);
    function.model.present = true;
    (void)set_text(function.model.value.name, "OTR-4TS3-U1");
    function.identification_reason = wire::IdentificationReason::none;
    function.present = true;

    list.receivers.present = true;
    list.receivers.value.count = otr::kReceiverCount;
    for (std::uint8_t r = 0U; r < otr::kReceiverCount; ++r) {
        auto& rec = list.receivers.value.items[r];
        rec.id = r;
        rec.systems.mask = 3U;   // ISDB-T and ISDB-S
        rec.binding.present = true;
        rec.binding.value.function_id = 1U;
        rec.binding.value.local_id = r;
        rec.capabilities.measurement_mask = wire::kMetricMaskLock;
    }
    list.readers.present = true;
    list.readers.value.count = 0U;   // no card reader (D-13)
    return list;
}

class OtrFamily final : public host::EnclosureFamily {
public:
    OtrFamily(px4::userland::TunerServiceBackend& tuner, px4::userland::CardServiceBackend& card,
              px4::userland::CardProtocolSession& protocol, px4::userland::TunerServiceTime& time,
              px4::userland::Q3U4StreamDataPlane& stream) noexcept
        : tuner_(tuner), card_(card), protocol_(protocol), time_(time), stream_(stream)
    {
    }
    Result<void> shutdown() noexcept override { return stream_.shutdown(); }
    px4::userland::TunerServiceBackend& tuner_backend() noexcept override { return tuner_; }
    px4::userland::CardServiceBackend& card_backend() noexcept override { return card_; }
    px4::userland::CardProtocolSession& card_protocol() noexcept override { return protocol_; }
    px4::userland::TunerServiceTime& time() noexcept override { return time_; }
    px4::userland::TunerStreamControl* stream_control() noexcept override { return &stream_; }

private:
    px4::userland::TunerServiceBackend& tuner_;
    px4::userland::CardServiceBackend& card_;
    px4::userland::CardProtocolSession& protocol_;
    px4::userland::TunerServiceTime& time_;
    px4::userland::Q3U4StreamDataPlane& stream_;
};

int status(Error e) { return e == Error::OK ? 0 : 69; }

int fail(const char* what, Error e)
{
    std::fprintf(stderr, "otrd: %s: %s\n", what, px4::userland::error_string(e));
    return status(e);
}

}  // namespace

int otrd_serve(const OtrdOptions& options, fx3::UsbBackend& usb, const std::atomic<bool>& stop,
               const std::function<void(const std::string&)>& on_ready) noexcept
{
    otr::fx3::Fx3Device device(usb);
    otr::OtrBoard board(device);
    const auto info = board.info();
    if (!info) return fail("GET_INFO", info.error());
    const std::string serial = info.value().flash_present ? hex_serial(info.value().flash_uid)
                                                          : std::string("0000000000000001");
    std::fprintf(stderr, "otrd: board %s, firmware %u.%u, FPGA %s\n", serial.c_str(),
                 info.value().fw_major, info.value().fw_minor,
                 info.value().fpga_source == otr::fx3::FpgaSource::flash ? "from flash"
                 : info.value().fpga_configured                          ? "loaded"
                                                                         : "not configured");
    if (!options.fpga.empty()) {
        std::ifstream f(options.fpga, std::ios::binary);
        const std::vector<std::uint8_t> bits((std::istreambuf_iterator<char>(f)),
                                             std::istreambuf_iterator<char>());
        if (!f.good() && !f.eof()) return fail("read bitstream", Error::NOT_FOUND);
        const auto loaded = board.load_fpga(bits.data(), bits.size());
        if (!loaded) return fail("FPGA load", loaded.error());
    }
    const auto checked = board.check_fpga();
    if (!checked) return fail("FPGA ID/version", checked.error());
    // Everything off before the service starts (D-50 stop state).
    (void)board.disable_all_channels();
    (void)board.set_stream(false);

    otr::OtrTime time;
    std::mutex i2c_lock;
    otr::fx3::Fx3I2cMaster bus0(device, i2c_lock, 0U), bus1(device, i2c_lock, 1U),
        bus2(device, i2c_lock, 2U), bus3(device, i2c_lock, 3U);
    otr::BoardChannelPower power(board);
    otr::OtrFrontend frontend({&bus0, &bus1, &bus2, &bus3}, power, time);
    otr::OtrTunerBackend tuner(frontend);
    otr::AbsentCardBackend card;
    otr::AbsentCardProtocol protocol;
    otr::OtrStreamTransport transport(device, board);
    // The PX-MLT5PE layout maps tags 1..4 to receivers 0..3, which is the
    // FPGA tagging (D-38); receiver 4 is never attached.
    auto stream = px4::userland::Q3U4StreamDataPlane::create_mlt_family(
        transport, px4::userland::DeviceModel::px_mlt5pe);
    if (!stream) return fail("stream data plane", stream.error());

    // ---- service ----
    auto nonce = std::make_unique<px4::userland::ipc::posix::PosixTunerNonceSource>();
    const auto epoch_seed = nonce->generate();
    if (!epoch_seed) return fail("nonce", epoch_seed.error());
    const wire::Epoch epoch = epoch_seed.value();
    wire::Producer producer;
    (void)set_text(producer.product, "otr-userland");
    (void)set_text(producer.version, OTR_USERLAND_VERSION);
    producer.core_version.present = true;
    (void)set_text(producer.core_version.value, DTV_USERLAND_CORE_VERSION);

    listener::ContextConfig config;
    config.runtime_directory = options.runtime_dir;
    config.product = "otr-userland";
    config.instance = options.instance.empty() ? serial : options.instance;
    config.access = options.group ? host::EndpointAccess::shared_group
                               : host::EndpointAccess::private_user;
    config.server.epoch = epoch;
    config.server.producer = producer;
    config.server.features = wire::kFeatureStateEvent | wire::kFeaturePidFilter;
    config.canonical_list = make_list(serial, epoch, producer);
    config.context.epoch = epoch;
    config.context.state = wire::ContextState::ready;
    config.context.instance.present = true;
    (void)set_text(config.context.instance.value, config.instance);
    for (std::size_t i = 0U; i < 8U; ++i)
        config.stream_engine_epoch |= static_cast<std::uint64_t>(epoch[i]) << (i * 8U);
    if (config.stream_engine_epoch == 0U) config.stream_engine_epoch = 1U;
    config.stream_arm_budget_ms = 5000U;

    auto family = std::make_unique<OtrFamily>(tuner, card, protocol, time, *stream.value());
    auto context = listener::EnclosureContext::assemble(std::move(family), std::move(nonce), config);
    if (!context) return fail("enclosure context", context.error());
    const auto reserved = context.value()->reserve_namespace();
    if (!reserved) return fail("endpoint reservation", reserved.error());
    const auto published = context.value()->publish();
    if (!published) return fail("endpoint publication", published.error());

    std::fprintf(stderr, "otrd ready: instance=%s\n", config.instance.c_str());
    if (on_ready) on_ready(config.instance);
    Error loop_error = Error::OK;
    // FPGA drop counters (FIFO overflow, D-47): logged when they change.
    std::array<std::uint32_t, otr::kReceiverCount> drops{};
    std::uint64_t next_poll = time.monotonic_ms() + 10000U;
    while (!stop.load()) {
        if (time.monotonic_ms() >= next_poll) {
            next_poll = time.monotonic_ms() + 10000U;
            for (unsigned ch = 0U; ch < otr::kReceiverCount; ++ch) {
                const auto d = board.fpga_read(otr::fpga::drop_cnt(ch));
                if (d && d.value() != drops[ch]) {
                    std::fprintf(stderr, "otrd: channel %u FPGA drops %u (+%u)\n", ch, d.value(),
                                 static_cast<unsigned>((d.value() - drops[ch]) & 0xFFFFU));
                    drops[ch] = d.value();
                }
            }
        }
        const auto polled = context.value()->poll_once(px4::userland::Timeout{100U});
        if (!polled) {
            loop_error = polled.error();
            std::fprintf(stderr, "otrd: poll: %s\n", px4::userland::error_string(loop_error));
            break;
        }
    }
    for (;;) {
        const auto closed = context.value()->close(px4::userland::Timeout{100U});
        if (!closed) {
            if (loop_error == Error::OK) loop_error = closed.error();
            break;
        }
        if (closed.value() == listener::CloseState::quiesced) break;
    }
    (void)board.disable_all_channels();
    (void)board.set_stream(false);
    return status(loop_error);
}

}  // namespace otr
