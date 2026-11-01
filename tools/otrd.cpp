// SPDX-License-Identifier: GPL-2.0-only
// otrd: owns one OTR-4TS3-U1 and serves its four receivers over the
// dtv-userland-core control/data endpoints, like px4d (docs/design.md 1).
//
// Composition (modelled on px4-userland tools/px4d.cpp serve_core):
//   LibusbBackend -> Fx3Device -> OtrBoard (GPIO, FPGA, STREAM)
//   Fx3I2cMaster x4 -> OtrFrontend -> OtrTunerBackend
//   OtrStreamTransport -> Q3U4StreamDataPlane (tags 1..4 -> receivers 0..3)
//   EnclosureFamily -> EnclosureContext (listener, control, data)
// POSIX only for now; Windows follows once the board runs (D-10).
#include "fx3_device.h"
#include "fx3_i2c.h"
#include "libusb_backend.h"
#include "otr_board.h"
#include "otr_frontend.h"
#include "otr_transport.h"
#include "px4/posix_tuner_nonce.h"
#include "px4/q3u4_stream.h"

#include <dtv/userland/host_listener_context/enclosure_context.h>
#include <dtv/userland/host_runtime/family.h>

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#ifndef OTR_USERLAND_VERSION
#define OTR_USERLAND_VERSION "0.0.0-dev"
#endif
#ifndef DTV_USERLAND_CORE_VERSION
#define DTV_USERLAND_CORE_VERSION "unknown"
#endif

namespace {

using px4::userland::Error;
using px4::userland::Result;
namespace wire = dtv::userland::wire;
namespace listener = dtv::userland::host_listener_context;
namespace host = dtv::userland::host_runtime;

volatile std::sig_atomic_t g_stop = 0;
extern "C" void on_signal(int) { g_stop = 1; }

struct Args final {
    std::string runtime_dir;
    std::string instance;
    std::string device;   // serial (16 hex digits)
    std::string fpga;     // bitstream to load instead of the one in flash
    bool group = false;
    bool help = false;
    bool version = false;
    std::string error;
};

void usage()
{
    std::fprintf(stderr,
                 "usage: otrd [--device SERIAL] [--instance NAME] [--runtime-dir DIR]\n"
                 "            [--group] [--fpga BITSTREAM]\n"
                 "  --device SERIAL   board serial (16 hex digits, the USB serial number)\n"
                 "  --instance NAME   endpoint instance (default: the serial, lowercase)\n"
                 "  --runtime-dir DIR endpoint directory (default: the platform runtime dir)\n"
                 "  --group           allow the owning group to connect\n"
                 "  --fpga FILE       load this FPGA bitstream (development; normally the\n"
                 "                    FX3 loads the one in the boot flash, D-49)\n");
}

Args parse(int argc, char** argv)
{
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string opt = argv[i];
        auto value = [&](std::string& out) {
            if (i + 1 >= argc || argv[i + 1][0] == '\0') {
                a.error = opt + " needs a value";
                return;
            }
            out = argv[++i];
        };
        if (opt == "--help" || opt == "-h") a.help = true;
        else if (opt == "--version") a.version = true;
        else if (opt == "--group") a.group = true;
        else if (opt == "--runtime-dir") value(a.runtime_dir);
        else if (opt == "--instance") value(a.instance);
        else if (opt == "--device") value(a.device);
        else if (opt == "--fpga") value(a.fpga);
        else a.error = "unknown option " + opt;
        if (!a.error.empty()) break;
    }
    return a;
}

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

int main(int argc, char** argv)
{
    const Args args = parse(argc, argv);
    if (!args.error.empty()) {
        std::fprintf(stderr, "otrd: %s\n", args.error.c_str());
        usage();
        return 2;
    }
    if (args.help) {
        usage();
        return 0;
    }
    if (args.version) {
        std::printf("otrd %s (dtv-userland-core %s)\n", OTR_USERLAND_VERSION,
                    DTV_USERLAND_CORE_VERSION);
        return 0;
    }

    // ---- hardware ----
    otr::fx3::UsbIds ids;
    ids.serial = args.device;
    auto usb = otr::fx3::LibusbBackend::open(ids);
    if (!usb) return fail(usb.error() == Error::BUSY ? "more than one board; use --device"
                                                      : "open board", usb.error());
    otr::fx3::Fx3Device device(*usb.value());
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
    if (!args.fpga.empty()) {
        std::ifstream f(args.fpga, std::ios::binary);
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
    config.runtime_directory = args.runtime_dir;
    config.product = "otr-userland";
    config.instance = args.instance.empty() ? serial : args.instance;
    config.access = args.group ? host::EndpointAccess::shared_group
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

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::fprintf(stderr, "otrd ready: instance=%s\n", config.instance.c_str());
    Error loop_error = Error::OK;
    while (!g_stop) {
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
