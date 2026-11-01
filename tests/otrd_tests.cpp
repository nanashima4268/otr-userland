// SPDX-License-Identifier: GPL-2.0-only
// otrd composition without hardware: a fake FX3 board (vendor requests, FPGA
// registers with the D-50 path reset, and a bulk stream of tagged packets)
// under the real Fx3Device, OtrBoard, OtrStreamTransport and px4-userland's
// Q3U4StreamDataPlane; plus the OtrFrontend power sequencing.
#include "fx3_device.h"
#include "fx3_protocol.h"
#include "otr_board.h"
#include "otr_frontend.h"
#include "otr_transport.h"
#include "px4/q3u4_stream.h"
#include "fake_board.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace otr;
using namespace otr::test;
using px4::userland::Error;
using px4::userland::Result;

namespace {

int g_failures = 0;
#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                         #cond);                                                 \
            ++g_failures;                                                        \
        }                                                                        \
    } while (0)


void test_fake_raw()
{
    FakeBoard fake;
    fx3::Fx3Device dev(fake);
    OtrBoard board(dev);
    CHECK(board.set_stream(true));
    CHECK(dev.start_stream(px4::userland::StreamConfig{0x81U, 16384U, 2U}));
    const auto ev = dev.wait_stream(px4::userland::Timeout{2000U});
    CHECK(ev);
    if (ev) {
        ChannelGen g0(0U), g1(1U), g2(2U);
        auto p0 = g0.next(); auto p1 = g1.next(); auto p2 = g2.next();
        CHECK(std::equal(p0.begin(), p0.end(), ev.value().data));
        CHECK(std::equal(p1.begin(), p1.end(), ev.value().data + kPkt));
        CHECK(std::equal(p2.begin(), p2.end(), ev.value().data + 2 * kPkt));
    }
    CHECK(dev.stop_stream());
}

void test_shape()
{
    const auto s = fx3_stream_shape(px4::userland::StreamConfig{0x84U, 188U * 816U, 6U});
    CHECK(s.endpoint == 0x81U);
    CHECK(s.transfer_size % 16384U == 0U);
    CHECK(s.transfer_size >= 188U * 816U);
    CHECK(s.transfer_count == 6U);
    CHECK(fx3_stream_shape({0x84U, 100U, 0U}).transfer_size == 16384U);
    CHECK(fx3_stream_shape({0x84U, 100U, 0U}).transfer_count == 1U);
    CHECK(fx3_stream_shape({0x84U, 8U << 20U, 99U}).transfer_size == fx3::Fx3Device::kMaxTransferSize);
    CHECK(fx3_stream_shape({0x84U, 8U << 20U, 99U}).transfer_count == fx3::Fx3Device::kMaxTransfers);
}

void test_board_info()
{
    FakeBoard fake;
    fx3::Fx3Device dev(fake);
    OtrBoard board(dev);
    const auto info = board.info();
    CHECK(info);
    if (info) {
        CHECK(info.value().fpga_source == fx3::FpgaSource::flash);
        CHECK(info.value().flash_present);
        CHECK(info.value().dma_buffer_size == 16384U);
        CHECK(info.value().flash_uid[0] == 0xA0U && info.value().flash_uid[7] == 0xA7U);
    }
    CHECK(board.check_fpga());
    CHECK(board.set_channel_enabled(2U, true));
    CHECK(board.set_channel_enabled(0U, true));
    CHECK(fake.ch_enable == 0x5U);
    CHECK(board.set_channel_enabled(2U, false));
    CHECK(fake.ch_enable == 0x1U);
    CHECK(!board.set_channel_enabled(4U, true));
}

// The data plane over our transport: receiver 2 gets channel 2's packets,
// restored (sync 0x47), in order; start/stop follow the D-50 sequence.
void test_data_plane()
{
    FakeBoard fake;
    fx3::Fx3Device dev(fake);
    OtrBoard board(dev);
    OtrStreamTransport transport(dev, board);
    auto plane = px4::userland::Q3U4StreamDataPlane::create_mlt_family(
        transport, px4::userland::DeviceModel::px_mlt5pe);
    CHECK(plane);
    if (!plane) return;

    px4::userland::TunerAttachment a;
    a.owner_client_id = 1U;
    a.lease_id = 1U;
    a.attachment_id = 1U;
    a.receiver = 2U;
    a.system = px4::userland::ipc::System::ISDB_T;
    const auto attached = plane.value()->attach(a);
    CHECK(attached);
    if (!attached) return;

    std::vector<std::uint8_t> got;
    std::vector<std::uint8_t> buf(188U * 64U);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (got.size() < 300U * kPkt && std::chrono::steady_clock::now() < deadline) {
        const auto r = plane.value()->read(a, dtv::userland::MutableByteView{buf.data(), buf.size()},
                                           px4::userland::Timeout{200U});
        if (!r) {
            std::fprintf(stderr, "read: %s\n", px4::userland::error_string(r.error()));
            break;
        }
        got.insert(got.end(), buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(r.value().bytes));
    }
    CHECK(got.size() >= 300U * kPkt);
    CHECK(got.size() % kPkt == 0U);
    // The fake streams far faster than real time, so the first packet read
    // may be far into the sequence: find it, then the rest must follow
    // the generator exactly (sync byte restored to 0x47).
    ChannelGen ref(2U);
    std::size_t start = 0U;
    bool found = false;
    std::vector<std::uint8_t> p;
    for (; start < 20000000U; ++start) {
        p = ref.next();
        p[0] = 0x47U;
        if (got.size() >= kPkt && std::equal(p.begin(), p.end(), got.begin())) { found = true; break; }
    }
    CHECK(found);
    std::size_t mismatches = 0U;
    for (std::size_t k = 1; found && k * kPkt < got.size(); ++k) {
        p = ref.next();
        p[0] = 0x47U;
        if (!std::equal(p.begin(), p.end(), got.begin() + static_cast<std::ptrdiff_t>(k * kPkt)))
            ++mismatches;
    }
    CHECK(mismatches == 0U);
    std::fprintf(stderr, "data plane: %zu packets for receiver 2 from #%zu, %zu out of sequence\n",
                 got.size() / kPkt, start, mismatches);

    CHECK(fake.last_endpoint == 0x81U);
    CHECK(fake.last_size % 16384U == 0U);
    CHECK(plane.value()->detach(a));
    (void)plane.value()->release_final(a);
    CHECK(plane.value()->shutdown());

    // D-50: path reset before STREAM 1, never while streaming; STREAM 0 at the end.
    std::size_t reset_at = fake.log.size(), on_at = fake.log.size(), off_at = fake.log.size();
    for (std::size_t i = 0; i < fake.log.size(); ++i) {
        if (fake.log[i] == "path_reset" && reset_at == fake.log.size()) reset_at = i;
        if (fake.log[i] == "stream1" && on_at == fake.log.size()) on_at = i;
        if (fake.log[i] == "stream0") off_at = i;
        CHECK(fake.log[i].rfind("BAD:", 0) != 0);
    }
    CHECK(reset_at < on_at);
    CHECK(on_at < off_at && off_at < fake.log.size());
}

// ---- frontend power sequencing ----
class FakePower final : public ChannelPower {
public:
    std::vector<std::string> calls;
    Result<void> set_channel_power(std::uint8_t ch, bool power, bool rst) noexcept override
    {
        calls.push_back("pwr" + std::to_string(ch) + (power ? "+" : "-") + (rst ? "R" : "r"));
        return Result<void>::success();
    }
    Result<void> set_channel_capture(std::uint8_t ch, bool en) noexcept override
    {
        calls.push_back("cap" + std::to_string(ch) + (en ? "+" : "-"));
        return Result<void>::success();
    }
};

class FakeDelay final : public px4::userland::Cxd285xDelay {
public:
    std::vector<std::uint32_t> sleeps;
    void sleep_ms(std::uint32_t ms) noexcept override { sleeps.push_back(ms); }
};

class DeadBus final : public px4::userland::BridgeI2cMaster {
public:
    Result<void> request(px4::userland::BridgeI2cRequest*, std::size_t) noexcept override
    {
        return Result<void>::failure(Error::USB_IO);
    }
};

void test_frontend_power()
{
    DeadBus b0, b1, b2, b3;
    FakePower power;
    FakeDelay delay;
    {
        OtrFrontend fe({&b0, &b1, &b2, &b3}, power, delay);
        const auto opened = fe.open_receiver(1U);
        CHECK(!opened);   // the demodulator does not answer
        CHECK(fe.receiver_state(1U) == ReceiverState::closed);
        CHECK(!fe.start_capture(1U));
        CHECK(!fe.open_receiver(4U));
    }
    // power on with reset held, 80 ms, release reset, 20 ms, then (failed
    // init) reset and power off.
    const std::vector<std::string> want = {"pwr1+r", "pwr1+R", "pwr1+r", "pwr1-r"};
    CHECK(power.calls == want);
    CHECK(delay.sleeps.size() >= 2U && delay.sleeps[0] == 80U && delay.sleeps[1] == 20U);
}

void test_backend_profile()
{
    DeadBus b0, b1, b2, b3;
    FakePower power;
    FakeDelay delay;
    OtrFrontend fe({&b0, &b1, &b2, &b3}, power, delay);
    OtrTunerBackend be(fe);
    using px4::userland::ipc::System;
    CHECK(be.receiver_count() == 4U);
    CHECK(be.receiver_supports(3U, System::ISDB_S));
    CHECK(!be.receiver_supports(4U, System::ISDB_T));
    CHECK(be.begin_tune_power(0U, System::ISDB_S, 0U));
    CHECK(be.begin_tune_power(0U, System::ISDB_S, 15U).error() == Error::UNSUPPORTED);
    CHECK(be.strict_profile(0U, System::ISDB_S).lnb_voltage_mask == 0x01U);
    CHECK(!be.tune_terrestrial_hz(0U, 10000000U, 1000U));   // below range
}

}  // namespace

int main()
{
    test_fake_raw();
    test_shape();
    test_board_info();
    test_data_plane();
    test_frontend_power();
    test_backend_profile();
    if (g_failures == 0) std::printf("otrd_tests: all passed\n");
    return g_failures == 0 ? 0 : 1;
}
