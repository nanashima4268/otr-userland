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

constexpr std::size_t kPkt = 188U;

// Same generator as tests/ts_path_tests.cpp and fpga/sim/tb_check.py.
struct Lcg {
    std::uint32_t s;
    std::uint8_t next() noexcept
    {
        s = s * 1103515245U + 12345U;
        return static_cast<std::uint8_t>(s >> 16U);
    }
};

struct ChannelGen {
    std::size_t ch;
    std::size_t index = 0U;
    Lcg rng;
    explicit ChannelGen(std::size_t c) : ch(c), rng{static_cast<std::uint32_t>(0x1234U + c)} {}
    // Packet as the FPGA sends it: sync byte replaced by the channel tag.
    std::vector<std::uint8_t> next()
    {
        std::vector<std::uint8_t> p(kPkt);
        const auto pid = static_cast<std::uint16_t>(0x100U + ch);
        p[0] = static_cast<std::uint8_t>(((ch + 1U) << 4U) | 0x07U);
        p[1] = static_cast<std::uint8_t>(pid >> 8U);
        p[2] = static_cast<std::uint8_t>(pid & 0xFFU);
        p[3] = static_cast<std::uint8_t>(0x10U | (index & 0x0FU));
        for (std::size_t k = 4; k < kPkt; ++k) p[k] = rng.next();
        ++index;
        return p;
    }
};

// ---- fake board ----
class FakeBoard final : public fx3::UsbBackend {
public:
    FakeBoard() : thread_([this] { run(); }) {}
    ~FakeBoard() override
    {
        {
            std::lock_guard<std::mutex> l(m_);
            quit_ = true;
        }
        cv_.notify_all();
        thread_.join();
    }

    std::vector<std::string> log;     // request trace
    std::uint8_t last_endpoint = 0U;
    std::size_t last_size = 0U;
    std::uint32_t ch_enable = 0U;
    std::uint32_t gpio = 0U;

    Result<void> control_out(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                             fx3::ByteView data) noexcept override
    {
        std::lock_guard<std::mutex> l(m_);
        if (request == static_cast<std::uint8_t>(fx3::Request::fpga_reg)) {
            const std::uint32_t v = data.size == 4U
                ? static_cast<std::uint32_t>(data.data[0] | (data.data[1] << 8U) |
                                             (data.data[2] << 16U) |
                                             (static_cast<std::uint32_t>(data.data[3]) << 24U))
                : 0U;
            if (value == static_cast<std::uint8_t>(fpga::Reg::ch_enable)) ch_enable = v;
            if (value == static_cast<std::uint8_t>(fpga::Reg::ctrl) && (v & 1U) != 0U) {
                log.push_back("path_reset");
                // Misalignment model: a reset while streaming is a bug.
                if (stream_on_) log.push_back("BAD:path_reset_while_streaming");
            }
            return Result<void>::success();
        }
        if (request == static_cast<std::uint8_t>(fx3::Request::stream)) {
            stream_on_ = value != 0U;
            log.push_back(value ? "stream1" : "stream0");
            cv_.notify_all();
            return Result<void>::success();
        }
        if (request == static_cast<std::uint8_t>(fx3::Request::gpio)) {
            gpio = (gpio & ~static_cast<std::uint32_t>(index)) | (value & index);
            return Result<void>::success();
        }
        return Result<void>::success();
    }
    Result<std::size_t> control_in(std::uint8_t request, std::uint16_t value, std::uint16_t,
                                   fx3::MutableByteView data) noexcept override
    {
        std::lock_guard<std::mutex> l(m_);
        if (request == static_cast<std::uint8_t>(fx3::Request::fpga_reg) && data.size >= 4U) {
            std::uint32_t v = 0U;
            if (value == static_cast<std::uint8_t>(fpga::Reg::id)) v = fpga::kId;
            if (value == static_cast<std::uint8_t>(fpga::Reg::version)) v = 0x00010000U;
            if (value == static_cast<std::uint8_t>(fpga::Reg::status)) v = 0U;   // reset done
            if (value == static_cast<std::uint8_t>(fpga::Reg::ch_enable)) v = ch_enable;
            for (int i = 0; i < 4; ++i) data.data[i] = static_cast<std::uint8_t>(v >> (8 * i));
            return Result<std::size_t>::success(4U);
        }
        if (request == static_cast<std::uint8_t>(fx3::Request::get_info) && data.size >= 32U) {
            std::uint8_t b[32] = {'O', 'T', 'R', '1', 1, 0, 1, 0, 1, 0, 1, 1};
            b[12] = 0x00; b[13] = 0x40;   // 16384
            for (int i = 0; i < 8; ++i) b[20 + i] = static_cast<std::uint8_t>(0xA0 + i);
            for (int i = 0; i < 32; ++i) data.data[i] = b[i];
            return Result<std::size_t>::success(32U);
        }
        return Result<std::size_t>::failure(Error::UNSUPPORTED);
    }
    Result<std::size_t> bulk_read(std::uint8_t, fx3::MutableByteView, px4::userland::Timeout) noexcept override
    {
        return Result<std::size_t>::failure(Error::UNSUPPORTED);
    }
    Result<std::size_t> bulk_write(std::uint8_t, fx3::ByteView, px4::userland::Timeout) noexcept override
    {
        return Result<std::size_t>::failure(Error::UNSUPPORTED);
    }
    Result<void> submit_read(std::size_t slot, std::uint8_t endpoint, std::uint8_t* buffer,
                             std::size_t size) noexcept override
    {
        std::lock_guard<std::mutex> l(m_);
        last_endpoint = endpoint;
        last_size = size;
        pending_.push_back({slot, buffer, size, false});
        cv_.notify_all();
        return Result<void>::success();
    }
    void cancel_read(std::size_t slot) noexcept override
    {
        std::lock_guard<std::mutex> l(m_);
        for (auto& p : pending_)
            if (p.slot == slot) p.cancelled = true;
        cv_.notify_all();
    }
    void set_sink(fx3::ReadCompletionSink* sink) noexcept override
    {
        std::lock_guard<std::mutex> l(m_);
        sink_ = sink;
    }

private:
    struct Pending {
        std::size_t slot;
        std::uint8_t* buffer;
        std::size_t size;
        bool cancelled;
    };

    // Interleaved tagged stream: channels 0..3 round robin, as the FPGA mux.
    std::uint8_t next_byte()
    {
        if (cur_pos_ == cur_.size()) {
            cur_ = gens_[rr_].next();
            rr_ = (rr_ + 1U) % 4U;
            cur_pos_ = 0U;
        }
        return cur_[cur_pos_++];
    }

    void run()
    {
        std::unique_lock<std::mutex> l(m_);
        for (;;) {
            cv_.wait_for(l, std::chrono::milliseconds(1));
            if (quit_) return;
            if (pending_.empty() || sink_ == nullptr) continue;
            Pending p = pending_.front();
            if (!p.cancelled && !stream_on_) continue;
            pending_.pop_front();
            fx3::ReadCompletionSink* sink = sink_;
            std::size_t len = 0U;
            Error e = Error::NOT_READY;
            if (!p.cancelled) {
                for (std::size_t i = 0; i < p.size; ++i) p.buffer[i] = next_byte();
                len = p.size;
                e = Error::OK;
            }
            l.unlock();
            sink->on_read_complete(p.slot, e, len);
            l.lock();
        }
    }

    std::mutex m_;
    std::condition_variable cv_;
    std::deque<Pending> pending_;
    fx3::ReadCompletionSink* sink_ = nullptr;
    bool stream_on_ = false;
    bool quit_ = false;
    ChannelGen gens_[4] = {ChannelGen(0), ChannelGen(1), ChannelGen(2), ChannelGen(3)};
    std::size_t rr_ = 0U;
    std::vector<std::uint8_t> cur_;
    std::size_t cur_pos_ = 0U;
    std::thread thread_;
};

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
