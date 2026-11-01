// SPDX-License-Identifier: GPL-2.0-only
// A fake OTR-4TS3-U1 behind the FX3 UsbBackend seam, for tests: vendor
// requests (GET_INFO, GPIO, FPGA registers with the D-50 path reset,
// STREAM, I2C_RUN/I2C_RESULT with CXD2856ER/CXD2858ER register models) and
// a bulk stream of tagged packets in the FPGA mux format.
#ifndef OTR_TESTS_FAKE_BOARD_H
#define OTR_TESTS_FAKE_BOARD_H

#include "fx3_device.h"
#include "fx3_protocol.h"
#include "fpga_regs.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace otr::test {

using px4::userland::Error;
using px4::userland::Result;

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
    // Emit only packets of channels enabled in CH_ENABLE (the FPGA discards
    // the others, D-50), at about 16 MB/s; off for the data plane test.
    bool gate_by_ch_enable = false;
    std::size_t i2c_runs = 0U;

    // I2C model behind the TCA9546A: per channel, a CXD2856ER (SLVT 0x64,
    // SLVX 0x66; register 0x00 selects the bank) and a CXD2858ER (0x60).
    // Lock bits are preset: ISDB-T bank 0x60 reg 0x10 bit 0, ISDB-S bank
    // 0xA0 reg 0x12 bit 6 (cxd2856er.cpp is_ts_locked_*).
    struct I2cDevice {
        bool banked = false;
        std::uint8_t bank = 0U;
        std::uint8_t ptr = 0U;
        std::map<std::uint16_t, std::uint8_t> mem;
        std::uint16_t key(std::uint8_t reg) const
        {
            return static_cast<std::uint16_t>(((banked ? bank : 0U) << 8U) | reg);
        }
    };
    std::map<std::pair<std::uint8_t, std::uint8_t>, I2cDevice> i2c;   // (channel, addr)
    std::vector<std::uint8_t> i2c_result;

    void install_receivers()
    {
        for (std::uint8_t ch = 0U; ch < 4U; ++ch) {
            I2cDevice slvt;
            slvt.banked = true;
            slvt.mem[(0x60U << 8U) | 0x10U] = 0x01U;
            slvt.mem[(0xA0U << 8U) | 0x12U] = 0x40U;
            I2cDevice slvx;
            slvx.banked = true;
            i2c[{ch, 0x64U}] = slvt;
            i2c[{ch, 0x66U}] = slvx;
            i2c[{ch, 0x60U}] = I2cDevice{};
        }
    }

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
        if (request == static_cast<std::uint8_t>(fx3::Request::i2c_run)) {
            run_i2c(static_cast<std::uint8_t>(index), data);
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
        if (request == static_cast<std::uint8_t>(fx3::Request::i2c_result)) {
            const std::size_t n = std::min(i2c_result.size(), data.size);
            std::copy(i2c_result.begin(), i2c_result.begin() + static_cast<std::ptrdiff_t>(n), data.data);
            return Result<std::size_t>::success(n);
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
    void run_i2c(std::uint8_t channel, fx3::ByteView program)
    {
        ++i2c_runs;
        std::vector<std::uint8_t> reads;
        std::size_t pos = 0U;
        std::uint8_t op_index = 0U;
        std::uint8_t status = 0U;
        while (pos + 4U <= program.size) {
            const std::uint8_t op = program.data[pos];
            const std::uint8_t addr = program.data[pos + 1U];
            const std::size_t len = program.data[pos + 2U] | (program.data[pos + 3U] << 8U);
            pos += 4U;
            auto it = i2c.find({channel, addr});
            if (it == i2c.end()) {
                status = static_cast<std::uint8_t>(fx3::I2cStatus::nak_address);
                break;
            }
            I2cDevice& dev = it->second;
            if (op == static_cast<std::uint8_t>(fx3::I2cOp::write)) {
                for (std::size_t k = 0U; k < len && pos + k < program.size; ++k) {
                    const std::uint8_t b = program.data[pos + k];
                    if (k == 0U) {
                        dev.ptr = b;
                    } else {
                        if (dev.banked && dev.ptr == 0x00U) dev.bank = b;
                        dev.mem[dev.key(dev.ptr)] = b;
                        ++dev.ptr;
                    }
                }
                pos += len;
            } else {
                for (std::size_t k = 0U; k < len; ++k) {
                    reads.push_back(dev.mem[dev.key(dev.ptr)]);
                    ++dev.ptr;
                }
            }
            ++op_index;
        }
        i2c_result = {status, op_index, static_cast<std::uint8_t>(status ? 0U : reads.size() & 0xFFU),
                      static_cast<std::uint8_t>(status ? 0U : reads.size() >> 8U)};
        if (status == 0U) i2c_result.insert(i2c_result.end(), reads.begin(), reads.end());
    }

    struct Pending {
        std::size_t slot;
        std::uint8_t* buffer;
        std::size_t size;
        bool cancelled;
    };

    // Interleaved tagged stream: channels 0..3 round robin, as the FPGA mux.
    std::uint8_t next_byte()
    {
        while (cur_pos_ == cur_.size()) {
            const std::size_t ch = rr_;
            rr_ = (rr_ + 1U) % 4U;
            // Every channel's source keeps running; a disabled one is dropped.
            auto p = gens_[ch].next();
            if (gate_by_ch_enable && (ch_enable & (1U << ch)) == 0U) continue;
            cur_ = std::move(p);
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
            if (!p.cancelled && gate_by_ch_enable && ch_enable == 0U) continue;
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
            if (gate_by_ch_enable)   // about 16 MB/s, closer to the real board
                std::this_thread::sleep_for(std::chrono::microseconds(len / 16U));
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

}  // namespace otr::test

#endif  // OTR_TESTS_FAKE_BOARD_H
