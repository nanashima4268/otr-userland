// SPDX-License-Identifier: GPL-2.0-only
// Offline tests for Fx3I2cMaster: the I2C program encoding, the result decode,
// and the reused px4-userland CXD2856ER driver running on top of it.
#include "cxd2856er.h"
#include "fx3_i2c.h"

#include <array>
#include <cstdio>
#include <cstring>
#include <map>
#include <mutex>
#include <utility>
#include <vector>

using namespace otr::fx3;
using px4::userland::BridgeI2cRequest;
using px4::userland::BridgeI2cRequestType;
using px4::userland::Error;

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

// Register-pointer I2C device: a write sets the pointer from its first byte
// and stores the rest sequentially; a read returns bytes from the pointer.
struct RegDevice {
    std::array<std::uint8_t, 256> regs{};
    std::uint8_t ptr = 0U;
};

// Fake FX3 firmware: executes I2C programs against per-channel devices.
class FakeFx3 final : public Fx3Control {
public:
    std::map<std::pair<std::uint8_t, std::uint8_t>, RegDevice> devices;  // (channel, addr)
    std::vector<std::uint8_t> last_program;
    std::uint16_t last_channel = 0xFFFFU;
    std::vector<std::uint8_t> result;

    Result<void> control_out(std::uint8_t request, std::uint16_t, std::uint16_t index,
                             ByteView data) noexcept override
    {
        if (request != static_cast<std::uint8_t>(Request::i2c_run))
            return Result<void>::failure(Error::UNSUPPORTED);
        last_program.assign(data.data, data.data + data.size);
        last_channel = index;
        run(static_cast<std::uint8_t>(index));
        return Result<void>::success();
    }

    Result<std::size_t> control_in(std::uint8_t request, std::uint16_t, std::uint16_t,
                                   MutableByteView data) noexcept override
    {
        if (request != static_cast<std::uint8_t>(Request::i2c_result))
            return Result<std::size_t>::failure(Error::UNSUPPORTED);
        const std::size_t n = result.size() < data.size ? result.size() : data.size;
        std::memcpy(data.data, result.data(), n);
        return Result<std::size_t>::success(n);
    }

private:
    void run(std::uint8_t channel)
    {
        std::vector<std::uint8_t> reads;
        std::size_t pos = 0U;
        std::uint8_t op_index = 0U;
        I2cStatus status = I2cStatus::ok;
        while (pos < last_program.size()) {
            if (pos + kI2cOpHeader > last_program.size()) {
                status = I2cStatus::bad_program;
                break;
            }
            const auto op = static_cast<I2cOp>(last_program[pos]);
            const std::uint8_t addr = last_program[pos + 1U];
            const std::size_t len = last_program[pos + 2U] | (last_program[pos + 3U] << 8U);
            pos += kI2cOpHeader;
            auto it = devices.find({channel, addr});
            if (it == devices.end()) {
                status = I2cStatus::nak_address;
                break;
            }
            RegDevice& dev = it->second;
            if (op == I2cOp::write) {
                for (std::size_t k = 0U; k < len; ++k) {
                    const std::uint8_t b = last_program[pos + k];
                    if (k == 0U)
                        dev.ptr = b;
                    else
                        dev.regs[dev.ptr++] = b;
                }
                pos += len;
            } else {
                for (std::size_t k = 0U; k < len; ++k)
                    reads.push_back(dev.regs[dev.ptr++]);
            }
            ++op_index;
        }
        result = {static_cast<std::uint8_t>(status), op_index,
                  static_cast<std::uint8_t>(reads.size() & 0xFFU),
                  static_cast<std::uint8_t>(reads.size() >> 8U)};
        if (status == I2cStatus::ok)
            result.insert(result.end(), reads.begin(), reads.end());
        else
            result[2] = result[3] = 0U;
    }
};

struct NoDelay final : px4::userland::Cxd285xDelay {
    void sleep_ms(std::uint32_t) noexcept override {}
};

void test_encoding()
{
    FakeFx3 fx3;
    fx3.devices[{2U, 0x64U}] = RegDevice{};
    std::mutex lock;
    Fx3I2cMaster bus(fx3, lock, 2U);

    const std::uint8_t w1[] = {0x00U, 0x10U};
    const std::uint8_t w2[] = {0x05U};
    std::uint8_t r[3] = {};
    BridgeI2cRequest reqs[] = {
        {BridgeI2cRequestType::write, 0x64U, {w1, sizeof(w1)}, {nullptr, 0U}},
        {BridgeI2cRequestType::write, 0x64U, {w2, sizeof(w2)}, {nullptr, 0U}},
        {BridgeI2cRequestType::read, 0x64U, {nullptr, 0U}, {r, sizeof(r)}},
    };
    CHECK(bus.request(reqs, 3U));
    const std::vector<std::uint8_t> expect = {
        0x01, 0x64, 0x02, 0x00, 0x00, 0x10,  // write 2 bytes
        0x01, 0x64, 0x01, 0x00, 0x05,        // write 1 byte
        0x02, 0x64, 0x03, 0x00,              // read 3 bytes
    };
    CHECK(fx3.last_program == expect);
    CHECK(fx3.last_channel == 2U);
}

void test_cxd2856er_roundtrip()
{
    FakeFx3 fx3;
    fx3.devices[{1U, 0x64U}] = RegDevice{};  // SLVT
    fx3.devices[{1U, 0x66U}] = RegDevice{};  // SLVX = SLVT + 2
    std::mutex lock;
    Fx3I2cMaster bus(fx3, lock, 1U);
    NoDelay delay;
    px4::userland::Cxd2856er demod(bus, 0x64U, delay);

    const std::uint8_t values[] = {0x11U, 0x22U, 0x33U};
    CHECK(demod.write_regs(px4::userland::Cxd2856erTarget::slvt, 0x40U,
                           ByteView{values, sizeof(values)}));
    std::uint8_t back[3] = {};
    CHECK(demod.read_regs(px4::userland::Cxd2856erTarget::slvt, 0x40U,
                          MutableByteView{back, sizeof(back)}));
    CHECK(std::memcmp(back, values, sizeof(values)) == 0);

    // write_reg_mask reads, merges and writes back through the same path.
    CHECK(demod.write_reg_mask(px4::userland::Cxd2856erTarget::slvt, 0x40U, 0x0FU, 0x0FU));
    std::uint8_t merged = 0U;
    CHECK(demod.read_regs(px4::userland::Cxd2856erTarget::slvt, 0x40U,
                          MutableByteView{&merged, 1U}));
    CHECK(merged == 0x1FU);
}

void test_channel_isolation()
{
    FakeFx3 fx3;
    fx3.devices[{0U, 0x64U}] = RegDevice{};
    fx3.devices[{3U, 0x64U}] = RegDevice{};
    std::mutex lock;
    Fx3I2cMaster bus0(fx3, lock, 0U);
    Fx3I2cMaster bus3(fx3, lock, 3U);
    const std::uint8_t w[] = {0x10U, 0xAAU};
    BridgeI2cRequest req{BridgeI2cRequestType::write, 0x64U, {w, sizeof(w)}, {nullptr, 0U}};
    CHECK(bus0.request(&req, 1U));
    CHECK((fx3.devices[{0U, 0x64U}].regs[0x10] == 0xAAU));
    CHECK((fx3.devices[{3U, 0x64U}].regs[0x10] == 0x00U));
    (void)bus3;
}

void test_nak()
{
    FakeFx3 fx3;  // no devices: every address NAKs
    std::mutex lock;
    Fx3I2cMaster bus(fx3, lock, 0U);
    const std::uint8_t w[] = {0x00U};
    BridgeI2cRequest req{BridgeI2cRequestType::write, 0x64U, {w, sizeof(w)}, {nullptr, 0U}};
    Result<void> r = bus.request(&req, 1U);
    CHECK(!r);
    CHECK(r.error() == Error::USB_IO);
    CHECK(bus.last_status() == I2cStatus::nak_address);
}

void test_limits()
{
    FakeFx3 fx3;
    fx3.devices[{0U, 0x64U}] = RegDevice{};
    std::mutex lock;
    Fx3I2cMaster bus(fx3, lock, 0U);
    std::vector<std::uint8_t> big(kI2cMaxProgram, 0U);
    BridgeI2cRequest w{BridgeI2cRequestType::write, 0x64U, {big.data(), big.size()}, {nullptr, 0U}};
    CHECK(bus.request(&w, 1U).error() == Error::BUFFER_TOO_SMALL);
    std::vector<std::uint8_t> rb(kI2cMaxRead + 1U, 0U);
    BridgeI2cRequest r{BridgeI2cRequestType::read, 0x64U, {nullptr, 0U}, {rb.data(), rb.size()}};
    CHECK(bus.request(&r, 1U).error() == Error::BUFFER_TOO_SMALL);
    Fx3I2cMaster bad(fx3, lock, kI2cChannels);
    const std::uint8_t one[] = {0x00U};
    BridgeI2cRequest ok{BridgeI2cRequestType::write, 0x64U, {one, 1U}, {nullptr, 0U}};
    CHECK(bad.request(&ok, 1U).error() == Error::INVALID_ARGUMENT);
}

}  // namespace

int main()
{
    test_encoding();
    test_cxd2856er_roundtrip();
    test_channel_isolation();
    test_nak();
    test_limits();
    if (g_failures == 0)
        std::printf("otr_backend_tests: all passed\n");
    return g_failures == 0 ? 0 : 1;
}
