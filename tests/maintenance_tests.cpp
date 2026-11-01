// SPDX-License-Identifier: GPL-2.0-only
// otrctl maintenance without hardware: FX3 image parsing, the ROM boot
// loader download, and the flash writer against a W25Q32JV model behind
// the FX3 flash requests (erase -> 0xFF, program ANDs bits, busy status).
// Optionally parses a real elf2img image: OTR_FX3_IMG=path/otr_fx3.img.
#include "fx3_protocol.h"
#include "otr_maintenance.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <map>
#include <vector>

using namespace otr;

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

void put32(std::vector<std::uint8_t>& b, std::uint32_t v)
{
    for (int i = 0; i < 4; ++i) b.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
}

// elf2img layout: "CY", ctl, 0xB0, {words, addr, data}..., {0, entry}, checksum
std::vector<std::uint8_t> make_image(const std::vector<std::pair<std::uint32_t, std::vector<std::uint32_t>>>& sections,
                                     std::uint32_t entry, bool corrupt = false)
{
    std::vector<std::uint8_t> b = {'C', 'Y', 0x1C, 0xB0};
    std::uint32_t sum = 0U;
    for (const auto& s : sections) {
        put32(b, static_cast<std::uint32_t>(s.second.size()));
        put32(b, s.first);
        for (std::uint32_t w : s.second) {
            put32(b, w);
            sum += w;
        }
    }
    put32(b, 0U);
    put32(b, entry);
    put32(b, corrupt ? sum + 1U : sum);
    return b;
}

class FakeLoader final : public fx3::Fx3Control {
public:
    std::map<std::uint32_t, std::uint8_t> ram;
    std::uint32_t jumped = 0U;
    bool jumped_set = false;
    Result<void> control_out(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                             fx3::ByteView data) noexcept override
    {
        if (request != kFx3BootRequest) return Result<void>::failure(Error::UNSUPPORTED);
        const std::uint32_t a = value | (static_cast<std::uint32_t>(index) << 16U);
        if (data.size == 0U) {
            jumped = a;
            jumped_set = true;
        }
        for (std::size_t i = 0; i < data.size; ++i) ram[a + static_cast<std::uint32_t>(i)] = data.data[i];
        return Result<void>::success();
    }
    Result<std::size_t> control_in(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                                   fx3::MutableByteView data) noexcept override
    {
        if (request != kFx3BootRequest) return Result<std::size_t>::failure(Error::UNSUPPORTED);
        const std::uint32_t a = value | (static_cast<std::uint32_t>(index) << 16U);
        for (std::size_t i = 0; i < data.size; ++i) data.data[i] = ram[a + static_cast<std::uint32_t>(i)];
        return Result<std::size_t>::success(data.size);
    }
};

class FakeFlash final : public fx3::Fx3Control {
public:
    std::vector<std::uint8_t> mem = std::vector<std::uint8_t>(fx3::kFlashSize, 0x00U);
    int busy_polls = 0;
    int erases = 0;
    bool fail_bit = false;   // a stuck bit: program cannot clear bit 0 at 0x100010
    Result<void> control_out(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                             fx3::ByteView data) noexcept override
    {
        const std::uint32_t a = value | (static_cast<std::uint32_t>(index) << 16U);
        if (request == static_cast<std::uint8_t>(fx3::Request::flash_erase)) {
            if (a % fx3::kFlashSector != 0U || busy_polls > 0) return Result<void>::failure(Error::BUSY);
            std::fill(mem.begin() + a, mem.begin() + a + fx3::kFlashSector, 0xFFU);
            busy_polls = 3;
            ++erases;
            return Result<void>::success();
        }
        if (request == static_cast<std::uint8_t>(fx3::Request::flash_write)) {
            if (data.size > fx3::kFlashPage || (a % fx3::kFlashPage) + data.size > fx3::kFlashPage)
                return Result<void>::failure(Error::INVALID_ARGUMENT);
            for (std::size_t i = 0; i < data.size; ++i) {
                std::uint8_t v = data.data[i];
                if (fail_bit && a + i == 0x100010U) v |= 0x01U;
                mem[a + i] &= v;
            }
            return Result<void>::success();
        }
        return Result<void>::failure(Error::UNSUPPORTED);
    }
    Result<std::size_t> control_in(std::uint8_t request, std::uint16_t value, std::uint16_t index,
                                   fx3::MutableByteView data) noexcept override
    {
        const std::uint32_t a = value | (static_cast<std::uint32_t>(index) << 16U);
        if (request == static_cast<std::uint8_t>(fx3::Request::flash_status)) {
            data.data[0] = busy_polls > 0 ? fx3::kFlashBusy : 0U;
            if (busy_polls > 0) --busy_polls;
            return Result<std::size_t>::success(1U);
        }
        if (request == static_cast<std::uint8_t>(fx3::Request::flash_read)) {
            if (data.size > fx3::kFlashReadMax) return Result<std::size_t>::failure(Error::INVALID_ARGUMENT);
            std::copy(mem.begin() + a, mem.begin() + a + data.size, data.data);
            return Result<std::size_t>::success(data.size);
        }
        return Result<std::size_t>::failure(Error::UNSUPPORTED);
    }
};

void test_image()
{
    std::vector<std::uint32_t> big(3000);
    for (std::size_t i = 0; i < big.size(); ++i) big[i] = static_cast<std::uint32_t>(i * 2654435761U);
    const auto file = make_image({{0x40003000U, big}, {0x10000U, {0xDEADBEEFU, 1U}}}, 0x40003100U);
    const auto image = parse_fx3_image(file);
    CHECK(image);
    if (image) {
        CHECK(image.value().sections.size() == 2U);
        CHECK(image.value().sections[0].data.size() == 12000U);
        CHECK(image.value().entry == 0x40003100U);
    }
    CHECK(parse_fx3_image(make_image({{0x10000U, {1U}}}, 0U, true)).error() == Error::PROTOCOL_ERROR);
    auto bad = file;
    bad[0] = 'X';
    CHECK(!parse_fx3_image(bad));
    bad = file;
    bad.resize(40U);
    CHECK(!parse_fx3_image(bad));

    FakeLoader loader;
    CHECK(fx3_boot_ram(loader, image.value()));
    CHECK(loader.jumped_set && loader.jumped == 0x40003100U);
    bool same = true;
    for (std::size_t i = 0; i < 12000U; ++i)
        same = same && loader.ram[0x40003000U + static_cast<std::uint32_t>(i)] ==
                           image.value().sections[0].data[i];
    CHECK(same);

    if (const char* path = std::getenv("OTR_FX3_IMG")) {
        std::ifstream f(path, std::ios::binary);
        const std::vector<std::uint8_t> real((std::istreambuf_iterator<char>(f)),
                                             std::istreambuf_iterator<char>());
        const auto parsed = parse_fx3_image(real);
        CHECK(parsed);
        if (parsed)
            std::fprintf(stderr, "real image: %zu sections, entry 0x%08x\n",
                         parsed.value().sections.size(), parsed.value().entry);
    }
}

void test_flash()
{
    FakeFlash flash;
    std::vector<std::uint8_t> bits(135100);
    for (std::size_t i = 0; i < bits.size(); ++i) bits[i] = static_cast<std::uint8_t>(i * 7U + 3U);
    const auto slot = fpga_slot_image(bits);
    CHECK(slot.size() == 256U + bits.size());
    CHECK(slot[0] == 'O' && slot[3] == 'B' && slot[4] == 0xBCU && slot[5] == 0x0FU && slot[6] == 0x02U);
    std::size_t last = 0U;
    CHECK(flash_write(flash, fx3::kFlashFpgaSlot, slot,
                      [&last](std::size_t done, std::size_t total) { last = done * 100U / total; }));
    CHECK(last == 100U);
    CHECK(std::equal(slot.begin(), slot.end(), flash.mem.begin() + fx3::kFlashFpgaSlot));
    CHECK(flash.erases == static_cast<int>((slot.size() + 4095U) / 4096U));
    CHECK(flash.mem[fx3::kFlashFpgaSlot - 1U] == 0x00U);   // the FX3 image region is untouched

    CHECK(flash_write(flash, 0x100100U, slot).error() == Error::INVALID_ARGUMENT);   // not sector-aligned
    CHECK(flash_write(flash, fx3::kFlashSize - 16U, slot).error() == Error::INVALID_ARGUMENT);

    FakeFlash stuck;
    stuck.fail_bit = true;
    std::vector<std::uint8_t> zeros(64, 0x00U);
    CHECK(flash_write(stuck, fx3::kFlashFpgaSlot, zeros).error() == Error::PROTOCOL_ERROR);
}

}  // namespace

int main()
{
    test_image();
    test_flash();
    if (g_failures == 0) std::printf("maintenance_tests: all passed\n");
    return g_failures == 0 ? 0 : 1;
}
