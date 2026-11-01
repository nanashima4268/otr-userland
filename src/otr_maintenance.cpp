// SPDX-License-Identifier: GPL-2.0-only
#include "otr_maintenance.h"
#include "fx3_protocol.h"

#include <algorithm>
#include <chrono>
#include <thread>

namespace otr {

namespace {

std::uint32_t le32(const std::vector<std::uint8_t>& b, std::size_t at) noexcept
{
    return static_cast<std::uint32_t>(b[at]) | (static_cast<std::uint32_t>(b[at + 1U]) << 8U) |
           (static_cast<std::uint32_t>(b[at + 2U]) << 16U) |
           (static_cast<std::uint32_t>(b[at + 3U]) << 24U);
}

constexpr std::uint8_t req(fx3::Request r) noexcept { return static_cast<std::uint8_t>(r); }

}  // namespace

Result<Fx3Image> parse_fx3_image(const std::vector<std::uint8_t>& file) noexcept
{
    using R = Result<Fx3Image>;
    if (file.size() < 4U + 8U + 4U || file[0] != 'C' || file[1] != 'Y')
        return R::failure(Error::INVALID_ARGUMENT);
    if (file[3] != 0xB0U || (file[2] & 0x01U) != 0U)   // normal executable image only
        return R::failure(Error::UNSUPPORTED);
    Fx3Image image;
    std::uint32_t checksum = 0U;
    std::size_t at = 4U;
    for (;;) {
        if (at + 8U > file.size()) return R::failure(Error::INVALID_ARGUMENT);
        const std::uint32_t words = le32(file, at);
        const std::uint32_t address = le32(file, at + 4U);
        at += 8U;
        if (words == 0U) {
            image.entry = address;
            break;
        }
        const std::size_t bytes = static_cast<std::size_t>(words) * 4U;
        if (bytes > file.size() - at) return R::failure(Error::INVALID_ARGUMENT);
        Fx3Section section;
        section.address = address;
        section.data.assign(file.begin() + static_cast<std::ptrdiff_t>(at),
                            file.begin() + static_cast<std::ptrdiff_t>(at + bytes));
        for (std::size_t w = 0U; w < bytes; w += 4U) checksum += le32(file, at + w);
        at += bytes;
        image.sections.push_back(std::move(section));
    }
    if (at + 4U > file.size() || le32(file, at) != checksum)
        return R::failure(Error::PROTOCOL_ERROR);
    if (image.sections.empty()) return R::failure(Error::INVALID_ARGUMENT);
    return R::success(std::move(image));
}

Result<void> fx3_boot_ram(fx3::Fx3Control& loader, const Fx3Image& image) noexcept
{
    constexpr std::size_t kChunk = 4096U;
    std::vector<std::uint8_t> back(kChunk);
    for (const Fx3Section& s : image.sections) {
        for (std::size_t off = 0U; off < s.data.size(); off += kChunk) {
            const std::size_t n = std::min(kChunk, s.data.size() - off);
            const std::uint32_t a = s.address + static_cast<std::uint32_t>(off);
            const auto w = loader.control_out(kFx3BootRequest, static_cast<std::uint16_t>(a),
                                              static_cast<std::uint16_t>(a >> 16U),
                                              fx3::ByteView{s.data.data() + off, n});
            if (!w) return w;
            const auto r = loader.control_in(kFx3BootRequest, static_cast<std::uint16_t>(a),
                                             static_cast<std::uint16_t>(a >> 16U),
                                             fx3::MutableByteView{back.data(), n});
            if (!r) return Result<void>::failure(r.error());
            if (r.value() != n || !std::equal(back.begin(), back.begin() + static_cast<std::ptrdiff_t>(n),
                                              s.data.begin() + static_cast<std::ptrdiff_t>(off)))
                return Result<void>::failure(Error::PROTOCOL_ERROR);
        }
    }
    // The device leaves the boot loader here; a failed status stage is normal.
    (void)loader.control_out(kFx3BootRequest, static_cast<std::uint16_t>(image.entry),
                             static_cast<std::uint16_t>(image.entry >> 16U),
                             fx3::ByteView{nullptr, 0U});
    return Result<void>::success();
}

Result<void> flash_write(fx3::Fx3Control& board, std::uint32_t address,
                         const std::vector<std::uint8_t>& data, const Progress& progress) noexcept
{
    if (data.empty() || address + data.size() > fx3::kFlashSize)
        return Result<void>::failure(Error::INVALID_ARGUMENT);
    const std::uint32_t first = address - (address % fx3::kFlashSector);
    const std::uint32_t end = static_cast<std::uint32_t>(address + data.size());
    const std::size_t total = data.size() * 2U;   // write, then verify
    // Data outside [address, end) in the first/last sector would be lost:
    // only sector-aligned writes are accepted (both layout regions are).
    if (first != address) return Result<void>::failure(Error::INVALID_ARGUMENT);

    auto wait_idle = [&board]() -> Result<void> {
        for (int i = 0; i < 1000; ++i) {
            std::uint8_t sr = 0xFFU;
            const auto got = board.control_in(req(fx3::Request::flash_status), 0U, 0U,
                                              fx3::MutableByteView{&sr, 1U});
            if (!got) return Result<void>::failure(got.error());
            if ((sr & fx3::kFlashBusy) == 0U) return Result<void>::success();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        return Result<void>::failure(Error::TIMEOUT);
    };

    for (std::uint32_t s = first; s < end; s += fx3::kFlashSector) {
        const auto e = board.control_out(req(fx3::Request::flash_erase),
                                         static_cast<std::uint16_t>(s),
                                         static_cast<std::uint16_t>(s >> 16U),
                                         fx3::ByteView{nullptr, 0U});
        if (!e) return e;
        const auto idle = wait_idle();
        if (!idle) return idle;
    }
    for (std::size_t off = 0U; off < data.size(); off += fx3::kFlashPage) {
        const std::size_t n = std::min(fx3::kFlashPage, data.size() - off);
        const std::uint32_t a = address + static_cast<std::uint32_t>(off);
        const auto w = board.control_out(req(fx3::Request::flash_write),
                                         static_cast<std::uint16_t>(a),
                                         static_cast<std::uint16_t>(a >> 16U),
                                         fx3::ByteView{data.data() + off, n});
        if (!w) return w;
        if (progress) progress(off + n, total);
    }
    std::vector<std::uint8_t> back(fx3::kFlashReadMax);
    for (std::size_t off = 0U; off < data.size(); off += fx3::kFlashReadMax) {
        const std::size_t n = std::min(fx3::kFlashReadMax, data.size() - off);
        const std::uint32_t a = address + static_cast<std::uint32_t>(off);
        const auto r = board.control_in(req(fx3::Request::flash_read),
                                        static_cast<std::uint16_t>(a),
                                        static_cast<std::uint16_t>(a >> 16U),
                                        fx3::MutableByteView{back.data(), n});
        if (!r) return Result<void>::failure(r.error());
        if (r.value() != n || !std::equal(back.begin(), back.begin() + static_cast<std::ptrdiff_t>(n),
                                          data.begin() + static_cast<std::ptrdiff_t>(off)))
            return Result<void>::failure(Error::PROTOCOL_ERROR);
        if (progress) progress(data.size() + off + n, total);
    }
    return Result<void>::success();
}

std::vector<std::uint8_t> fpga_slot_image(const std::vector<std::uint8_t>& bitstream)
{
    std::vector<std::uint8_t> out(fx3::kFlashFpgaHeader, 0U);
    out[0] = 'O';
    out[1] = 'T';
    out[2] = 'R';
    out[3] = 'B';
    const auto n = static_cast<std::uint32_t>(bitstream.size());
    out[4] = static_cast<std::uint8_t>(n);
    out[5] = static_cast<std::uint8_t>(n >> 8U);
    out[6] = static_cast<std::uint8_t>(n >> 16U);
    out[7] = static_cast<std::uint8_t>(n >> 24U);
    out.insert(out.end(), bitstream.begin(), bitstream.end());
    return out;
}

}  // namespace otr
