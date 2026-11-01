// SPDX-License-Identifier: GPL-2.0-only
// Board maintenance for otrctl (D-46, D-49): FX3 boot images, the SPI boot
// flash (FX3 image at 0, FPGA slot at 0x100000), and the FX3 ROM USB boot
// loader (vendor request 0xA0). Written from the FX3 boot image format in the
// FX3 Programmer's Manual (bImageCTL/bImageType, sections in 32-bit words,
// terminating entry record, checksum of all section words).
#ifndef OTR_MAINTENANCE_H
#define OTR_MAINTENANCE_H

#include "fx3_control.h"

#include <cstdint>
#include <functional>
#include <vector>

namespace otr {

using px4::userland::Error;
using px4::userland::Result;

struct Fx3Section final {
    std::uint32_t address = 0U;
    std::vector<std::uint8_t> data;
};

struct Fx3Image final {
    std::vector<Fx3Section> sections;
    std::uint32_t entry = 0U;
};

// Parses and checks an elf2img image ("CY", type 0xB0, executable, checksum).
Result<Fx3Image> parse_fx3_image(const std::vector<std::uint8_t>& file) noexcept;

// Cypress ROM boot loader: 04b4:00f3, request 0xA0 (RW_INTERNAL).
inline constexpr std::uint16_t kFx3BootVid = 0x04B4U;
inline constexpr std::uint16_t kFx3BootPid = 0x00F3U;
inline constexpr std::uint8_t kFx3BootRequest = 0xA0U;

// Writes every section to RAM in <= 4 KiB chunks, reads each back, then
// jumps to the entry point. The device re-enumerates with the new firmware.
Result<void> fx3_boot_ram(fx3::Fx3Control& loader, const Fx3Image& image) noexcept;

using Progress = std::function<void(std::size_t done, std::size_t total)>;

// Erases the sectors covering [address, address + size), programs pages and
// verifies by reading back (FLASH_WRITE cannot report errors, design.md 4).
Result<void> flash_write(fx3::Fx3Control& board, std::uint32_t address,
                         const std::vector<std::uint8_t>& data,
                         const Progress& progress = nullptr) noexcept;

// FPGA slot payload: 256-byte header {"OTRB", u32 length LE, zero fill},
// followed by the bitstream.
std::vector<std::uint8_t> fpga_slot_image(const std::vector<std::uint8_t>& bitstream);

}  // namespace otr

#endif  // OTR_MAINTENANCE_H
