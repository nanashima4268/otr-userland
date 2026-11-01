// SPDX-License-Identifier: GPL-2.0-only
// OTR-4TS3-U1 FX3 vendor-request protocol v1 (docs/design.md section 4).
#ifndef OTR_FX3_PROTOCOL_H
#define OTR_FX3_PROTOCOL_H

#include <cstddef>
#include <cstdint>

namespace otr::fx3 {

inline constexpr std::uint16_t kProtocolVersion = 1U;

// bRequest values. 0xA0 is avoided: the FX3 boot loader uses it.
enum class Request : std::uint8_t {
    get_info = 0xB0U,
    i2c_run = 0xB1U,
    i2c_result = 0xB2U,
    gpio = 0xB3U,
    fpga_load = 0xB4U,
    fpga_reg = 0xB5U,
    stream = 0xB6U,
    // SPI boot flash (W25Q32JV, otr-4ts3-u1 D-49). Address = wIndex:wValue (24 bits).
    flash_write = 0xB7U,    // page program, <= 256 bytes within one page
    flash_read = 0xB8U,     // wLength <= 4096
    flash_erase = 0xB9U,    // 4 KiB sector, no data stage; poll flash_status until not busy
    flash_status = 0xBAU,   // IN 1 byte: status register 1, bit 0 = busy
    ms_os_20 = 0xBFU,       // MS OS 2.0 descriptor set (Windows only, wIndex = 7)
};

// SPI boot flash layout (D-49).
inline constexpr std::uint32_t kFlashSize = 0x400000U;
inline constexpr std::size_t kFlashPage = 256U;
inline constexpr std::uint32_t kFlashSector = 4096U;
inline constexpr std::size_t kFlashReadMax = 4096U;
inline constexpr std::uint32_t kFlashFx3Image = 0x000000U;   // elf2img output
inline constexpr std::uint32_t kFlashFpgaSlot = 0x100000U;   // {"OTRB", u32 length LE}, bitstream at +256
inline constexpr std::uint32_t kFlashFpgaHeader = 256U;
inline constexpr std::uint8_t kFlashBusy = 0x01U;

// GET_INFO (32 bytes): "OTR1", protocol u16, fw minor, fw major,
// [8] FPGA configured, [9] stream configured, [10] FPGA source
// (0 none, 1 flash, 2 host), [11] flash present, [12..15] DMA buffer size,
// [16..17] I2C max program, [18..19] I2C max read, [20..27] flash unique ID.
inline constexpr std::size_t kInfoLength = 32U;
enum class FpgaSource : std::uint8_t { none = 0U, flash = 1U, host = 2U };

// I2C program (I2C_RUN data stage): a sequence of
//   write: {0x01, addr7, len_lo, len_hi, data[len]}
//   read:  {0x02, addr7, len_lo, len_hi}
// executed in order with repeated starts, on the mux channel in wIndex.
enum class I2cOp : std::uint8_t {
    write = 0x01U,
    read = 0x02U,
};
inline constexpr std::size_t kI2cOpHeader = 4U;
inline constexpr std::size_t kI2cMaxProgram = 512U;
inline constexpr std::size_t kI2cMaxRead = 512U;
inline constexpr std::uint8_t kI2cChannels = 4U;

// I2C_RESULT data stage: {status, failed_op_index, nread_lo, nread_hi, data[nread]}
enum class I2cStatus : std::uint8_t {
    ok = 0x00U,
    nak_address = 0x01U,
    nak_data = 0x02U,
    bus_error = 0x03U,
    bad_program = 0x04U,
};
inline constexpr std::size_t kI2cResultHeader = 4U;

}  // namespace otr::fx3

#endif  // OTR_FX3_PROTOCOL_H
