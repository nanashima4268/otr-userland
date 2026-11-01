// SPDX-License-Identifier: GPL-2.0-only
// otrctl: control client for otrd, plus board maintenance.
//
//   otrctl [--runtime-dir DIR] [--instance NAME] list|status ...
//       the common dtv-userland-core control CLI against a running otrd
//   otrctl board [--device SERIAL] info|stats|flash-fx3 FILE|flash-fpga FILE
//       direct USB access; otrd must not be running (it owns the device)
//   otrctl board boot FILE
//       loads an FX3 image into RAM through the ROM USB boot loader (04b4:00f3),
//       for a board with an empty flash (D-49)
//
// --device is not passed to the common CLI: dtv-userland-core's identity
// products do not include otr-userland (docs/design.md 7).
#include "fx3_device.h"
#include "fx3_protocol.h"
#include "host_ctl.h"
#include "libusb_backend.h"
#include "otr_board.h"
#include "otr_maintenance.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#ifndef OTR_USERLAND_VERSION
#define OTR_USERLAND_VERSION "0.0.0-dev"
#endif

namespace {

using otr::Error;
using otr::Result;

int fail(const char* what, Error e)
{
    std::fprintf(stderr, "otrctl: %s: %s\n", what, px4::userland::error_string(e));
    if (e == Error::BUSY) std::fprintf(stderr, "otrctl: is otrd running? Stop it first.\n");
    return e == Error::INVALID_ARGUMENT ? 2 : 69;
}

bool read_file(const char* path, std::vector<std::uint8_t>& out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    return true;
}

void progress(std::size_t done, std::size_t total)
{
    std::fprintf(stderr, "\r  %3zu%%", total == 0U ? 100U : done * 100U / total);
    if (done == total) std::fputc('\n', stderr);
}

void board_usage()
{
    std::fprintf(stderr,
                 "usage: otrctl board [--device SERIAL] info | stats | flash-fx3 FILE.img |"
                 " flash-fpga FILE.bin\n"
                 "       otrctl board boot FILE.img\n");
}

int run_board(int argc, char** argv)
{
    int i = 2;
    otr::fx3::UsbIds ids;
    if (i + 1 < argc && std::strcmp(argv[i], "--device") == 0) {
        ids.serial = argv[i + 1];
        i += 2;
    }
    if (i >= argc) {
        board_usage();
        return 2;
    }
    const std::string cmd = argv[i];
    const char* file = i + 1 < argc ? argv[i + 1] : nullptr;

    if (cmd == "boot") {
        std::vector<std::uint8_t> bytes;
        if (file == nullptr || !read_file(file, bytes)) return fail("read image", Error::NOT_FOUND);
        const auto image = otr::parse_fx3_image(bytes);
        if (!image) return fail("FX3 image", image.error());
        otr::fx3::UsbIds boot{otr::kFx3BootVid, otr::kFx3BootPid, {}};
        auto usb = otr::fx3::LibusbBackend::open(boot);
        if (!usb) return fail("FX3 boot loader (04b4:00f3)", usb.error());
        otr::fx3::Fx3Device loader(*usb.value());
        const auto loaded = otr::fx3_boot_ram(loader, image.value());
        if (!loaded) return fail("RAM load", loaded.error());
        std::fprintf(stderr, "otrctl: firmware started (entry 0x%08x); the board re-enumerates\n",
                     image.value().entry);
        return 0;
    }

    auto usb = otr::fx3::LibusbBackend::open(ids);
    if (!usb) return fail(usb.error() == Error::BUSY ? "more than one board; use --device"
                                                     : "open board", usb.error());
    otr::fx3::Fx3Device device(*usb.value());
    otr::OtrBoard board(device);

    if (cmd == "info") {
        const auto info = board.info();
        if (!info) return fail("GET_INFO", info.error());
        const auto& v = info.value();
        std::printf("serial      %s\n", usb.value()->serial().c_str());
        std::printf("firmware    %u.%u (protocol %u)\n", v.fw_major, v.fw_minor, v.protocol);
        std::printf("fpga        %s\n", !v.fpga_configured ? "not configured"
                                       : v.fpga_source == otr::fx3::FpgaSource::flash ? "from flash"
                                                                                       : "from host");
        std::printf("flash       %s\n", v.flash_present ? "present" : "not found");
        std::printf("dma buffer  %u bytes\n", v.dma_buffer_size);
        if (v.fpga_configured) {
            const auto version = board.fpga_read(otr::fpga::Reg::version);
            if (version)
                std::printf("fpga ver    %u.%u\n", version.value() >> 16U, version.value() & 0xFFFFU);
        }
        return 0;
    }
    if (cmd == "stats") {
        const auto checked = board.check_fpga();
        if (!checked) return fail("FPGA", checked.error());
        for (unsigned ch = 0U; ch < 4U; ++ch) {
            const auto drops = board.fpga_read(otr::fpga::drop_cnt(ch));
            const auto pkts = board.fpga_read(otr::fpga::pkt_cnt(ch));
            if (!drops || !pkts) return fail("FPGA counters", drops ? pkts.error() : drops.error());
            std::printf("ch%u packets %u drops %u\n", ch, pkts.value(), drops.value());
        }
        const auto en = board.fpga_read(otr::fpga::Reg::ch_enable);
        if (en) std::printf("ch_enable   0x%x\n", en.value());
        return 0;
    }
    if (cmd == "flash-fx3" || cmd == "flash-fpga") {
        std::vector<std::uint8_t> bytes;
        if (file == nullptr || !read_file(file, bytes)) return fail("read file", Error::NOT_FOUND);
        std::uint32_t address = otr::fx3::kFlashFx3Image;
        if (cmd == "flash-fx3") {
            const auto image = otr::parse_fx3_image(bytes);   // refuse anything else at 0
            if (!image) return fail("FX3 image", image.error());
            if (bytes.size() > otr::fx3::kFlashFpgaSlot) return fail("FX3 image size", Error::INVALID_ARGUMENT);
        } else {
            if (bytes.empty() || bytes.size() > 160U * 1024U)
                return fail("bitstream size", Error::INVALID_ARGUMENT);
            bytes = otr::fpga_slot_image(bytes);
            address = otr::fx3::kFlashFpgaSlot;
        }
        std::fprintf(stderr, "otrctl: writing %zu bytes at 0x%06x\n", bytes.size(), address);
        const auto written = otr::flash_write(device, address, bytes, progress);
        if (!written) return fail("flash write", written.error());
        std::fprintf(stderr, "otrctl: written and verified; power-cycle the board to boot from it\n");
        return 0;
    }
    board_usage();
    return 2;
}

const dtv::userland::host_ctl::Profile& profile() noexcept
{
    static const dtv::userland::host_ctl::Profile value{
        "otrctl", "otr-userland", OTR_USERLAND_VERSION,
        dtv::userland::identity::Product::px4, false, false, nullptr, nullptr, false, false};
    return value;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc >= 2 && std::strcmp(argv[1], "board") == 0) return run_board(argc, argv);
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--device") == 0 || std::strcmp(argv[i], "-d") == 0) {
            std::fprintf(stderr, "otrctl: --device is not supported here; use --instance SERIAL\n");
            return 2;
        }
    }
    std::vector<const char*> args(argv, argv + argc);
    const int rc = dtv::userland::host_ctl::run_control_cli(argc, args.data(), profile());
    if (argc == 2 && (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0)) {
        std::printf("board maintenance (direct USB, otrd stopped):\n");
        std::printf("  otrctl board [--device SERIAL] info | stats | flash-fx3 FILE.img |"
                    " flash-fpga FILE.bin\n  otrctl board boot FILE.img   (ROM USB boot loader)\n");
        std::printf("note: select a running otrd with --instance (its board serial); "
                    "--device is not supported\n");
    }
    return rc;
}
