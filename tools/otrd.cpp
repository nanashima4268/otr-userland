// SPDX-License-Identifier: GPL-2.0-only
// otrd: owns one OTR-4TS3-U1 and serves its four receivers over the
// dtv-userland-core control/data endpoints, like px4d (docs/design.md 1).
//
// The composition is in src/otrd_service.cpp; this file opens the board
// with libusb and handles arguments and signals.
// POSIX only for now; Windows follows once the board runs (D-10).
#include "libusb_backend.h"
#include "otrd_service.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>

#ifndef OTR_USERLAND_VERSION
#define OTR_USERLAND_VERSION "0.0.0-dev"
#endif
#ifndef DTV_USERLAND_CORE_VERSION
#define DTV_USERLAND_CORE_VERSION "unknown"
#endif

namespace {

using px4::userland::Error;
using px4::userland::Result;

volatile std::sig_atomic_t g_stop = 0;
std::atomic<bool> g_stop_atomic{false};
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

int fail(const char* what, Error e)
{
    std::fprintf(stderr, "otrd: %s: %s\n", what, px4::userland::error_string(e));
    return 69;
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
    otr::OtrdOptions options;
    options.runtime_dir = args.runtime_dir;
    options.instance = args.instance;
    options.fpga = args.fpga;
    options.group = args.group;
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::thread watch([] {
        while (!g_stop_atomic.load()) {
            if (g_stop) g_stop_atomic.store(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });
    const int rc = otr::otrd_serve(options, *usb.value(), g_stop_atomic);
    g_stop_atomic.store(true);
    watch.join();
    return rc;
}
