// SPDX-License-Identifier: GPL-2.0-only
// otr-ts: one receiver of an otrd instance to stdout or a file, with the
// common *-ts CLI (--instance, --receiver, --system, --freq, --tsid/--slot,
// --runtime-dir, --output), so BonDriver_Userland and mirakc can use it.
// Adapted from px4-userland tools/px4_ts.cpp (POSIX).
//
// --device is not offered: dtv-userland-core's identity products do not
// include otr-userland, so a board is selected by --instance, which otrd
// sets to the board serial by default (docs/design.md 7).
#include <dtv/host_ts/px4_ts_core.h>
#include "px4_ts_batched_output.h"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <signal.h>
#include <thread>
#include <unistd.h>

#ifndef OTR_USERLAND_VERSION
#define OTR_USERLAND_VERSION "0.0.0-dev"
#endif

namespace {

using namespace px4::userland;
using namespace px4::userland::cli;

volatile std::sig_atomic_t stop_flag = 0;

void stop_handler(int) noexcept
{
    stop_flag = 1;
}

class PosixClock final : public Px4TsClock {
public:
    std::uint64_t monotonic_ms() noexcept override
    {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    void sleep_ms(std::uint32_t milliseconds) noexcept override
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }
};

class PosixSignal final : public Px4TsSignal {
public:
    bool stop_requested() const noexcept override { return stop_flag != 0; }
};

bool install_signals() noexcept
{
    struct sigaction ignore_pipe {};
    ignore_pipe.sa_handler = SIG_IGN;
    if (sigemptyset(&ignore_pipe.sa_mask) != 0 ||
        ::sigaction(SIGPIPE, &ignore_pipe, nullptr) != 0) return false;
    struct sigaction action {};
    action.sa_handler = stop_handler;
    if (sigemptyset(&action.sa_mask) != 0) return false;
    return ::sigaction(SIGINT, &action, nullptr) == 0 &&
           ::sigaction(SIGTERM, &action, nullptr) == 0;
}

void print_counters(const char* label, const ipc::CountersPayload& counters) noexcept
{
    const auto print = [](const char* name, std::uint64_t value,
                          dtv::userland::wire::MeasureState state) noexcept {
        if (state == dtv::userland::wire::MeasureState::valid)
            std::fprintf(stderr, " %s=%llu", name, static_cast<unsigned long long>(value));
        else if (state == dtv::userland::wire::MeasureState::unknown)
            std::fprintf(stderr, " %s=unknown", name);
    };
    std::fprintf(stderr, "%s", label);
    print("packets", counters.packets, counters.availability[0U]);
    print("bytes", counters.bytes, counters.availability[1U]);
    print("sync-errors", counters.sync_errors, counters.availability[2U]);
    print("tei", counters.tei_packets, counters.availability[3U]);
    print("continuity-errors", counters.continuity_errors, counters.availability[4U]);
    print("queue-drops", counters.queue_drops, counters.availability[5U]);
    print("usb-errors", counters.usb_errors, counters.availability[6U]);
    print("empty-intervals", counters.empty_intervals, counters.availability[7U]);
    std::fputc('\n', stderr);
}

}  // namespace

int main(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--device") == 0 || std::strncmp(argv[i], "--device=", 9) == 0) {
            std::fprintf(stderr, "otr-ts: --device is not supported; use --instance SERIAL\n");
            return 2;
        }
    }
    static const TsClientProfile profile{
        "otr-ts", "otr-userland", dtv::userland::identity::Product::px4,
        false, false, nullptr};
    const Px4TsArguments arguments =
        parse_px4_ts_product_arguments(argc, const_cast<const char* const*>(argv), profile);
    if (!arguments.valid) {
        std::fprintf(stderr, "argument error: %s\n", arguments.error.c_str());
        print_px4_ts_usage(profile, stderr);
        return 2;
    }
    if (arguments.help) {
        print_px4_ts_usage(profile, stdout);
        return 0;
    }
    if (arguments.version) {
        std::printf("otr-ts %s\n", OTR_USERLAND_VERSION);
        return 0;
    }
    if (!install_signals()) {
        std::fprintf(stderr, "signal setup failed\n");
        return 70;
    }
    const bool stdout_output = arguments.output == "-";
    int fd = STDOUT_FILENO;
    if (!stdout_output) {
        fd = ::open(arguments.output.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0666);
        if (fd < 0) {
            std::fprintf(stderr, "output: %s\n", std::strerror(errno));
            return 70;
        }
    }
    Px4TsBatchedFdOutput output(fd, !stdout_output);
    if (!output.ready()) {
        std::fprintf(stderr, "output setup failed\n");
        return 70;
    }
    PosixClock clock;
    PosixSignal signal;
    Px4TsFailureKind failure_kind = Px4TsFailureKind::ipc;
    Px4TsRunDiagnostics diagnostics;
    const auto result = Px4TsRunner::run_wire(arguments, profile, output, clock, signal,
                                              &failure_kind, &diagnostics);
    const auto flushed = output.finish(signal);
    diagnostics.packets_written = output.written_packets();
    if (!result) {
        std::fprintf(stderr, "otr-ts: %s\n", error_string(result.error()));
        print_px4_ts_run_diagnostic(stderr, "otr-ts", result.error(), diagnostics);
        print_px4_ts_written_diagnostic(stderr, diagnostics.packets_written);
        if (diagnostics.stop_counters_valid)
            print_counters("STOP_STREAM", diagnostics.stop_counters);
        if (diagnostics.stream_end_valid) {
            print_counters("STREAM_END", diagnostics.stream_end.counters);
            std::fprintf(stderr, "STREAM_END error=%u\n",
                         static_cast<unsigned int>(diagnostics.stream_end.error_code));
        }
        if (!flushed) {
            std::fprintf(stderr, "output: %s\n", std::strerror(output.system_error()));
            return px4_ts_exit_status(flushed.error(), Px4TsFailureKind::output);
        }
        return px4_ts_exit_status(result.error(), failure_kind);
    }
    if (!flushed) {
        const int exit_status = Px4TsBatchedFdOutput::finish_exit_status(
            flushed.error(), signal.stop_requested());
        if (exit_status != 0)
            std::fprintf(stderr, "output: %s\n", std::strerror(output.system_error()));
        else
            print_counters("stream", result.value().counters);
        return exit_status;
    }
    print_counters("stream", result.value().counters);
    return 0;
}
