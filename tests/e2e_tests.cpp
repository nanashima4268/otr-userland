// SPDX-License-Identifier: GPL-2.0-only
// End to end without hardware: the otrd service on a fake board (tests/
// fake_board.h: FX3 requests, CXD2856ER/CXD2858ER register models, tagged
// TS gated by CH_ENABLE) and the real otr-ts / otrctl executables as client
// processes over the dtv-userland-core endpoints.
//   e2e_tests <path/to/otr-ts> <path/to/otrctl>
#include "fake_board.h"
#include "otrd_service.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iterator>
#include <spawn.h>
#include <string>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

extern char** environ;

using namespace otr;
using namespace otr::test;

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

int run(const std::vector<std::string>& args, const std::string& stdout_path = {})
{
    std::vector<char*> argv;
    for (const auto& a : args) argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);
    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    if (!stdout_path.empty())
        posix_spawn_file_actions_addopen(&actions, 1, stdout_path.c_str(),
                                         O_WRONLY | O_CREAT | O_TRUNC, 0600);
    pid_t pid = 0;
    const int rc = posix_spawn(&pid, argv[0], &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    if (rc != 0) return -1;
    int status = 0;
    if (waitpid(pid, &status, 0) != pid) return -1;
    return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}

std::vector<std::uint8_t> slurp(const std::string& path)
{
    std::ifstream f(path, std::ios::binary);
    return std::vector<std::uint8_t>((std::istreambuf_iterator<char>(f)),
                                     std::istreambuf_iterator<char>());
}

// The output must be channel `ch`'s packets (sync restored), consecutive.
std::size_t check_channel(const std::vector<std::uint8_t>& ts, std::size_t ch)
{
    CHECK(!ts.empty());
    CHECK(ts.size() % kPkt == 0U);
    if (ts.size() < kPkt) return 0U;
    ChannelGen ref(ch);
    std::vector<std::uint8_t> p;
    bool found = false;
    for (std::size_t i = 0; i < 20000000U; ++i) {
        p = ref.next();
        p[0] = 0x47U;
        if (std::equal(p.begin(), p.end(), ts.begin())) {
            found = true;
            break;
        }
    }
    CHECK(found);
    std::size_t bad = 0U;
    for (std::size_t k = 1; found && (k + 1U) * kPkt <= ts.size(); ++k) {
        p = ref.next();
        p[0] = 0x47U;
        if (!std::equal(p.begin(), p.end(), ts.begin() + static_cast<std::ptrdiff_t>(k * kPkt))) ++bad;
    }
    CHECK(bad == 0U);
    return ts.size() / kPkt;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::fprintf(stderr, "usage: e2e_tests OTR_TS OTRCTL\n");
        return 2;
    }
    const std::string otr_ts = argv[1], otrctl = argv[2];
    char tmpl[] = "/tmp/otr-e2e-XXXXXX";
    const char* dir_c = mkdtemp(tmpl);
    if (dir_c == nullptr) return 2;
    const std::string dir = dir_c;
    chmod(dir.c_str(), 0700);

    FakeBoard fake;
    fake.gate_by_ch_enable = true;
    fake.install_receivers();

    std::atomic<bool> stop{false};
    std::atomic<bool> ready{false};
    std::string instance;
    int service_rc = -1;
    OtrdOptions options;
    options.runtime_dir = dir;
    std::thread service([&] {
        service_rc = otrd_serve(options, fake, stop, [&](const std::string& inst) {
            instance = inst;
            ready.store(true);
        });
        ready.store(true);
    });
    for (int i = 0; i < 200 && !ready.load(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK(ready.load() && !instance.empty());
    CHECK(instance == "a0a1a2a3a4a5a6a7");   // the fake flash unique ID

    if (!instance.empty()) {
        // control client: the canonical list names the board
        const std::string list_out = dir + "/list.txt";
        CHECK(run({otrctl, "--runtime-dir", dir, "--instance", instance, "list"}, list_out) == 0);
        const auto list = slurp(list_out);
        const std::string text(list.begin(), list.end());
        CHECK(text.find("OTR-4TS3-U1") != std::string::npos);

        // ISDB-T on receiver 2 (UHF 13ch), 2 s
        const std::string out_t = dir + "/t.ts";
        CHECK(run({otr_ts, "--runtime-dir", dir, "--instance", instance, "--receiver", "2",
                   "--system", "isdb-t", "--freq", "473142857", "--time", "2", "--output",
                   out_t}) == 0);
        const std::size_t nt = check_channel(slurp(out_t), 2U);
        std::fprintf(stderr, "e2e: ISDB-T receiver 2: %zu packets\n", nt);
        CHECK(nt > 1000U);

        // ISDB-S on receiver 0 (BS1 IF, TSID 0x4010), 2 s
        const std::string out_s = dir + "/s.ts";
        CHECK(run({otr_ts, "--runtime-dir", dir, "--instance", instance, "--receiver", "0",
                   "--system", "isdb-s", "--freq", "1049480000", "--tsid", "16400", "--time",
                   "2", "--output", out_s}) == 0);
        const std::size_t ns = check_channel(slurp(out_s), 0U);
        std::fprintf(stderr, "e2e: ISDB-S receiver 0: %zu packets\n", ns);
        CHECK(ns > 1000U);

        // --device is refused with a hint
        CHECK(run({otr_ts, "--runtime-dir", dir, "--device", "serial:x", "--receiver", "0",
                   "--channel", "T13"}) == 2);
    }

    stop.store(true);
    service.join();
    CHECK(service_rc == 0);
    // After the service: every channel off, the stream stopped (D-50), the
    // receivers were powered and talked to over I2C.
    CHECK(fake.ch_enable == 0U);
    CHECK(!fake.log.empty() && fake.log.back() == "stream0");
    CHECK(fake.i2c_runs > 100U);
    for (const auto& line : fake.log) CHECK(line.rfind("BAD:", 0) != 0);

    std::string cmd = "rm -rf '" + dir + "'";
    (void)std::system(cmd.c_str());
    if (g_failures == 0) std::printf("e2e_tests: all passed\n");
    return g_failures == 0 ? 0 : 1;
}
