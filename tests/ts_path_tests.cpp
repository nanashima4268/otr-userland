// SPDX-License-Identifier: GPL-2.0-only
// Host side of the TS path. Tagged packets are produced the way the
// OTR-4TS3-U1 FPGA mux does it (sync byte rewritten to ((ch+1)<<4)|0x07,
// whole packets interleaved across channels). They are cut into arbitrary
// USB transfer sizes and fed to px4-userland's TaggedTsDemux. Every
// receiver must get its packets back bit-exact and in order.
//
// Cross-check with the FPGA simulation (optional, local only): set
// OTR_FPGA_SIM_DIR to a directory that holds ch0..3.hex (the stimulus) and
// out.hex (the FPGA output), produced by otr-4ts3-u1 fpga/sim with
// `tb_check.py prep-synth`. The test then demuxes the real FPGA output.
#include "tagged_ts_demux.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

using px4::userland::ByteView;
using px4::userland::Result;
using px4::userland::TaggedTsDemux;

namespace {

constexpr std::size_t kPkt = 188U;
constexpr std::size_t kCh = 4U;
int g_failures = 0;
#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::fprintf(stderr, "%s:%d: CHECK failed: %s\n", __FILE__, __LINE__, \
                         #cond);                                                 \
            ++g_failures;                                                        \
        }                                                                        \
    } while (0)

// Deterministic generator shared with otr-4ts3-u1 fpga/sim/tb_check.py
// (prep-synth): sync 0x47, PID 0x100+ch, CC increments, LCG payload.
struct Lcg {
    std::uint32_t s;
    std::uint8_t next() noexcept
    {
        s = s * 1103515245U + 12345U;
        return static_cast<std::uint8_t>(s >> 16U);
    }
};

std::vector<std::vector<std::uint8_t>> synth_channel(std::size_t ch, std::size_t count)
{
    std::vector<std::vector<std::uint8_t>> pkts;
    Lcg rng{static_cast<std::uint32_t>(0x1234U + ch)};
    for (std::size_t i = 0; i < count; ++i) {
        std::vector<std::uint8_t> p(kPkt);
        const std::uint16_t pid = static_cast<std::uint16_t>(0x100U + ch);
        p[0] = 0x47U;
        p[1] = static_cast<std::uint8_t>(pid >> 8U);
        p[2] = static_cast<std::uint8_t>(pid & 0xFFU);
        p[3] = static_cast<std::uint8_t>(0x10U | (i & 0x0FU));
        for (std::size_t k = 4; k < kPkt; ++k)
            p[k] = rng.next();
        pkts.push_back(std::move(p));
    }
    return pkts;
}

struct Collected {
    std::vector<std::vector<std::uint8_t>> per_rx[kCh];
};

Result<void> sink(void* ctx, std::size_t rx, ByteView packet) noexcept
{
    auto* c = static_cast<Collected*>(ctx);
    if (rx >= kCh)
        return Result<void>::failure(px4::userland::Error::INTERNAL);
    c->per_rx[rx].emplace_back(packet.data, packet.data + packet.size);
    return Result<void>::success();
}

// Interleave whole tagged packets with uneven per-channel rates, as the FPGA
// round-robin mux does when channels run at different bit rates.
std::vector<std::uint8_t> mux(const std::vector<std::vector<std::uint8_t>> (&src)[kCh],
                              std::uint32_t seed)
{
    std::vector<std::uint8_t> out;
    std::size_t idx[kCh] = {};
    Lcg rng{seed};
    for (;;) {
        bool any = false;
        for (std::size_t ch = 0; ch < kCh; ++ch) {
            if (idx[ch] >= src[ch].size())
                continue;
            any = true;
            // channel weights 1..4 imitate different bit rates
            if ((rng.next() & 3U) > ch)
                continue;
            const auto& p = src[ch][idx[ch]++];
            out.push_back(static_cast<std::uint8_t>(((ch + 1U) << 4U) | 0x07U));
            out.insert(out.end(), p.begin() + 1, p.end());
        }
        if (!any)
            break;
    }
    return out;
}

void feed(TaggedTsDemux& demux, const std::vector<std::uint8_t>& wire, Collected& got,
          std::uint32_t seed)
{
    Lcg rng{seed};
    std::size_t pos = 0;
    while (pos < wire.size()) {
        // USB bulk transfers: 1 byte .. 16 KiB, so packets straddle transfers.
        std::size_t n = 1U + ((static_cast<std::size_t>(rng.next()) << 8U | rng.next()) % 16384U);
        if (n > wire.size() - pos)
            n = wire.size() - pos;
        CHECK(demux.push(ByteView{wire.data() + pos, n}, sink, &got));
        pos += n;
    }
}

void test_synthetic()
{
    std::vector<std::vector<std::uint8_t>> src[kCh];
    for (std::size_t ch = 0; ch < kCh; ++ch)
        src[ch] = synth_channel(ch, 500U + ch * 137U);
    const auto wire = mux(src, 99U);

    TaggedTsDemux demux(TaggedTsDemux::kQ3U4MaxTag);   // tags 1..4
    Collected got;
    feed(demux, wire, got, 7U);
    for (std::size_t ch = 0; ch < kCh; ++ch) {
        CHECK(got.per_rx[ch].size() == src[ch].size());
        CHECK(got.per_rx[ch] == src[ch]);
    }
    const auto c = demux.counters();
    CHECK(c.invalid_tag_packets == 0U);
    CHECK(c.sync_loss_events == 0U);
    CHECK(c.discarded_sync_search_bytes == 0U);
}

// The stream can start mid-packet (e.g. after a restart); the demux must
// re-acquire and then deliver everything that follows intact.
void test_mid_packet_start()
{
    std::vector<std::vector<std::uint8_t>> src[kCh];
    for (std::size_t ch = 0; ch < kCh; ++ch)
        src[ch] = synth_channel(ch, 100U);
    auto wire = mux(src, 5U);
    wire.erase(wire.begin(), wire.begin() + 1000);   // 5 packets + 60 bytes lost

    TaggedTsDemux demux(TaggedTsDemux::kQ3U4MaxTag);
    Collected got;
    feed(demux, wire, got, 11U);
    std::size_t total = 0;
    for (std::size_t ch = 0; ch < kCh; ++ch) {
        total += got.per_rx[ch].size();
        // whatever arrives must be a suffix of the source, in order
        const auto& g = got.per_rx[ch];
        const auto& s = src[ch];
        CHECK(g.size() <= s.size());
        for (std::size_t i = 0; i < g.size(); ++i)
            CHECK(g[g.size() - 1 - i] == s[s.size() - 1 - i]);
    }
    CHECK(total == 400U - 6U);   // 5 lost + the partial one dropped while resyncing
}

std::vector<std::uint8_t> read_hex(const std::string& path, bool& ok)
{
    std::vector<std::uint8_t> v;
    std::ifstream f(path);
    if (!f) {
        ok = false;
        return v;
    }
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '/')
            continue;
        v.push_back(static_cast<std::uint8_t>(std::strtoul(line.c_str(), nullptr, 16)));
    }
    return v;
}

void test_fpga_sim_output()
{
    const char* dir = std::getenv("OTR_FPGA_SIM_DIR");
    if (dir == nullptr) {
        std::printf("fpga cross-check: skipped (OTR_FPGA_SIM_DIR not set)\n");
        return;
    }
    bool ok = true;
    const auto wire = read_hex(std::string(dir) + "/out.hex", ok);
    std::vector<std::vector<std::uint8_t>> src[kCh];
    for (std::size_t ch = 0; ch < kCh; ++ch) {
        const auto raw = read_hex(std::string(dir) + "/ch" + std::to_string(ch) + ".hex", ok);
        for (std::size_t i = 0; i + kPkt <= raw.size(); i += kPkt)
            src[ch].emplace_back(raw.begin() + static_cast<long>(i),
                                 raw.begin() + static_cast<long>(i + kPkt));
    }
    CHECK(ok);
    if (!ok)
        return;
    TaggedTsDemux demux(TaggedTsDemux::kQ3U4MaxTag);
    Collected got;
    feed(demux, wire, got, 3U);
    std::size_t matched = 0;
    for (std::size_t ch = 0; ch < kCh; ++ch) {
        // The FPGA testbench drops one truncated packet on channel 1 on
        // purpose, so require an in-order subsequence rather than equality.
        std::size_t j = 0;
        for (const auto& p : got.per_rx[ch]) {
            while (j < src[ch].size() && src[ch][j] != p)
                ++j;
            CHECK(j < src[ch].size());
            ++j;
            ++matched;
        }
    }
    std::printf("fpga cross-check: %zu packets demuxed and matched\n", matched);
    CHECK(matched > 0U);
}

}  // namespace

int main()
{
    test_synthetic();
    test_mid_packet_start();
    test_fpga_sim_output();
    if (g_failures == 0)
        std::printf("ts_path_tests: all passed\n");
    return g_failures == 0 ? 0 : 1;
}
