// SPDX-License-Identifier: GPL-2.0-only
// Offline tests for the Fx3Device stream contract, against a fake USB
// backend whose completions arrive on its own "event thread" (as with libusb).
#include "fx3_device.h"

#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

using namespace otr::fx3;

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

class FakeBackend final : public UsbBackend {
public:
    struct Submitted {
        std::uint8_t* buffer = nullptr;
        std::size_t size = 0U;
        bool pending = false;
    };

    FakeBackend() : thread_([this] { run(); }) {}
    ~FakeBackend() override
    {
        {
            std::lock_guard<std::mutex> l(m_);
            quit_ = true;
        }
        cv_.notify_all();
        thread_.join();
    }

    std::vector<Submitted> slots = std::vector<Submitted>(Fx3Device::kMaxTransfers);
    std::size_t submit_count = 0U;

    // Test helper: complete a pending read with `data` (or an error).
    void complete(std::size_t slot, const std::vector<std::uint8_t>& data,
                  Error error = Error::OK)
    {
        std::lock_guard<std::mutex> l(m_);
        queue_.push_back({slot, data, error});
        cv_.notify_all();
    }
    void drain()
    {
        std::unique_lock<std::mutex> l(m_);
        idle_cv_.wait(l, [this] { return queue_.empty() && !busy_; });
    }

    Result<void> control_out(std::uint8_t, std::uint16_t, std::uint16_t, ByteView) noexcept override
    {
        return Result<void>::success();
    }
    Result<std::size_t> control_in(std::uint8_t, std::uint16_t, std::uint16_t,
                                   MutableByteView) noexcept override
    {
        return Result<std::size_t>::success(0U);
    }
    Result<std::size_t> bulk_read(std::uint8_t, MutableByteView, Timeout) noexcept override
    {
        return Result<std::size_t>::failure(Error::UNSUPPORTED);
    }
    Result<std::size_t> bulk_write(std::uint8_t, ByteView, Timeout) noexcept override
    {
        return Result<std::size_t>::failure(Error::UNSUPPORTED);
    }
    Result<void> submit_read(std::size_t slot, std::uint8_t, std::uint8_t* buffer,
                             std::size_t size) noexcept override
    {
        std::lock_guard<std::mutex> l(m_);
        slots[slot] = {buffer, size, true};
        ++submit_count;
        return Result<void>::success();
    }
    void cancel_read(std::size_t slot) noexcept override
    {
        // libusb reports cancellation asynchronously from the event thread.
        std::lock_guard<std::mutex> l(m_);
        queue_.push_back({slot, {}, Error::NOT_READY});
        cv_.notify_all();
    }
    void set_sink(ReadCompletionSink* sink) noexcept override
    {
        std::lock_guard<std::mutex> l(m_);
        sink_ = sink;
    }

private:
    struct Item {
        std::size_t slot;
        std::vector<std::uint8_t> data;
        Error error;
    };
    void run()
    {
        std::unique_lock<std::mutex> l(m_);
        for (;;) {
            cv_.wait(l, [this] { return quit_ || !queue_.empty(); });
            if (quit_)
                return;
            Item it = queue_.front();
            queue_.pop_front();
            Submitted& s = slots[it.slot];
            if (!s.pending) {
                // e.g. a cancel after the read already completed
                if (queue_.empty())
                    idle_cv_.notify_all();
                continue;
            }
            s.pending = false;
            std::size_t n = it.data.size() < s.size ? it.data.size() : s.size;
            if (n != 0U)
                std::memcpy(s.buffer, it.data.data(), n);
            ReadCompletionSink* sink = sink_;
            busy_ = true;
            l.unlock();
            if (sink != nullptr)
                sink->on_read_complete(it.slot, it.error, n);
            l.lock();
            busy_ = false;
            if (queue_.empty())
                idle_cv_.notify_all();
        }
    }

    std::mutex m_;
    std::condition_variable cv_, idle_cv_;
    std::deque<Item> queue_;
    ReadCompletionSink* sink_ = nullptr;
    bool quit_ = false;
    bool busy_ = false;
    std::thread thread_;
};

std::vector<std::uint8_t> fill(std::uint8_t v, std::size_t n) { return std::vector<std::uint8_t>(n, v); }
constexpr Timeout kWait{1000U};

void test_in_order_delivery_and_resubmit()
{
    FakeBackend usb;
    Fx3Device dev(usb);
    CHECK(dev.start_stream(StreamConfig{0x81U, 16U, 3U}));
    CHECK(usb.submit_count == 3U);
    // Completions arrive out of order: 2, 0, 1.
    usb.complete(2U, fill(0xC2U, 16U));
    usb.complete(0U, fill(0xC0U, 16U));
    usb.complete(1U, fill(0xC1U, 16U));
    usb.drain();
    for (std::uint8_t expect : {0xC0U, 0xC1U, 0xC2U}) {
        Result<StreamEvent> ev = dev.wait_stream(kWait);
        CHECK(ev);
        if (!ev)
            continue;
        CHECK(ev.value().kind == StreamEventKind::data);
        CHECK(ev.value().size == 16U);
        CHECK(ev.value().data[0] == expect && ev.value().data[15] == expect);
    }
    // Each wait resubmitted the previous buffer: 3 initial + 2 so far.
    CHECK(usb.submit_count == 5U);
    CHECK(dev.stop_stream());
    CHECK(!dev.stream_active());
}

void test_buffer_valid_until_next_wait()
{
    FakeBackend usb;
    Fx3Device dev(usb);
    CHECK(dev.start_stream(StreamConfig{0x81U, 8U, 1U}));
    usb.complete(0U, fill(0x11U, 8U));
    Result<StreamEvent> ev = dev.wait_stream(kWait);
    CHECK(ev);
    const std::uint8_t* data = ev ? ev.value().data : nullptr;
    // The slot is not resubmitted while the caller holds the event, so a
    // further completion cannot land in it.
    CHECK(!usb.slots[0].pending);
    usb.complete(0U, fill(0x22U, 8U));
    usb.drain();
    CHECK(data != nullptr && data[0] == 0x11U);
    // The next wait resubmits it, and then a new completion is delivered.
    std::thread late([&usb] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        usb.complete(0U, fill(0x33U, 8U));
    });
    ev = dev.wait_stream(kWait);
    late.join();
    CHECK(ev && ev.value().data[0] == 0x33U);
    CHECK(dev.stop_stream());
}

void test_short_timeout_and_errors()
{
    FakeBackend usb;
    Fx3Device dev(usb);
    CHECK(dev.start_stream(StreamConfig{0x81U, 16U, 2U}));
    CHECK(dev.wait_stream(Timeout{30U}).error() == Error::TIMEOUT);   // nothing yet
    usb.complete(0U, fill(0x01U, 5U));
    usb.complete(1U, {}, Error::USB_IO);
    usb.drain();
    Result<StreamEvent> ev = dev.wait_stream(kWait);
    CHECK(ev && ev.value().kind == StreamEventKind::short_transfer && ev.value().size == 5U);
    ev = dev.wait_stream(kWait);
    CHECK(!ev && ev.error() == Error::USB_IO);
    // After an ordinary error the slot is resubmitted and streaming goes on.
    const std::size_t before = usb.submit_count;
    usb.complete(0U, fill(0x02U, 16U));
    ev = dev.wait_stream(kWait);
    CHECK(usb.submit_count == before + 1U);
    CHECK(ev && ev.value().data[0] == 0x02U);
    CHECK(dev.stop_stream());
}

void test_disconnect_not_resubmitted()
{
    FakeBackend usb;
    Fx3Device dev(usb);
    CHECK(dev.start_stream(StreamConfig{0x81U, 16U, 1U}));
    usb.complete(0U, {}, Error::DISCONNECTED);
    Result<StreamEvent> ev = dev.wait_stream(kWait);
    CHECK(!ev && ev.error() == Error::DISCONNECTED);
    const std::size_t before = usb.submit_count;
    CHECK(dev.wait_stream(Timeout{30U}).error() == Error::TIMEOUT);
    CHECK(usb.submit_count == before);   // never resubmitted
    CHECK(dev.stop_stream());
}

void test_stop_cancels_outstanding_and_config_checks()
{
    FakeBackend usb;
    Fx3Device dev(usb);
    CHECK(dev.start_stream(StreamConfig{0x81U, 0U, 1U}).error() == Error::INVALID_ARGUMENT);
    CHECK(dev.start_stream(StreamConfig{0x81U, 16U, Fx3Device::kMaxTransfers + 1U}).error() ==
          Error::INVALID_ARGUMENT);
    CHECK(dev.start_stream(StreamConfig{0x81U, 16384U, 8U}));
    CHECK(dev.start_stream(StreamConfig{0x81U, 16384U, 8U}).error() == Error::BUSY);
    // All 8 reads are outstanding; stop must cancel them and wait for every
    // cancellation to be reported before freeing the buffers.
    CHECK(dev.stop_stream());
    CHECK(!dev.stream_active());
    CHECK(dev.wait_stream(Timeout{10U}).error() == Error::NOT_READY);
    // The device can be started again afterwards.
    CHECK(dev.start_stream(StreamConfig{0x81U, 16384U, 8U}));
    CHECK(dev.stop_stream());
}

}  // namespace

int main()
{
    test_in_order_delivery_and_resubmit();
    test_buffer_valid_until_next_wait();
    test_short_timeout_and_errors();
    test_disconnect_not_resubmitted();
    test_stop_cancels_outstanding_and_config_checks();
    if (g_failures == 0)
        std::printf("fx3_device_tests: all passed\n");
    return g_failures == 0 ? 0 : 1;
}
