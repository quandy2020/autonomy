/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/system/monitor/channel_monitor/channel_monitor.hpp"

#include <atomic>
#include <chrono>
#include <mutex>

#include "autolink/autolink.hpp"
#include "autolink/message/raw_message.hpp"

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
#include <prometheus/family.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#endif

namespace autonomy {
namespace system {
namespace monitor {

struct ChannelMonitor::ChannelRuntime {
    ChannelWatchOptions options;
    std::mutex mutex;
    std::chrono::steady_clock::time_point last_rx{
        std::chrono::steady_clock::time_point::min()};
    std::atomic<uint64_t> rx_count{0};
    uint64_t prev_rx_count{0};
    std::chrono::steady_clock::time_point rate_window_start{
        std::chrono::steady_clock::now()};
    double rate_hz{0.0};
    std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> reader;
};

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
struct ChannelMonitor::PrometheusGauges {
    prometheus::Family<prometheus::Gauge>* ok_family{nullptr};
    prometheus::Family<prometheus::Gauge>* age_family{nullptr};
    prometheus::Family<prometheus::Gauge>* rate_family{nullptr};
    std::vector<prometheus::Gauge*> ok;
    std::vector<prometheus::Gauge*> age;
    std::vector<prometheus::Gauge*> rate;
};
#endif

ChannelMonitor::ChannelMonitor(std::vector<ChannelWatchOptions> watches)
    : watches_(std::move(watches)) {}

ChannelMonitor::~ChannelMonitor() = default;

std::unique_ptr<ChannelMonitor> ChannelMonitor::Create(
    std::vector<ChannelWatchOptions> watches) {
    return std::make_unique<ChannelMonitor>(std::move(watches));
}

std::unique_ptr<ChannelMonitor> CreateChannelMonitor(
    std::vector<ChannelWatchOptions> watches) {
    return ChannelMonitor::Create(std::move(watches));
}

bool ChannelMonitor::AttachNode(const std::shared_ptr<autolink::Node>& node) {
    if (!node || watches_.empty() || attached_)
        return false;

    node_ = node;
    for (const auto& watch : watches_) {
        if (watch.channel.empty())
            continue;
        auto runtime = std::make_unique<ChannelRuntime>();
        runtime->options = watch;
        ChannelRuntime* raw = runtime.get();
        auto callback =
            [raw](const std::shared_ptr<autolink::message::RawMessage>&) {
                const auto now = std::chrono::steady_clock::now();
                {
                    std::lock_guard<std::mutex> lock(raw->mutex);
                    raw->last_rx = now;
                }
                raw->rx_count.fetch_add(1, std::memory_order_relaxed);
            };
        raw->reader = node_->CreateReader<autolink::message::RawMessage>(
            watch.channel, callback);
        if (!raw->reader)
            continue;
        runtimes_.push_back(std::move(runtime));
    }
    attached_ = !runtimes_.empty();
    return attached_;
}

std::vector<ChannelHealth> ChannelMonitor::SnapshotHealth() const {
    std::vector<ChannelHealth> out;
    const auto now = std::chrono::steady_clock::now();
    for (const auto& rt : runtimes_) {
        ChannelHealth h;
        h.channel = rt->options.channel;
        std::lock_guard<std::mutex> lock(rt->mutex);
        if (rt->last_rx == std::chrono::steady_clock::time_point::min()) {
            h.ever_received = false;
            h.age_sec = 0.0;
            h.healthy = rt->options.min_rate_hz <= 0.0 &&
                        rt->options.timeout_sec <= 0.0;
        } else {
            h.ever_received = true;
            h.age_sec =
                std::chrono::duration<double>(now - rt->last_rx).count();
            h.rate_hz = rt->rate_hz;
            h.healthy = true;
            if (rt->options.timeout_sec > 0.0 &&
                h.age_sec > rt->options.timeout_sec)
                h.healthy = false;
            if (rt->options.min_rate_hz > 0.0 &&
                h.rate_hz < rt->options.min_rate_hz)
                h.healthy = false;
        }
        out.push_back(h);
    }
    return out;
}

void ChannelMonitor::Collect() {
    const auto now = std::chrono::steady_clock::now();
    for (auto& rt : runtimes_) {
        const uint64_t count = rt->rx_count.load(std::memory_order_relaxed);
        const double window_sec =
            std::chrono::duration<double>(now - rt->rate_window_start).count();
        if (window_sec >= 1.0) {
            rt->rate_hz =
                window_sec > 0.0
                    ? static_cast<double>(count - rt->prev_rx_count) / window_sec
                    : 0.0;
            rt->prev_rx_count = count;
            rt->rate_window_start = now;
        }
    }

    const auto health = SnapshotHealth();

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (!gauges_)
        return;
    for (size_t i = 0; i < health.size(); ++i) {
        const std::string& ch = health[i].channel;
        while (gauges_->ok.size() <= i && gauges_->ok_family) {
            const std::string label =
                health[gauges_->ok.size()].channel;
            gauges_->ok.push_back(
                &gauges_->ok_family->Add({{"channel", label}}));
            gauges_->age.push_back(
                &gauges_->age_family->Add({{"channel", label}}));
            gauges_->rate.push_back(
                &gauges_->rate_family->Add({{"channel", label}}));
        }
        if (i < gauges_->ok.size()) {
            if (gauges_->ok[i])
                gauges_->ok[i]->Set(health[i].healthy ? 1.0 : 0.0);
            if (gauges_->age[i])
                gauges_->age[i]->Set(health[i].age_sec);
            if (gauges_->rate[i])
                gauges_->rate[i]->Set(health[i].rate_hz);
        }
    }
#else
    (void)health;
#endif
}

void ChannelMonitor::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    gauges_ = std::make_unique<PrometheusGauges>();
    gauges_->ok_family = &prometheus::BuildGauge()
                              .Name("autonomy_system_channel_ok")
                              .Help("1 if channel within timeout/rate bounds")
                              .Register(*reg);
    gauges_->age_family = &prometheus::BuildGauge()
                               .Name("autonomy_system_channel_age_seconds")
                               .Help("Seconds since last message")
                               .Register(*reg);
    gauges_->rate_family = &prometheus::BuildGauge()
                                .Name("autonomy_system_channel_rate_hz")
                                .Help("Receive rate over last second")
                                .Register(*reg);
#else
    (void)registry;
#endif
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
