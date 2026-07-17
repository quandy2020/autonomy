/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/system/monitor/latency_monitor/latency_monitor.hpp"

#include <mutex>

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
#include <prometheus/family.h>
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#endif

namespace autonomy {
namespace system {
namespace monitor {

namespace {

double StampAgeSeconds(const commsgs::std_msgs::Header& header) {
    using commsgs::builtin_interfaces::Time;
    const Time now = Time::Now();
    if (header.stamp > now)
        return 0.0;
    const commsgs::builtin_interfaces::Duration delta = now - header.stamp;
    return static_cast<double>(delta.Nanoseconds()) * 1e-9;
}

}  // namespace

struct LatencyMonitor::LatencyRuntime {
    LatencyWatchOptions options;
    mutable std::mutex mutex;
    bool ever_received{false};
    double last_age_sec{0.0};
    std::shared_ptr<autolink::Reader<commsgs::geometry_msgs::TwistStamped>>
        reader;
};

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
struct LatencyMonitor::PrometheusGauges {
    prometheus::Family<prometheus::Gauge>* age_family{nullptr};
    prometheus::Family<prometheus::Gauge>* ok_family{nullptr};
    std::vector<prometheus::Gauge*> age;
    std::vector<prometheus::Gauge*> ok;
};
#endif

LatencyMonitor::LatencyMonitor(std::vector<LatencyWatchOptions> watches)
    : watches_(std::move(watches)) {}

LatencyMonitor::~LatencyMonitor() = default;

std::unique_ptr<LatencyMonitor> LatencyMonitor::Create(
    std::vector<LatencyWatchOptions> watches) {
    return std::make_unique<LatencyMonitor>(std::move(watches));
}

std::unique_ptr<LatencyMonitor> CreateLatencyMonitor(
    std::vector<LatencyWatchOptions> watches) {
    return LatencyMonitor::Create(std::move(watches));
}

bool LatencyMonitor::AttachNode(const std::shared_ptr<autolink::Node>& node) {
    if (!node || watches_.empty() || attached_)
        return false;

    node_ = node;
    for (const auto& watch : watches_) {
        if (watch.channel.empty())
            continue;
        auto runtime = std::make_unique<LatencyRuntime>();
        runtime->options = watch;
        LatencyRuntime* raw = runtime.get();
        auto callback =
            [raw](const std::shared_ptr<commsgs::geometry_msgs::TwistStamped>&
                      msg) {
                if (!msg)
                    return;
                const double age = StampAgeSeconds(msg->header);
                std::lock_guard<std::mutex> lock(raw->mutex);
                raw->ever_received = true;
                raw->last_age_sec = age;
            };
        raw->reader =
            node_->CreateReader<commsgs::geometry_msgs::TwistStamped>(
                watch.channel, callback);
        if (!raw->reader)
            continue;
        runtimes_.push_back(std::move(runtime));
    }
    attached_ = !runtimes_.empty();
    return attached_;
}

std::vector<LatencyHealth> LatencyMonitor::SnapshotHealth() const {
    std::vector<LatencyHealth> out;
    for (const auto& rt : runtimes_) {
        LatencyHealth h;
        h.channel = rt->options.channel;
        std::lock_guard<std::mutex> lock(rt->mutex);
        h.ever_received = rt->ever_received;
        h.message_age_sec = rt->last_age_sec;
        if (!h.ever_received) {
            h.healthy = rt->options.max_age_sec <= 0.0;
        } else {
            h.healthy = rt->options.max_age_sec <= 0.0 ||
                        h.message_age_sec <= rt->options.max_age_sec;
        }
        out.push_back(h);
    }
    return out;
}

void LatencyMonitor::Collect() {
    const auto health = SnapshotHealth();
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (!gauges_)
        return;
    for (size_t i = 0; i < health.size(); ++i) {
        while (gauges_->age.size() <= i && gauges_->age_family) {
            const std::string label =
                health[gauges_->age.size()].channel;
            gauges_->age.push_back(
                &gauges_->age_family->Add({{"channel", label}}));
            gauges_->ok.push_back(
                &gauges_->ok_family->Add({{"channel", label}}));
        }
        if (i < gauges_->age.size()) {
            if (gauges_->age[i])
                gauges_->age[i]->Set(health[i].message_age_sec);
            if (gauges_->ok[i])
                gauges_->ok[i]->Set(health[i].healthy ? 1.0 : 0.0);
        }
    }
#else
    (void)health;
#endif
}

void LatencyMonitor::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    gauges_ = std::make_unique<PrometheusGauges>();
    gauges_->age_family = &prometheus::BuildGauge()
                               .Name("autonomy_system_pipeline_latency_seconds")
                               .Help("Message header age in seconds")
                               .Register(*reg);
    gauges_->ok_family = &prometheus::BuildGauge()
                             .Name("autonomy_system_pipeline_latency_ok")
                             .Help("1 if message age within max_age_sec")
                             .Register(*reg);
#else
    (void)registry;
#endif
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
