/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/system/monitor/mrm_handler/mrm_handler.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/system/monitor/hazard_monitor/hazard_monitor.hpp"

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
#include <prometheus/gauge.h>
#include <prometheus/registry.h>
#endif

namespace autonomy {
namespace system {
namespace monitor {

MrmHandler::MrmHandler(Options options) : options_(std::move(options)) {}

std::unique_ptr<MrmHandler> MrmHandler::Create(Options options) {
    return std::make_unique<MrmHandler>(std::move(options));
}

std::unique_ptr<MrmHandler> CreateMrmHandler(MrmHandler::Options opts) {
    return MrmHandler::Create(std::move(opts));
}

void MrmHandler::SetHazardSource(HazardMonitor* hazard) {
    hazard_ = hazard;
}

bool MrmHandler::AttachNode(const std::shared_ptr<autolink::Node>& node) {
    if (!node || attached_)
        return false;
    node_ = node;
    attached_ = true;
    return true;
}

void MrmHandler::PublishStop() {
    if (!node_ || options_.cmd_vel_channel.empty())
        return;
    static std::shared_ptr<autolink::Writer<commsgs::geometry_msgs::TwistStamped>>
        writer;
    if (!writer) {
        writer =
            node_->CreateWriter<commsgs::geometry_msgs::TwistStamped>(
                options_.cmd_vel_channel);
    }
    if (!writer)
        return;
    commsgs::geometry_msgs::TwistStamped msg;
    msg.header.stamp = commsgs::builtin_interfaces::Time::Now();
    msg.header.frame_id = "base_link";
    writer->Write(msg);
}

void MrmHandler::Collect() {
    active_ = false;
    if (!options_.emergency_stop_on_error || hazard_ == nullptr)
        return;

    if (hazard_->level() == HazardLevel::kError) {
        active_ = true;
        PublishStop();
    }

#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    if (active_gauge_)
        static_cast<prometheus::Gauge*>(active_gauge_)->Set(active_ ? 1.0 : 0.0);
#endif
}

void MrmHandler::RegisterWithPrometheus(void* registry) {
#if defined(USE_PROMETHEUS) && USE_PROMETHEUS
    auto* reg = static_cast<prometheus::Registry*>(registry);
    if (reg == nullptr)
        return;
    auto& family = prometheus::BuildGauge()
                       .Name("autonomy_system_mrm_active")
                       .Help("1 while emergency stop is commanding zero cmd_vel")
                       .Register(*reg);
    active_gauge_ = &family.Add({});
#else
    (void)registry;
#endif
}

}  // namespace monitor
}  // namespace system
}  // namespace autonomy
