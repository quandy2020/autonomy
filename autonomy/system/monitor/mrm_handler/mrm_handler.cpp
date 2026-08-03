/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/system/monitor/mrm_handler/mrm_handler.hpp"

#include "autolink/autolink.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
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
    static std::shared_ptr<autolink::Writer<automsgs::msgs::geometry_msgs::TwistStamped>>
        writer;
    if (!writer) {
        writer =
            node_->CreateWriter<automsgs::msgs::geometry_msgs::TwistStamped>(
                options_.cmd_vel_channel);
    }
    if (!writer)
        return;
    automsgs::msgs::geometry_msgs::TwistStamped msg;
    *msg.mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
    msg.mutable_header()->set_frame_id("base_link");
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
