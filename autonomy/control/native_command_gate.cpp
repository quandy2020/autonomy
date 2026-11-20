/* Copyright 2026 The Openbot Authors. */
/** @file native_command_gate.cpp
 * @brief Native command/frame/time admission using the existing conservative geometry checker.
 */
#include "autonomy/control/native_command_gate.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy::control {
namespace {
/** @brief Source-compatible age checks retain the original message stamp. */
std::optional<double> Stamp(const automsgs::msgs::std_msgs::Header& header) {
    if (!header.has_stamp() || header.stamp().sec() < 0 || header.stamp().nanosec() >= 1000000000U) return {};
    return header.stamp().sec()+header.stamp().nanosec()*1e-9;
}
}  // namespace

autonomy::navigation::SafetyResult VerifyNativeCommand(
    const automsgs::msgs::geometry_msgs::TwistStamped& command,
    const automsgs::msgs::nav_msgs::Odometry& feedback,
    const perception::MapReadView& map,const NativeCommandGateOptions& options,double now) {
    const auto deny = [](const char* reason) { return autonomy::navigation::SafetyResult{false,0,reason}; };
    for (double value : {now,options.maximum_linear_speed,options.maximum_yaw_rate,options.maximum_feedback_age,
        options.maximum_map_age,options.actuator_watchdog,options.minimum_braking_deceleration})
        if (!std::isfinite(value) || value <= 0) return deny("invalid native command bounds or current clock");
    if (!std::isfinite(options.reaction_time) || options.reaction_time < 0) return deny("invalid native reaction allowance");
    if (!command.has_header() || !command.has_twist() || !feedback.has_header() || !feedback.has_pose() ||
        !feedback.pose().has_pose() || !feedback.pose().pose().has_pose() || !feedback.has_twist() || !feedback.twist().has_twist() ||
        !map.grid || !map.map_epoch || !map.has_height_evidence || map.frame_id.empty() || map.grid->frame_id() != map.frame_id ||
        feedback.header().frame_id() != map.frame_id || command.header().frame_id() != feedback.child_frame_id() ||
        feedback.child_frame_id().empty()) return deny("missing or mismatched native command/geometry/feedback");
    const auto stamp = Stamp(feedback.header()), candidate_stamp = Stamp(command.header());
    if (!stamp || !candidate_stamp || now < *stamp || now-*stamp > options.maximum_feedback_age ||
        now < *candidate_stamp || now-*candidate_stamp > options.maximum_feedback_age ||
        !std::isfinite(map.source_stamp) || now < map.source_stamp || now-map.source_stamp > options.maximum_map_age)
        return deny("stale or future native command evidence");
    const auto& twist = command.twist(); const auto& measured = feedback.twist().twist();
    for (double value : {twist.linear().x(),twist.linear().y(),twist.linear().z(),twist.angular().x(),twist.angular().y(),twist.angular().z(),
        measured.linear().x(),measured.linear().y(),measured.linear().z(),measured.angular().z()})
        if (!std::isfinite(value)) return deny("nonfinite native command or actual velocity");
    if (twist.linear().y() || twist.linear().z() || twist.angular().x() || twist.angular().y() ||
        std::abs(twist.linear().x()) > options.maximum_linear_speed || std::abs(twist.angular().z()) > options.maximum_yaw_rate)
        return deny("native command exceeds the planar drive envelope");
    const auto& pose = feedback.pose().pose().pose(); const auto& q = pose.orientation();
    if (!pose.has_position() || !pose.has_orientation() || !feedback.pose().pose().has_header() ||
        feedback.pose().pose().header().frame_id() != feedback.header().frame_id() ||
        Stamp(feedback.pose().pose().header()) != stamp) return deny("mismatched actual pose acquisition identity");
    for (double value : {pose.position().x(),pose.position().y(),pose.position().z(),q.w(),q.x(),q.y(),q.z()})
        if (!std::isfinite(value)) return deny("nonfinite actual native pose");
    if (std::abs(q.w()*q.w()+q.x()*q.x()+q.y()*q.y()+q.z()*q.z()-1)>1e-3)
        return deny("invalid actual native pose rotation");
    const double speed = std::max(std::abs(twist.linear().x()),std::hypot(measured.linear().x(),measured.linear().y()));
    const double distance = speed*(options.actuator_watchdog+options.reaction_time)+
        speed*speed/(2*options.minimum_braking_deceleration);
    if (!std::isfinite(distance)) return deny("native stopping envelope overflow");
    auto body = options.body;
    if (map.footprint_inflated) { body.robot_radius = 1e-9; body.horizontal_margin = 0; }
    if (map.startup_support) body.startup_support = map.startup_support;
    const double yaw = std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
    try {
        autonomy::navigation::GeometrySafety geometry(map.grid,body);
        return geometry.VerifyPose({pose.position().x(),pose.position().y(),yaw},distance);
    } catch (const std::exception&) { return deny("native geometry checker rejected its input"); }
}

autonomy::navigation::SafetyResult VerifyNativeStartupRotation(
    const automsgs::msgs::geometry_msgs::TwistStamped& command,
    const automsgs::msgs::nav_msgs::Odometry& feedback, double start_x,
    double start_y, double deadline,
    const NativeStartupRotationOptions& options, double now) {
    const auto deny = [](const char* reason) {
        return autonomy::navigation::SafetyResult{false, 0, reason};
    };
    for (double value : {start_x, start_y, deadline, now,
                         options.maximum_position_drift, options.maximum_yaw_rate,
                         options.maximum_feedback_age})
        if (!std::isfinite(value)) return deny("invalid startup envelope or clock");
    if (options.maximum_position_drift <= 0 || options.maximum_yaw_rate <= 0 ||
        options.maximum_feedback_age <= 0 || now > deadline)
        return deny("startup envelope expired or has invalid bounds");
    if (!command.has_header() || !command.has_twist() || !feedback.has_header() ||
        !feedback.has_pose() || !feedback.pose().has_pose() ||
        !feedback.pose().pose().has_pose() || !feedback.pose().pose().has_header())
        return deny("missing startup command or actual feedback");
    const auto stamp = Stamp(feedback.header()), command_stamp = Stamp(command.header());
    if (!stamp || !command_stamp)
        return deny("missing startup command or feedback timestamp");
    if (now < *stamp) return deny("future startup feedback timestamp");
    if (now - *stamp > options.maximum_feedback_age)
        return deny("stale startup feedback timestamp");
    if (now < *command_stamp) return deny("future startup command timestamp");
    if (now - *command_stamp > options.maximum_feedback_age)
        return deny("stale startup command timestamp");
    if (Stamp(feedback.pose().pose().header()) != stamp)
        return deny("mismatched startup pose acquisition timestamp");
    if (command.header().frame_id().empty() ||
        command.header().frame_id() != feedback.child_frame_id() ||
        feedback.header().frame_id() != feedback.pose().pose().header().frame_id())
        return deny("startup command/feedback frame mismatch");
    const auto& twist = command.twist();
    for (double value : {twist.linear().x(), twist.linear().y(), twist.linear().z(),
                         twist.angular().x(), twist.angular().y(), twist.angular().z()})
        if (!std::isfinite(value)) return deny("nonfinite startup rotation command");
    if (std::abs(twist.linear().x()) > 1e-9 || std::abs(twist.linear().y()) > 1e-9 ||
        std::abs(twist.linear().z()) > 1e-9 || std::abs(twist.angular().x()) > 1e-9 ||
        std::abs(twist.angular().y()) > 1e-9 ||
        std::abs(twist.angular().z()) > options.maximum_yaw_rate)
        return deny("startup admission permits only bounded planar yaw");
    const auto& position = feedback.pose().pose().pose().position();
    if (!std::isfinite(position.x()) || !std::isfinite(position.y()))
        return deny("nonfinite startup feedback position");
    const double drift = std::hypot(position.x() - start_x, position.y() - start_y);
    if (drift > options.maximum_position_drift)
        return deny("startup rotation exceeded its deployment-confirmed position envelope");
    return {true, options.maximum_position_drift - drift, "startup yaw admitted"};
}
}  // namespace autonomy::control
