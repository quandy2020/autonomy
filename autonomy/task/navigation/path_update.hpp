/* Copyright 2026 The Openbot Authors */
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <automsgs/msgs/nav_msgs/path.pb.h>

namespace autonomy::task::navigation {

inline double PoseYaw(const automsgs::msgs::geometry_msgs::Quaternion& q)
{
    const double norm = q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w();
    if (!std::isfinite(norm) || norm < 1e-12) return std::numeric_limits<double>::quiet_NaN();
    return std::atan2(2.0*(q.w()*q.z() + q.x()*q.y()),
                      norm - 2.0*(q.y()*q.y() + q.z()*q.z()));
}

inline bool CompatibleFrames(const std::string& a, const std::string& b)
{
    return a.empty() || b.empty() || a == b;
}

inline bool SameGoalPose(const automsgs::msgs::geometry_msgs::PoseStamped& a,
                         const automsgs::msgs::geometry_msgs::PoseStamped& b,
                         double xy_tolerance = 0.05, double yaw_tolerance = 0.05)
{
    const auto& x = a.pose().position();
    const auto& y = b.pose().position();
    if (!CompatibleFrames(a.header().frame_id(), b.header().frame_id()) ||
        !std::isfinite(xy_tolerance) || xy_tolerance < 0.0 ||
        !std::isfinite(yaw_tolerance) || yaw_tolerance < 0.0 ||
        !std::isfinite(x.x()) || !std::isfinite(x.y()) ||
        !std::isfinite(y.x()) || !std::isfinite(y.y()) ||
        std::hypot(x.x()-y.x(), x.y()-y.y()) > xy_tolerance) {
        return false;
    }
    const double yaw_a = PoseYaw(a.pose().orientation());
    const double yaw_b = PoseYaw(b.pose().orientation());
    // Missing/zero quaternion: compare XY only (2D Goal retries).
    if (!std::isfinite(yaw_a) || !std::isfinite(yaw_b)) {
        return true;
    }
    return std::abs(std::remainder(yaw_a - yaw_b, 2.0 * std::acos(-1.0))) <=
           yaw_tolerance;
}

// Compare common unexecuted geometry by arc length measured back from the goal.
// Shortening a moving-start prefix or changing point density does not preempt;
// a changed detour, endpoint heading, frame or controller does. The comparison
// is bounded; an oversized/malformed path is conservatively treated as changed.
inline bool EquivalentPathSuffix(const automsgs::msgs::nav_msgs::Path& a,
                                  const automsgs::msgs::nav_msgs::Path& b,
                                  double tolerance = 0.10)
{
    if (a.poses().empty() || b.poses().empty() ||
        !CompatibleFrames(a.header().frame_id(), b.header().frame_id()) ||
        a.poses_size() > 10000 || b.poses_size() > 10000 ||
        !SameGoalPose(a.poses(a.poses_size()-1), b.poses(b.poses_size()-1),
                      0.35, 1.0)) return false;
    auto finite_path = [](const auto& path) {
        for (const auto& pose : path.poses()) {
            const auto& point = pose.pose().position();
            if ((!pose.header().frame_id().empty() &&
                 pose.header().frame_id() != path.header().frame_id()) ||
                !std::isfinite(point.x()) || !std::isfinite(point.y())) return false;
        }
        return true;
    };
    if (!std::isfinite(tolerance) || tolerance < 0.0 || !finite_path(a) || !finite_path(b))
        return false;
    auto lengths = [](const auto& path) {
        std::vector<double> arc(path.poses_size(), 0.0);
        for (int i = path.poses_size()-2; i >= 0; --i) {
            const auto& p = path.poses(i).pose().position();
            const auto& q = path.poses(i+1).pose().position();
            arc[i] = arc[i+1] + std::hypot(p.x()-q.x(), p.y()-q.y());
        }
        return arc;
    };
    const auto aa = lengths(a), bb = lengths(b);
    const double common = std::min(aa.front(), bb.front());
    if (!std::isfinite(aa.front()) || !std::isfinite(bb.front()) || common > 1000.0) return false;
    // A one-point path must not hide an entire replacement route.
    if (common < 1e-6) return std::abs(aa.front()-bb.front()) <= tolerance;
    auto at = [](const auto& path, const auto& arc, double distance, int& i) {
        while (i > 0 && arc[i-1] < distance) --i;
        if (i == 0) return path.poses(0).pose().position();
        const double span = arc[i-1] - arc[i];
        const double u = span > 1e-12 ? std::clamp((distance-arc[i])/span, 0.0, 1.0) : 0.0;
        auto point = path.poses(i).pose().position();
        const auto& previous = path.poses(i-1).pose().position();
        point.set_x(point.x() + u*(previous.x()-point.x()));
        point.set_y(point.y() + u*(previous.y()-point.y()));
        return point;
    };
    int ia = a.poses_size()-1, ib = b.poses_size()-1;
    const int samples = static_cast<int>(std::ceil(common/0.10));
    for (int k = 0; k <= samples; ++k) {
        const double distance = common*k/samples;
        const auto p = at(a, aa, distance, ia), q = at(b, bb, distance, ib);
        if (std::hypot(p.x()-q.x(), p.y()-q.y()) > tolerance) return false;
    }
    return true;
}
}  // namespace autonomy::task::navigation
