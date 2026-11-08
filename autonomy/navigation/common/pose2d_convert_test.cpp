/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/common/pose2d_convert.hpp"
#include "automsgs/msgs/time_utils.hpp"

int main()
{
    using namespace navigation::common;  // NOLINT

    constexpr double kHalfPi = 1.5707963267948966;
    automsgs::msgs::geometry_msgs::Pose proto;
    proto.mutable_position()->set_x(1.0);
    proto.mutable_position()->set_y(2.0);
    proto.mutable_orientation()->set_w(std::cos(kHalfPi / 2.0));
    proto.mutable_orientation()->set_z(std::sin(kHalfPi / 2.0));

    const Pose2D pose = FromProto(proto);
    if (std::abs(pose.x() - 1.0) > 1e-9 || std::abs(pose.y() - 2.0) > 1e-9 ||
        std::abs(pose.theta() - kHalfPi) > 1e-9)
    {
        std::cerr << "from proto failed\n";
        return 1;
    }

    automsgs::msgs::geometry_msgs::PoseStamped stamped;
    ToProto(pose, "map", 123456789, &stamped);
    if (stamped.header().frame_id() != "map")
    {
        std::cerr << "frame failed\n";
        return 1;
    }
    if (automsgs::msgs::builtin_interfaces::TimeToNanoseconds(
            stamped.header().stamp()) != 123456789U)
    {
        std::cerr << "stamp failed\n";
        return 1;
    }
    const Pose2D round_trip = FromProto(stamped);
    if (std::abs(round_trip.theta() - kHalfPi) > 1e-9)
    {
        std::cerr << "round trip failed\n";
        return 1;
    }
    return 0;
}
