/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/common/twist2d_convert.hpp"

int main() {
    using namespace navigation::common;  // NOLINT

    automsgs::msgs::geometry_msgs::Twist proto;
    proto.mutable_linear()->set_x(0.5);
    proto.mutable_linear()->set_y(-0.1);
    proto.mutable_angular()->set_z(0.4);

    const Twist2D twist = FromProto(proto);
    // The planar alias stores float values, including after a round trip.
    if (std::abs(twist.x() - 0.5) > 1e-6 || std::abs(twist.y() + 0.1) > 1e-6 ||
        std::abs(twist.theta() - 0.4) > 1e-6) {
        std::cerr << "from proto failed\n";
        return 1;
    }

    automsgs::msgs::geometry_msgs::Twist round_trip;
    ToProto(twist, &round_trip);
    if (std::abs(round_trip.linear().x() - 0.5) > 1e-6 ||
        std::abs(round_trip.angular().z() - 0.4) > 1e-6) {
        std::cerr << "round trip failed\n";
        return 1;
    }
    return 0;
}
