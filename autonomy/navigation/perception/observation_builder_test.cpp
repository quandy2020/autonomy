/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "automsgs/msgs/geometry_msgs/transform_stamped.pb.h"
#include "automsgs/msgs/nav_msgs/odometry.pb.h"
#include "navigation/perception/observation_builder.hpp"

int main()
{
    automsgs::msgs::sensor_msgs::Image depth;
    depth.set_width(2);
    depth.set_height(2);
    depth.set_encoding("32FC1");
    const float values[4] = {1.0F, 1.0F, 1.0F, 1.0F};
    depth.set_data(reinterpret_cast<const char*>(values), sizeof(values));
    depth.mutable_header()->mutable_stamp()->set_sec(1);

    navigation::perception::ObservationInputs inputs;
    inputs.depth = &depth;
    inputs.tf_valid = true;
    navigation::observation::Observation observation;
    navigation::perception::ObservationBuilder builder;
    if (!builder.Build(inputs, &observation))
    {
        std::cerr << "build failed\n";
        return 1;
    }
    if (observation.sequence() != 1 ||
        !(observation.validity() & navigation::observation::OBSERVATION_FLAG_DEPTH_VALID) ||
        !(observation.validity() & navigation::observation::OBSERVATION_FLAG_TF_VALID))
    {
        std::cerr << "validity failed\n";
        return 1;
    }
    if (observation.validity() &
        navigation::observation::OBSERVATION_FLAG_CAMERA_INFO_VALID)
    {
        std::cerr << "missing camera marked valid\n";
        return 1;
    }
    if (builder.Build(navigation::perception::ObservationInputs{}, &observation))
    {
        std::cerr << "empty depth accepted\n";
        return 1;
    }
    automsgs::msgs::map_msgs::OccupancyGrid map;
    map.mutable_header()->set_frame_id("map");
    map.mutable_info()->set_width(2); map.mutable_info()->set_height(2);
    map.mutable_info()->set_resolution(.1F);
    for (int i = 0; i < 4; ++i) map.add_data(0);
    inputs.map = &map;
    if (!builder.Build(inputs, &observation) || !observation.has_map() ||
        !(observation.validity() & navigation::observation::OBSERVATION_FLAG_MAP_VALID)) return 1;
    map.mutable_header()->set_frame_id("other_map");
    builder.Build(inputs, &observation);
    if (observation.validity() & navigation::observation::OBSERVATION_FLAG_MAP_VALID) return 1;
    map.mutable_header()->set_frame_id("map"); map.mutable_info()->set_width(0xffffffff);
    builder.Build(inputs, &observation);
    if (observation.validity() & navigation::observation::OBSERVATION_FLAG_MAP_VALID) return 1;
    inputs.map = nullptr;
    builder.Build(inputs, &observation);
    if (observation.has_map() || (observation.validity() & navigation::observation::OBSERVATION_FLAG_MAP_VALID)) return 1;

    automsgs::msgs::nav_msgs::Odometry odom;
    odom.mutable_header()->set_frame_id("odom");
    odom.mutable_pose()->mutable_pose()->mutable_pose()->mutable_position()->set_x(1.0);
    odom.mutable_pose()->mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
    automsgs::msgs::geometry_msgs::TransformStamped map_to_odom;
    map_to_odom.mutable_header()->set_frame_id("map");
    map_to_odom.set_child_frame_id("odom");
    map_to_odom.mutable_transform()->mutable_translation()->set_x(10.0);
    map_to_odom.mutable_transform()->mutable_rotation()->set_w(1.0);
    inputs.odom = &odom;
    inputs.map_to_odom = &map_to_odom;
    if (!builder.Build(inputs, &observation) ||
        observation.odom().header().frame_id() != "map" ||
        std::abs(observation.odom().pose().pose().pose().position().x() - 11.0) > 1e-6) {
        std::cerr << "map-frame odom compose failed\n";
        return 1;
    }
    return 0;
}
