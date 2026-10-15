/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/path_encoder.hpp"

#include "gtest/gtest.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

TEST(PathEncoderTest, RelativeRobotFrame) {
    ObservationSnapshot snapshot;
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_pose()->mutable_position()->set_x(1.0);
    pose.mutable_pose()->mutable_position()->set_y(2.0);
    pose.mutable_pose()->mutable_orientation()->set_w(1.0);
    snapshot.pose = pose;

    auto path = std::make_shared<automsgs::msgs::nav_msgs::Path>();
    auto* p0 = path->add_poses();
    p0->mutable_pose()->mutable_position()->set_x(1.0);
    p0->mutable_pose()->mutable_position()->set_y(2.0);
    auto* p1 = path->add_poses();
    p1->mutable_pose()->mutable_position()->set_x(2.0);
    p1->mutable_pose()->mutable_position()->set_y(2.0);
    snapshot.path = path;

    proto::ObservationInput input;
    input.set_name("observation_path");
    std::vector<float> out;
    std::string error;
    ASSERT_TRUE(EncodePath(snapshot, input, 2, &out, &error)) << error;
    ASSERT_EQ(out.size(), 4u);
    EXPECT_NEAR(out[0], 0.0f, 1e-5);
    EXPECT_NEAR(out[1], 0.0f, 1e-5);
    EXPECT_NEAR(out[2], 1.0f, 1e-5);
    EXPECT_NEAR(out[3], 0.0f, 1e-5);
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
