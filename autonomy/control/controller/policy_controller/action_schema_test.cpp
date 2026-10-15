/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/action/space.hpp"
#include "autonomy/control/controller/policy_controller/config.hpp"

#include "gtest/gtest.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

TEST(ActionSchemaTest, DefaultTwistLayoutDecodesLxAz) {
    proto::PolicyConfig proto;
    proto.set_name("t");
    proto.mutable_observation()->set_kinematic(true);
    proto.mutable_engine()->set_model_path("/dev/null");

    PolicyConfig config;
    std::string error;
    ASSERT_TRUE(PolicyConfig::LoadFromProto(proto, &config, &error)) << error;
    const auto& schema = config.action_schema();
    EXPECT_EQ(schema.tensor_name, "action");
    EXPECT_GE(schema.linear_x_index, 0);
    EXPECT_GE(schema.angular_z_index, 0);

    std::vector<float> raw = {0.4f, -0.2f};
    automsgs::msgs::geometry_msgs::Twist twist;
    ASSERT_TRUE(TwistActionSpace::Decode(raw, schema, &twist, &error)) << error;
    EXPECT_FLOAT_EQ(twist.linear().x(), 0.4f);
    EXPECT_FLOAT_EQ(twist.angular().z(), -0.2f);
}

TEST(ActionSchemaTest, Twist6Layout) {
    proto::PolicyConfig proto;
    proto.set_name("t6");
    proto.mutable_observation()->set_kinematic(true);
    auto* action = proto.mutable_action();
    action->add_layout("linear_velocity_x");
    action->add_layout("linear_velocity_y");
    action->add_layout("linear_velocity_z");
    action->add_layout("angular_velocity_x");
    action->add_layout("angular_velocity_y");
    action->add_layout("angular_velocity_z");
    proto.mutable_engine()->set_model_path("/dev/null");

    PolicyConfig config;
    std::string error;
    ASSERT_TRUE(PolicyConfig::LoadFromProto(proto, &config, &error)) << error;
    std::vector<float> raw = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
    automsgs::msgs::geometry_msgs::Twist twist;
    ASSERT_TRUE(TwistActionSpace::Decode(raw, config.action_schema(), &twist,
                                         &error))
        << error;
    EXPECT_FLOAT_EQ(twist.linear().x(), 0.1f);
    EXPECT_FLOAT_EQ(twist.linear().y(), 0.2f);
    EXPECT_FLOAT_EQ(twist.linear().z(), 0.3f);
    EXPECT_FLOAT_EQ(twist.angular().x(), 0.4f);
    EXPECT_FLOAT_EQ(twist.angular().y(), 0.5f);
    EXPECT_FLOAT_EQ(twist.angular().z(), 0.6f);
}

TEST(PolicyConfigTest, RejectsEmptyObservation) {
    proto::PolicyConfig proto;
    proto.set_name("bad");
    proto.mutable_observation();  // no modalities
    proto.mutable_engine()->set_model_path("/dev/null");
    PolicyConfig config;
    std::string error;
    EXPECT_FALSE(PolicyConfig::LoadFromProto(proto, &config, &error));
    EXPECT_FALSE(error.empty());
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
