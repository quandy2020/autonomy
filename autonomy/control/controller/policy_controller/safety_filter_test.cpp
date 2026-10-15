/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/action/safety_filter.hpp"

#include "gtest/gtest.h"

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

TEST(SafetyFilterTest, ClampsAndForceZeroOnFailure) {
    proto::PolicyControllerOptions options;
    options.set_hold_on_failure(true);
    options.set_collision_check(false);
    proto::TwistLimits limits;
    limits.set_max_vx(0.5);
    limits.set_max_wz(1.0);

    CommandSafetyFilter filter;
    filter.Configure(options, limits);

    ObservationSnapshot snapshot;
    automsgs::msgs::geometry_msgs::Twist action;
    action.mutable_linear()->set_x(2.0);
    action.mutable_angular()->set_z(3.0);
    automsgs::msgs::geometry_msgs::TwistStamped cmd;
    std::string message;
    const uint32_t ok =
        filter.Apply(action, snapshot, nullptr, 0.05, &cmd, &message);
    EXPECT_EQ(ok, 0u);
    EXPECT_NEAR(cmd.twist().linear().x(), 0.5, 1e-6);
    EXPECT_NEAR(cmd.twist().angular().z(), 1.0, 1e-6);

    // Hold-last path.
    automsgs::msgs::geometry_msgs::TwistStamped held;
    filter.ApplyFailure("boom", &held, &message, /*force_zero=*/false);
    EXPECT_NEAR(held.twist().linear().x(), 0.5, 1e-6);

    // Timeout / trip path.
    automsgs::msgs::geometry_msgs::TwistStamped zeroed;
    filter.ApplyFailure("timeout", &zeroed, &message, /*force_zero=*/true);
    EXPECT_NEAR(zeroed.twist().linear().x(), 0.0, 1e-9);
    EXPECT_NEAR(zeroed.twist().angular().z(), 0.0, 1e-9);
}

TEST(InferBudgetTest, DetectsTimeout) {
    InferBudget budget;
    budget.timeout_sec = 0.001;
    budget.start = std::chrono::steady_clock::now() -
                   std::chrono::milliseconds(5);
    EXPECT_TRUE(budget.Exceeded());
    EXPECT_NEAR(budget.RemainingSec(), 0.0, 1e-9);
    budget.timeout_sec = 0.0;
    EXPECT_FALSE(budget.Exceeded());
}

TEST(SafetyFilterTest, PureRotationSkipsWhenCollisionDisabled) {
    proto::PolicyControllerOptions options;
    options.set_collision_check(false);
    proto::TwistLimits limits;
    limits.set_max_vx(0.5);
    limits.set_max_wz(1.0);

    CommandSafetyFilter filter;
    filter.Configure(options, limits);

    ObservationSnapshot snapshot;
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_pose()->mutable_orientation()->set_w(1.0);
    snapshot.pose = pose;

    automsgs::msgs::geometry_msgs::Twist action;
    action.mutable_angular()->set_z(0.8);
    automsgs::msgs::geometry_msgs::TwistStamped cmd;
    std::string message;
    const uint32_t ok =
        filter.Apply(action, snapshot, nullptr, 0.05, &cmd, &message);
    EXPECT_EQ(ok, 0u);
    EXPECT_NEAR(cmd.twist().angular().z(), 0.8, 1e-6);
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
