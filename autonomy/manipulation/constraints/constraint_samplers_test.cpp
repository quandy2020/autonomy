/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include <random>

#include "autonomy/manipulation/model/joint_state_utilities.hpp"
#include "autonomy/manipulation/motion/kinematics/null_kinematics.hpp"
#include "autonomy/manipulation/constraints/constraint_sampler_manager.hpp"
#include "autonomy/manipulation/constraints/constraint_sampler_types.hpp"
#include "autonomy/manipulation/constraints/constraint_samplers.hpp"
#include "autonomy/manipulation/constraints/joint_constraint_sampler.hpp"

namespace autonomy {
namespace manipulation {
namespace constraints {
namespace {

planner::MotionPlanRequest MakeJointReq() {
  planner::MotionPlanRequest req;
  req.pb.set_group("arm");
  automsgs::msgs::moveit_msgs::JointConstraint jc;
  jc.set_joint_name("j1");
  jc.set_position(0.5);
  jc.set_tolerance_above(0.1);
  jc.set_tolerance_below(0.1);
  *req.pb.add_joint_constraints() = jc;
  SetJointState(req.pb.mutable_start_state(), {"j1", "j2"}, {0.5, 0.0});
  *req.pb.mutable_goal_state() = req.pb.start_state();
  return req;
}

TEST(ConstraintSamplersTest, ClampProjectsOutOfBandJoints) {
  auto req = MakeJointReq();
  automsgs::msgs::sensor_msgs::JointState q = req.pb.start_state();
  q.set_position(0, 2.0);  // outside [0.4, 0.6]
  EXPECT_FALSE(SatisfiesJointConstraints(req, q));
  ASSERT_TRUE(ClampToJointConstraints(req, &q));
  EXPECT_NEAR(q.position(0), 0.6, 1e-9);
  EXPECT_TRUE(SatisfiesJointConstraints(req, q));
}

TEST(ConstraintSamplersTest, JointSamplerSamplesInsideBand) {
  auto req = MakeJointReq();
  JointConstraintSampler sampler("arm", req);
  std::mt19937 rng(7);
  automsgs::msgs::sensor_msgs::JointState out;
  ASSERT_TRUE(sampler.Sample(req.pb.start_state(), &out, &rng, 8));
  EXPECT_GE(out.position(0), 0.4);
  EXPECT_LE(out.position(0), 0.6);
}

TEST(ConstraintSamplersTest, EvaluateCatchesBadStartJoint) {
  auto req = MakeJointReq();
  req.pb.mutable_start_state()->set_position(0, 9.0);
  std::string err;
  EXPECT_EQ(EvaluateRequestConstraints(req, &err),
            ErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS);
  EXPECT_FALSE(err.empty());
}

TEST(ConstraintSamplersTest, ManagerSelectsJointAllocator) {
  ConstraintSamplerManager mgr;
  auto req = MakeJointReq();
  auto s = mgr.SelectSampler(nullptr, "arm", req);
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->GetName(), "JointConstraintSampler");
}

TEST(ConstraintSamplersTest, ManagerSelectsUnionForMixed) {
  ConstraintSamplerManager mgr;
  auto req = MakeJointReq();
  req.kinematics = std::make_shared<kinematics::NullKinematics>();
  ASSERT_TRUE(req.kinematics->Init("arm", "base", "tool0"));
  automsgs::msgs::moveit_msgs::PositionConstraint pc;
  pc.set_link_name("tool0");
  pc.set_tolerance(0.05);
  *req.pb.add_position_constraints() = pc;
  auto s = mgr.SelectSampler(nullptr, "arm", req);
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->GetName(), "UnionConstraintSampler");
}

}  // namespace
}  // namespace constraints
}  // namespace manipulation
}  // namespace autonomy
