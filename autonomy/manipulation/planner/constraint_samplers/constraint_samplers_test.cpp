/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include <random>

#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/motion/kinematics/stub_kinematics.hpp"
#include "autonomy/manipulation/planner/constraint_samplers/constraint_sampler.hpp"
#include "autonomy/manipulation/planner/constraint_samplers/constraint_sampler_manager.hpp"
#include "autonomy/manipulation/planner/constraint_samplers/constraint_samplers.hpp"

namespace autonomy {
namespace manipulation {
namespace constraint_samplers {
namespace {

planning::MotionPlanRequest MakeJointReq() {
  planning::MotionPlanRequest req;
  req.group = "arm";
  planning::JointConstraint jc;
  jc.set_joint_name("j1");
  jc.set_position(0.5);
  jc.set_tolerance_above(0.1);
  jc.set_tolerance_below(0.1);
  req.joint_constraints.push_back(jc);
  SetJointState(&req.start_state, {"j1", "j2"}, {0.5, 0.0});
  req.goal_state = req.start_state;
  return req;
}

TEST(ConstraintSamplersTest, ClampProjectsOutOfBandJoints) {
  auto req = MakeJointReq();
  core::JointState q = req.start_state;
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
  core::JointState out;
  ASSERT_TRUE(sampler.Sample(req.start_state, &out, &rng, 8));
  EXPECT_GE(out.position(0), 0.4);
  EXPECT_LE(out.position(0), 0.6);
}

TEST(ConstraintSamplersTest, EvaluateCatchesBadStartJoint) {
  auto req = MakeJointReq();
  req.start_state.set_position(0, 9.0);
  std::string err;
  EXPECT_EQ(EvaluateRequestConstraints(req, &err),
            ErrorCode::kStartStateViolatesPathConstraints);
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
  req.kinematics = std::make_shared<kinematics::StubKinematics>();
  ASSERT_TRUE(req.kinematics->Init("arm", "base", "tool0"));
  planning::PositionConstraint pc;
  pc.set_link_name("tool0");
  pc.set_tolerance(0.05);
  req.position_constraints.push_back(pc);
  auto s = mgr.SelectSampler(nullptr, "arm", req);
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->GetName(), "UnionConstraintSampler");
}

}  // namespace
}  // namespace constraint_samplers
}  // namespace manipulation
}  // namespace autonomy
