/*
 * Copyright 2026 The Openbot Authors
 */

#include "gtest/gtest.h"

#include <cmath>
#include <memory>

#include "autonomy/manipulation/common/collision_interface.hpp"
#include "autonomy/manipulation/model/joint_state_utilities.hpp"
#include "autonomy/manipulation/motion/kinematics/null_kinematics.hpp"
#include "autonomy/manipulation/motion/scene/point_cloud_occupancy.hpp"
#include "autonomy/manipulation/motion/scene/simple_planning_scene.hpp"
#include "autonomy/manipulation/motion/servo/cartesian_servo.hpp"

#include <automsgs/msgs/control_msgs/joint_jog.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>

namespace autonomy {
namespace manipulation {
namespace servo {
namespace {

/** Joint-space wall at q0 ≥ limit for servo collision tests. */
class JointWallDetector : public common::CollisionInterface {
 public:
  explicit JointWallDetector(double limit) : limit_(limit) {}

  bool Init(const std::string& /*id*/) override { return true; }

  ::autonomy::manipulation::proto::CollisionResult CheckRobotWorld(
      const automsgs::msgs::sensor_msgs::JointState& state,
      const scene::PlanningScene& /*scene*/) const override {
    ::autonomy::manipulation::proto::CollisionResult r;
    if (state.position_size() > 0 && state.position(0) >= limit_) {
      r.set_collision(true);
      r.set_contact_body_a("ee");
      r.set_contact_body_b("wall");
    }
    return r;
  }

  ::autonomy::manipulation::proto::CollisionResult CheckRobotSelf(
      const automsgs::msgs::sensor_msgs::JointState& /*state*/,
      const scene::PlanningScene* /*scene*/) const override {
    return {};
  }

  ::autonomy::manipulation::proto::DistanceResult DistanceRobotWorld(
      const automsgs::msgs::sensor_msgs::JointState& state,
      const scene::PlanningScene& /*scene*/) const override {
    ::autonomy::manipulation::proto::DistanceResult d;
    const double q0 =
        state.position_size() > 0 ? state.position(0) : 0.0;
    d.set_distance(limit_ - q0);
    d.set_collision(d.distance() <= 0.0);
    if (d.collision()) {
      d.set_distance(0.0);
    }
    return d;
  }

 private:
  double limit_ = 0.1;
};

automsgs::msgs::sensor_msgs::JointState MakeTwoDofState(double q0, double q1) {
  automsgs::msgs::sensor_msgs::JointState q;
  q.add_name("j1");
  q.add_name("j2");
  q.add_position(q0);
  q.add_position(q1);
  return q;
}

TEST(ServoTest, BoundedJointIncrement) {
  auto kin = std::make_shared<kinematics::NullKinematics>();
  ASSERT_TRUE(kin->Init("arm", "base", "tool0"));

  DampedLeastSquaresCartesianServo servo;
  servo.SetKinematics(kin);
  servo.SetState(MakeTwoDofState(0.0, 0.0));
  servo.SetControlPeriod(0.01);
  servo.SetMaxJointVelocity(0.5);
  ASSERT_TRUE(servo.Init());

  automsgs::msgs::geometry_msgs::Twist twist;
  twist.mutable_linear()->set_x(0.1);
  automsgs::msgs::sensor_msgs::JointState out;
  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(servo.Update(twist, &out));
  }
  ASSERT_EQ(out.position_size(), 2);
  EXPECT_LE(std::abs(out.position(0)), 0.5);
  EXPECT_LE(std::abs(out.position(1)), 0.5);
}

TEST(ServoTest, JointJogIntegratesNamedJoints) {
  auto kin = std::make_shared<kinematics::NullKinematics>();
  ASSERT_TRUE(kin->Init("arm", "base", "tool0"));

  DampedLeastSquaresCartesianServo servo;
  servo.SetKinematics(kin);
  servo.SetState(MakeTwoDofState(0.0, 0.0));
  servo.SetControlPeriod(0.1);
  servo.SetMaxJointVelocity(2.0);
  ASSERT_TRUE(servo.Init());

  automsgs::msgs::control_msgs::JointJog jog;
  jog.add_joint_names("j2");
  jog.add_velocities(1.0);
  automsgs::msgs::sensor_msgs::JointState out;
  ASSERT_TRUE(servo.UpdateJointJog(jog, &out));
  EXPECT_NEAR(out.position(0), 0.0, 1e-9);
  EXPECT_NEAR(out.position(1), 0.1, 1e-9);
  ASSERT_EQ(out.velocity_size(), 2);
  EXPECT_NEAR(out.velocity(1), 1.0, 1e-9);
}

TEST(ServoTest, PoseCommandProducesMotion) {
  auto kin = std::make_shared<kinematics::NullKinematics>();
  ASSERT_TRUE(kin->Init("arm", "base", "tool0"));

  DampedLeastSquaresCartesianServo servo;
  servo.SetKinematics(kin);
  servo.SetState(MakeTwoDofState(0.0, 0.0));
  servo.SetControlPeriod(0.01);
  servo.SetMaxJointVelocity(1.0);
  servo.SetTwistFilterAlpha(1.0);
  servo.SetPoseGains(1.0, 1.0);
  ASSERT_TRUE(servo.Init());

  automsgs::msgs::geometry_msgs::PoseStamped cmd;
  SetPose(cmd.mutable_pose(), 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  automsgs::msgs::sensor_msgs::JointState out;
  EXPECT_TRUE(servo.UpdatePose(cmd, &out));
  EXPECT_EQ(out.position_size(), 2);
}

TEST(ServoTest, CollisionProximityDeceleratesJointJog) {
  auto kin = std::make_shared<kinematics::NullKinematics>();
  ASSERT_TRUE(kin->Init("arm", "base", "tool0"));

  auto scene = std::make_shared<scene::SimplePlanningScene>();
  auto det = std::make_shared<JointWallDetector>(0.10);
  ASSERT_TRUE(det->Init("wall"));
  scene->SetCollisionDetector(det);

  DampedLeastSquaresCartesianServo servo;
  servo.SetKinematics(kin);
  servo.SetScene(scene);
  servo.SetCollisionProximityThreshold(0.08, 0.01);
  servo.SetState(MakeTwoDofState(0.05, 0.0));
  servo.SetControlPeriod(0.1);
  servo.SetMaxJointVelocity(1.0);
  ASSERT_TRUE(servo.Init());

  automsgs::msgs::control_msgs::JointJog jog;
  jog.add_joint_names("j1");
  jog.add_velocities(1.0);
  automsgs::msgs::sensor_msgs::JointState out;
  ASSERT_TRUE(servo.UpdateJointJog(jog, &out));
  EXPECT_EQ(servo.GetStatus(), CartesianServoStatus::kDecelerateForCollision);
  EXPECT_LT(out.position(0) - 0.05, 0.099);
  EXPECT_GT(out.position(0), 0.05);
}

TEST(ServoTest, CollisionHaltNearWall) {
  auto kin = std::make_shared<kinematics::NullKinematics>();
  ASSERT_TRUE(kin->Init("arm", "base", "tool0"));

  auto scene = std::make_shared<scene::SimplePlanningScene>();
  auto det = std::make_shared<JointWallDetector>(0.10);
  ASSERT_TRUE(det->Init("wall"));
  scene->SetCollisionDetector(det);

  DampedLeastSquaresCartesianServo servo;
  servo.SetKinematics(kin);
  servo.SetScene(scene);
  servo.SetCollisionProximityThreshold(0.05, 0.02);
  servo.SetState(MakeTwoDofState(0.095, 0.0));
  servo.SetControlPeriod(0.1);
  servo.SetMaxJointVelocity(1.0);
  ASSERT_TRUE(servo.Init());

  automsgs::msgs::control_msgs::JointJog jog;
  jog.add_joint_names("j1");
  jog.add_velocities(1.0);
  automsgs::msgs::sensor_msgs::JointState out;
  EXPECT_FALSE(servo.UpdateJointJog(jog, &out));
  EXPECT_EQ(servo.GetStatus(), CartesianServoStatus::kCollision);
  EXPECT_NEAR(out.position(0), 0.095, 1e-9);
}

TEST(PerceptionSelfFilter, DropsNearLinkOrigins) {
  std::vector<scene::OccupiedPoint> origins = {{0.0, 0.0, 0.0}};
  std::vector<scene::OccupiedPoint> pts = {
      {0.01, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.02, 0.0}};
  const std::size_t removed =
      perception::FilterSelfOccupiedPoints(origins, 0.05, &pts);
  EXPECT_EQ(removed, 2u);
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_NEAR(pts[0].x, 1.0, 1e-9);
}

}  // namespace
}  // namespace servo
}  // namespace manipulation
}  // namespace autonomy
