/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <iostream>
#include <memory>

#include <automsgs/msgs/geometry_msgs/pose_array.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

using autoviz::examples::SetYawPose;
using autoviz::examples::StampHeader;

std::shared_ptr<automsgs::msgs::nav_msgs::Odometry> MakeOdom(double t) {
  const double x = 2.0 * std::cos(t);
  const double y = 1.5 * std::sin(t);
  const double yaw = t + M_PI / 2.0;
  auto msg = std::make_shared<automsgs::msgs::nav_msgs::Odometry>();
  StampHeader(msg->mutable_header());
  msg->set_child_frame_id("base_link");
  auto* pwc = msg->mutable_pose();
  StampHeader(pwc->mutable_pose()->mutable_header());
  SetYawPose(pwc->mutable_pose()->mutable_pose(), x, y, yaw);
  for (int i = 0; i < 36; ++i) pwc->add_covariance(i % 7 == 0 ? 0.05 : 0.0);
  msg->mutable_twist()->mutable_twist()->mutable_linear()->set_x(0.8);
  msg->mutable_twist()->mutable_twist()->mutable_angular()->set_z(0.4);
  for (int i = 0; i < 36; ++i) {
    msg->mutable_twist()->add_covariance(i % 7 == 0 ? 0.01 : 0.0);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::PoseArray> MakePoses(double t) {
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::PoseArray>();
  StampHeader(msg->mutable_header());
  for (int i = 0; i < 5; ++i) {
    const double a = t + i * (2.0 * M_PI / 5.0);
    SetYawPose(msg->add_poses(), std::cos(a), std::sin(a), a, 0.1 * i);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::Imu> MakeImu(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Imu>();
  StampHeader(msg->mutable_header(), "imu_link");
  msg->mutable_orientation()->set_z(std::sin(t * 0.5));
  msg->mutable_orientation()->set_w(std::cos(t * 0.5));
  msg->mutable_angular_velocity()->set_z(0.2 * std::cos(t));
  msg->mutable_linear_acceleration()->set_x(0.1 * std::sin(t));
  msg->mutable_linear_acceleration()->set_z(9.81);
  return msg;
}

}  // namespace

using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 10.0);

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/raw");
  auto w_odom =
      MakeWriter<automsgs::msgs::nav_msgs::Odometry>(node, "/fake/raw/odom");
  auto w_poses = MakeWriter<automsgs::msgs::geometry_msgs::PoseArray>(
      node, "/fake/raw/poses");
  auto w_imu =
      MakeWriter<automsgs::msgs::sensor_msgs::Imu>(node, "/fake/raw/imu");

  autolink::Rate rate(rate_hz);
  std::cout << "raw @ " << rate_hz << " Hz → /fake/raw/{odom,poses,imu}\n";

  const double t0 = NowSec();
  while (autolink::OK()) {
    const double t = NowSec() - t0;
    w_odom->Write(MakeOdom(t));
    w_poses->Write(MakePoses(t));
    w_imu->Write(MakeImu(t));
    rate.Sleep();
  }
  return 0;
}
