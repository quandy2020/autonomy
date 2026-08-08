/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

#include <automsgs/msgs/geometry_msgs/pose_array.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_with_covariance_stamped.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

constexpr int kN = 40;

using autoviz::examples::MakeWriter;
using autoviz::examples::ParseFlag;
using autoviz::examples::ParseRate;
using autoviz::examples::SetYawPose;
using autoviz::examples::StampHeader;

void OnPath(double u, double phase, double* x, double* y, double* yaw) {
  *x = u * 8.0 - 4.0;
  *y = 2.0 * std::sin(u * 2.0 * M_PI + phase);
  const double dy = 4.0 * M_PI * std::cos(u * 2.0 * M_PI + phase);
  *yaw = std::atan2(dy, 8.0);
}

std::vector<double> CovXy(double sx, double sy) {
  std::vector<double> c(36, 0.0);
  c[0] = sx * sx;
  c[7] = sy * sy;
  c[14] = c[21] = c[28] = c[35] = 1e-4;
  return c;
}

std::shared_ptr<automsgs::msgs::nav_msgs::Path> MakePath(double phase) {
  auto msg = std::make_shared<automsgs::msgs::nav_msgs::Path>();
  StampHeader(msg->mutable_header());
  for (int i = 0; i < kN; ++i) {
    double x, y, yaw;
    OnPath(static_cast<double>(i) / (kN - 1), phase, &x, &y, &yaw);
    auto* ps = msg->add_poses();
    StampHeader(ps->mutable_header());
    SetYawPose(ps->mutable_pose(), x, y, yaw);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::nav_msgs::Odometry> MakeOdom(double phase,
                                                            double u) {
  double x, y, yaw;
  OnPath(u, phase, &x, &y, &yaw);
  auto msg = std::make_shared<automsgs::msgs::nav_msgs::Odometry>();
  StampHeader(msg->mutable_header());
  msg->set_child_frame_id("base_link");
  auto* pwc = msg->mutable_pose();
  StampHeader(pwc->mutable_pose()->mutable_header());
  SetYawPose(pwc->mutable_pose()->mutable_pose(), x, y, yaw);
  for (double v : CovXy(0.05, 0.05)) pwc->add_covariance(v);
  msg->mutable_twist()->mutable_twist()->mutable_linear()->set_x(0.8);
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::PoseStamped> MakePose(
    double phase) {
  double x, y, yaw;
  OnPath(1.0, phase, &x, &y, &yaw);
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::PoseStamped>();
  StampHeader(msg->mutable_header());
  SetYawPose(msg->mutable_pose(), x, y, yaw, 0.1);
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::PoseArray> MakePoseArray(
    double phase) {
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::PoseArray>();
  StampHeader(msg->mutable_header());
  for (int i = 0; i < 8; ++i) {
    double x, y, yaw;
    OnPath(static_cast<double>(i) / 7.0, phase, &x, &y, &yaw);
    SetYawPose(msg->add_poses(), x, y + 0.4, yaw);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped>
MakePoseCov(double phase, double u) {
  double x, y, yaw;
  OnPath(u, phase, &x, &y, &yaw);
  auto msg = std::make_shared<
      automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped>();
  StampHeader(msg->mutable_header());
  auto* pwc = msg->mutable_pose();
  StampHeader(pwc->mutable_pose()->mutable_header());
  SetYawPose(pwc->mutable_pose()->mutable_pose(), x, y - 0.5, yaw);
  for (double v : CovXy(0.3, 0.15)) pwc->add_covariance(v);
  return msg;
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 10.0);
  const bool is_static = ParseFlag(argc, argv, "--static");

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/poses");
  auto w_odom =
      MakeWriter<automsgs::msgs::nav_msgs::Odometry>(node, "/fake/odom");
  auto w_path = MakeWriter<automsgs::msgs::nav_msgs::Path>(node, "/fake/path");
  auto w_pose = MakeWriter<automsgs::msgs::geometry_msgs::PoseStamped>(
      node, "/fake/pose");
  auto w_arr = MakeWriter<automsgs::msgs::geometry_msgs::PoseArray>(
      node, "/fake/pose_array");
  auto w_cov =
      MakeWriter<automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped>(
          node, "/fake/pose_with_covariance");

  autolink::Rate rate(rate_hz);
  std::cout << "poses @ " << rate_hz
            << " Hz → /fake/{odom,path,pose,pose_array,pose_with_covariance}\n";

  double phase = 0.0;
  double u = 0.0;
  while (autolink::OK()) {
    w_path->Write(MakePath(phase));
    w_odom->Write(MakeOdom(phase, u));
    w_pose->Write(MakePose(phase));
    w_arr->Write(MakePoseArray(phase));
    w_cov->Write(MakePoseCov(phase, u));
    if (!is_static) {
      u = std::fmod(u + 0.01, 1.0);
      phase += 0.02;
    }
    rate.Sleep();
  }
  return 0;
}
