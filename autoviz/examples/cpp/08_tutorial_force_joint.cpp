/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include <automsgs/msgs/geometry_msgs/wrench_stamped.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/sensor_msgs/temperature.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

constexpr const char* kJoints[] = {
    "torso_lift_joint",   "head_pan_joint",       "head_tilt_joint",
    "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_elbow_flex_joint",
    "l_wrist_flex_joint", "r_shoulder_pan_joint", "r_shoulder_lift_joint",
    "r_elbow_flex_joint", "r_wrist_flex_joint",
};
constexpr int kJointCount = sizeof(kJoints) / sizeof(kJoints[0]);

using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;
using autoviz::examples::ReadFile;
using autoviz::examples::ResolveUrdfPath;
using autoviz::examples::StampHeader;

std::shared_ptr<automsgs::msgs::sensor_msgs::JointState> MakeJs(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::JointState>();
  StampHeader(msg->mutable_header(), "");
  for (const char* name : kJoints) msg->add_name(name);
  msg->add_position(0.15 + 0.1 * std::sin(t * 0.5));
  msg->add_position(0.6 * std::sin(t * 0.4));
  msg->add_position(0.2 * std::sin(t * 0.3));
  msg->add_position(0.5 + 0.4 * std::sin(t * 0.5));
  msg->add_position(0.4 + 0.3 * std::sin(t * 0.6));
  msg->add_position(-0.8 + 0.4 * std::sin(t * 0.7));
  msg->add_position(0.3 * std::sin(t));
  msg->add_position(-0.5 + 0.4 * std::sin(t * 0.5 + 1));
  msg->add_position(0.4 + 0.3 * std::sin(t * 0.6 + 0.5));
  msg->add_position(-0.8 + 0.4 * std::sin(t * 0.7 + 0.5));
  msg->add_position(0.3 * std::sin(t + 0.5));
  for (int i = 0; i < kJointCount; ++i) {
    msg->add_effort(20.0 * std::sin(t * 0.5 + i * 0.4));
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::tf2_msgs::TFMessage> MakeTf(double t) {
  auto msg = std::make_shared<automsgs::msgs::tf2_msgs::TFMessage>();
  auto* ts = msg->add_transforms();
  StampHeader(ts->mutable_header(), "map");
  ts->set_child_frame_id("base_link");
  ts->mutable_transform()->mutable_translation()->set_x(0.5 * std::cos(t * 0.2));
  ts->mutable_transform()->mutable_translation()->set_y(0.5 * std::sin(t * 0.2));
  ts->mutable_transform()->mutable_rotation()->set_w(1.0);
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::WrenchStamped> MakeWrench(
    double t) {
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::WrenchStamped>();
  StampHeader(msg->mutable_header(), "base_link");
  msg->mutable_wrench()->mutable_force()->set_x(5.0 * std::cos(t));
  msg->mutable_wrench()->mutable_force()->set_z(2.0 + std::sin(t * 1.5));
  msg->mutable_wrench()->mutable_torque()->set_y(1.5 * std::sin(t * 0.8));
  msg->mutable_wrench()->mutable_torque()->set_z(0.8 * std::cos(t * 0.6));
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::Temperature> MakeTemp(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Temperature>();
  StampHeader(msg->mutable_header(), "base_link");
  msg->set_temperature(25.0 + 15.0 * (0.5 + 0.5 * std::sin(t * 0.4)));
  msg->set_variance(0.05);
  return msg;
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 20.0);
  const std::string urdf = ReadFile(ResolveUrdfPath());

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/force");
  auto w_desc = MakeWriter<automsgs::msgs::std_msgs::String>(
      node, "/robot_description");
  auto w_js = MakeWriter<automsgs::msgs::sensor_msgs::JointState>(
      node, "/joint_states");
  auto w_tf = MakeWriter<automsgs::msgs::tf2_msgs::TFMessage>(node, "/tf");
  auto w_w = MakeWriter<automsgs::msgs::geometry_msgs::WrenchStamped>(
      node, "/fake/wrench");
  auto w_t = MakeWriter<automsgs::msgs::sensor_msgs::Temperature>(
      node, "/fake/temperature");

  auto desc = std::make_shared<automsgs::msgs::std_msgs::String>();
  desc->set_data(urdf);

  autolink::Rate rate(rate_hz);
  std::cout << "force @ " << rate_hz
            << " Hz → joint_states(+effort) /fake/wrench /fake/temperature\n";

  const double t0 = NowSec();
  while (autolink::OK()) {
    const double t = NowSec() - t0;
    w_desc->Write(desc);
    w_js->Write(MakeJs(t));
    w_tf->Write(MakeTf(t));
    w_w->Write(MakeWrench(t));
    w_t->Write(MakeTemp(t));
    rate.Sleep();
  }
  return 0;
}
