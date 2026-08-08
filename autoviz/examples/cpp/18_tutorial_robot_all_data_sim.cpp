/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <automsgs/msgs/action_msgs/goal_status.pb.h>
#include <automsgs/msgs/action_msgs/goal_status_array.pb.h>
#include <automsgs/msgs/diagnostic_msgs/diagnostic_array.pb.h>
#include <automsgs/msgs/diagnostic_msgs/diagnostic_status.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_with_covariance_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/wrench_stamped.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/sensor_msgs/battery_state.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>
#include <automsgs/msgs/sensor_msgs/joy.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/nav_sat_fix.pb.h>
#include <automsgs/msgs/sensor_msgs/nav_sat_status.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/point_field.pb.h>
#include <automsgs/msgs/sensor_msgs/range.pb.h>
#include <automsgs/msgs/std_msgs/float64.pb.h>
#include <automsgs/msgs/std_msgs/string.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/vision_msgs/detection3d_array.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>

#include "common/tutorial_utils.hpp"

namespace {

constexpr int kRW = 96;
constexpr int kRH = 72;
constexpr int kDW = 64;
constexpr int kDH = 48;
constexpr int kGW = 40;
constexpr int kGH = 30;
constexpr float kGRes = 0.1f;

constexpr const char* kJoints[] = {
    "torso_lift_joint",     "head_pan_joint",       "head_tilt_joint",
    "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_elbow_flex_joint",
    "l_wrist_flex_joint",   "r_shoulder_pan_joint", "r_shoulder_lift_joint",
    "r_elbow_flex_joint",   "r_wrist_flex_joint",
};
constexpr const char* kTasks[] = {"IDLE", "NAVIGATING", "GRASPING", "SPEAKING",
                                  "DONE"};
constexpr int kNumJoints =
    static_cast<int>(sizeof(kJoints) / sizeof(kJoints[0]));
constexpr int kNumTasks = static_cast<int>(sizeof(kTasks) / sizeof(kTasks[0]));

using autoviz::examples::MakeRawWriter;
using autoviz::examples::MakeWriter;
using autoviz::examples::NowSec;
using autoviz::examples::ParseRate;
using autoviz::examples::ReadFile;
using autoviz::examples::ResolveUrdfPath;
using autoviz::examples::SetYawPose;
using autoviz::examples::StampHeader;

void RobotXy(double t, double* x, double* y, double* yaw) {
  *x = 2.0 * std::cos(t * 0.25);
  *y = 1.5 * std::sin(t * 0.25);
  *yaw = t * 0.25 + M_PI / 2;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::Image> MakeRgb(
    int phase, const std::string& frame = "camera") {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Image>();
  StampHeader(msg->mutable_header(), frame);
  msg->set_height(kRH);
  msg->set_width(kRW);
  msg->set_encoding("rgb8");
  msg->set_step(kRW * 3);
  std::string buf(static_cast<size_t>(kRH * kRW * 3), '\0');
  for (int y = 0; y < kRH; ++y) {
    for (int x = 0; x < kRW; ++x) {
      const size_t i = static_cast<size_t>((y * kRW + x) * 3);
      buf[i] = static_cast<char>((x + phase) % 256);
      buf[i + 1] = static_cast<char>((y + phase / 2) % 256);
      buf[i + 2] = static_cast<char>(80);
    }
  }
  msg->set_data(std::move(buf));
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::Image> MakeDepth() {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Image>();
  StampHeader(msg->mutable_header(), "camera");
  msg->set_height(kDH);
  msg->set_width(kDW);
  msg->set_encoding("16UC1");
  msg->set_step(kDW * 2);
  std::string buf;
  buf.reserve(static_cast<size_t>(kDH * kDW * 2));
  const int cx = kDW / 2;
  const int cy = kDH / 2;
  for (int y = 0; y < kDH; ++y) {
    for (int x = 0; x < kDW; ++x) {
      const uint16_t mm =
          ((x - cx) * (x - cx) + (y - cy) * (y - cy) < 80) ? 1200 : 2200;
      char bytes[2];
      std::memcpy(bytes, &mm, 2);
      buf.append(bytes, 2);
    }
  }
  msg->set_data(std::move(buf));
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::LaserScan> MakeScan(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::LaserScan>();
  StampHeader(msg->mutable_header(), "laser");
  constexpr int n = 180;
  msg->set_angle_min(-M_PI / 2);
  msg->set_angle_max(M_PI / 2);
  msg->set_angle_increment(M_PI / (n - 1));
  msg->set_range_min(0.1f);
  msg->set_range_max(10.0f);
  msg->set_scan_time(0.1f);
  for (int i = 0; i < n; ++i) {
    const double a = msg->angle_min() + i * msg->angle_increment();
    msg->add_ranges(static_cast<float>(2.0 + 0.4 * std::sin(a * 3 + t)));
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::PointCloud2> MakePc2(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::PointCloud2>();
  StampHeader(msg->mutable_header(), "camera");
  constexpr int n = 60;
  msg->set_height(1);
  msg->set_width(n);
  msg->set_is_dense(true);
  msg->set_point_step(12);
  msg->set_row_step(12 * n);
  const char* names[] = {"x", "y", "z"};
  const uint32_t offs[] = {0, 4, 8};
  for (int i = 0; i < 3; ++i) {
    auto* f = msg->add_fields();
    f->set_name(names[i]);
    f->set_offset(offs[i]);
    f->set_count(1);
    f->set_datatype(automsgs::msgs::sensor_msgs::PointField::FLOAT32);
  }
  std::string buf(static_cast<size_t>(12 * n), '\0');
  for (int i = 0; i < n; ++i) {
    float xyz[3] = {static_cast<float>(0.05 * i),
                    static_cast<float>(std::sin(0.2 * i + t)),
                    static_cast<float>(0.05 * (i % 5))};
    std::memcpy(&buf[static_cast<size_t>(i * 12)], xyz, 12);
  }
  msg->set_data(std::move(buf));
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::Imu> MakeImu(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Imu>();
  StampHeader(msg->mutable_header(), "imu_link");
  const double yaw = 0.2 * std::sin(t);
  msg->mutable_orientation()->set_z(std::sin(yaw * 0.5));
  msg->mutable_orientation()->set_w(std::cos(yaw * 0.5));
  msg->mutable_angular_velocity()->set_z(0.25 * std::cos(t));
  msg->mutable_linear_acceleration()->set_z(9.81);
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::NavSatFix> MakeGps(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::NavSatFix>();
  StampHeader(msg->mutable_header(), "gps");
  msg->mutable_status()->set_status(
      automsgs::msgs::sensor_msgs::NavSatStatus::STATUS_FIX);
  msg->mutable_status()->set_service(
      automsgs::msgs::sensor_msgs::NavSatStatus::SERVICE_GPS);
  msg->set_latitude(31.2304 + 1e-5 * std::sin(t * 0.1));
  msg->set_longitude(121.4737 + 1e-5 * std::cos(t * 0.1));
  msg->set_altitude(12.0);
  for (double v : {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 4.0}) {
    msg->add_position_covariance(v);
  }
  msg->set_position_covariance_type(
      automsgs::msgs::sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::BatteryState> MakeBattery(
    double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::BatteryState>();
  StampHeader(msg->mutable_header());
  const float pct = static_cast<float>(0.75 + 0.1 * std::sin(t * 0.05));
  msg->set_voltage(12.4f);
  msg->set_current(-1.2f);
  msg->set_percentage(pct);
  msg->set_capacity(5.0f);
  msg->set_design_capacity(5.2f);
  msg->set_present(true);
  msg->set_power_supply_status(
      automsgs::msgs::sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING);
  msg->set_power_supply_health(
      automsgs::msgs::sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD);
  msg->set_power_supply_technology(
      automsgs::msgs::sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION);
  msg->set_location("base");
  msg->set_serial_number("BAT-001");
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::Range> MakeRange(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Range>();
  StampHeader(msg->mutable_header(), "ultrasonic");
  msg->set_radiation_type(automsgs::msgs::sensor_msgs::Range::ULTRASOUND);
  msg->set_field_of_view(0.3f);
  msg->set_min_range(0.05f);
  msg->set_max_range(4.0f);
  msg->set_range(static_cast<float>(1.2 + 0.4 * std::sin(t)));
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::JointState> MakeJs(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::JointState>();
  StampHeader(msg->mutable_header());
  for (int i = 0; i < kNumJoints; ++i) {
    msg->add_name(kJoints[i]);
    msg->add_position(0.2 * std::sin(t * 0.4 + i));
    msg->add_velocity(0.1 * std::cos(t * 0.4 + i));
    msg->add_effort(10 * std::sin(t * 0.5 + i * 0.3));
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::WrenchStamped> MakeWrench(
    double t) {
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::WrenchStamped>();
  StampHeader(msg->mutable_header(), "gripper");
  msg->mutable_wrench()->mutable_force()->set_z(2.0 + std::sin(t));
  msg->mutable_wrench()->mutable_torque()->set_y(0.5 * std::cos(t));
  return msg;
}

std::shared_ptr<automsgs::msgs::nav_msgs::Odometry> MakeOdom(double t) {
  double x, y, yaw;
  RobotXy(t, &x, &y, &yaw);
  auto msg = std::make_shared<automsgs::msgs::nav_msgs::Odometry>();
  StampHeader(msg->mutable_header());
  msg->set_child_frame_id("base_link");
  StampHeader(msg->mutable_pose()->mutable_pose()->mutable_header());
  SetYawPose(msg->mutable_pose()->mutable_pose()->mutable_pose(), x, y, yaw);
  msg->mutable_twist()->mutable_twist()->mutable_linear()->set_x(0.4);
  msg->mutable_twist()->mutable_twist()->mutable_angular()->set_z(0.25);
  return msg;
}

std::shared_ptr<automsgs::msgs::tf2_msgs::TFMessage> MakeTf(double t) {
  double x, y, yaw;
  RobotXy(t, &x, &y, &yaw);
  auto msg = std::make_shared<automsgs::msgs::tf2_msgs::TFMessage>();
  struct Frame {
    const char* parent;
    const char* child;
    double tx, ty, tz, ryaw;
  };
  const Frame frames[] = {
      {"map", "odom", 0.05, 0.0, 0.0, 0.0},
      {"odom", "base_link", x, y, 0.0, yaw},
      {"base_link", "laser", 0.1, 0.0, 0.2, 0.0},
      {"base_link", "camera", 0.15, 0.0, 0.4, 0.0},
      {"base_link", "imu_link", 0.0, 0.0, 0.3, 0.0},
  };
  for (const auto& f : frames) {
    auto* tr = msg->add_transforms();
    StampHeader(tr->mutable_header(), f.parent);
    tr->set_child_frame_id(f.child);
    tr->mutable_transform()->mutable_translation()->set_x(f.tx);
    tr->mutable_transform()->mutable_translation()->set_y(f.ty);
    tr->mutable_transform()->mutable_translation()->set_z(f.tz);
    tr->mutable_transform()->mutable_rotation()->set_z(std::sin(f.ryaw * 0.5));
    tr->mutable_transform()->mutable_rotation()->set_w(std::cos(f.ryaw * 0.5));
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid> MakeGrid(
    int channel_tag) {
  auto msg = std::make_shared<automsgs::msgs::map_msgs::OccupancyGrid>();
  StampHeader(msg->mutable_header());
  auto* info = msg->mutable_info();
  info->set_resolution(kGRes);
  info->set_width(kGW);
  info->set_height(kGH);
  info->mutable_origin()->mutable_position()->set_x(-0.5 * kGW * kGRes);
  info->mutable_origin()->mutable_position()->set_y(-0.5 * kGH * kGRes);
  info->mutable_origin()->mutable_orientation()->set_w(1.0);
  for (int y = 0; y < kGH; ++y) {
    for (int x = 0; x < kGW; ++x) {
      const bool edge = (x == 0 || x == kGW - 1 || y == 0 || y == kGH - 1);
      if (edge) {
        msg->add_data(100);
      } else if ((x + y + channel_tag) % 11 == 0) {
        msg->add_data(50);
      } else {
        msg->add_data(0);
      }
    }
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped>
MakeAmcl(double t) {
  double x, y, yaw;
  RobotXy(t, &x, &y, &yaw);
  auto msg = std::make_shared<
      automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped>();
  StampHeader(msg->mutable_header());
  StampHeader(msg->mutable_pose()->mutable_pose()->mutable_header());
  SetYawPose(msg->mutable_pose()->mutable_pose()->mutable_pose(), x + 0.02,
             y - 0.01, yaw);
  for (int i = 0; i < 36; ++i) {
    msg->mutable_pose()->add_covariance(i % 7 == 0 ? 0.05 : 0.0);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::nav_msgs::Path> MakePath(double t, int n = 30,
                                                        bool local = false) {
  auto msg = std::make_shared<automsgs::msgs::nav_msgs::Path>();
  StampHeader(msg->mutable_header());
  double x0, y0, yaw0;
  RobotXy(t, &x0, &y0, &yaw0);
  (void)yaw0;
  for (int i = 0; i < n; ++i) {
    const double u = static_cast<double>(i) / std::max(n - 1, 1);
    auto* ps = msg->add_poses();
    StampHeader(ps->mutable_header());
    if (local) {
      SetYawPose(ps->mutable_pose(), x0 + 0.4 * u,
                 y0 + 0.15 * std::sin(u * 6 + t), 0.0);
    } else {
      const double a = t * 0.25 + u * 1.5;
      SetYawPose(ps->mutable_pose(), 2.0 * std::cos(a), 1.5 * std::sin(a),
                 a + M_PI / 2);
    }
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::Twist> MakeCmd(double t) {
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::Twist>();
  msg->mutable_linear()->set_x(0.3 + 0.1 * std::sin(t));
  msg->mutable_angular()->set_z(0.2 * std::cos(t * 0.5));
  return msg;
}

std::shared_ptr<automsgs::msgs::action_msgs::GoalStatusArray> MakeNavStatus(
    double t) {
  using GS = automsgs::msgs::action_msgs::GoalStatus;
  auto msg = std::make_shared<automsgs::msgs::action_msgs::GoalStatusArray>();
  auto* st = msg->add_status_list();
  std::string uuid(16, '\0');
  for (int i = 0; i < 16; ++i) uuid[i] = static_cast<char>((i * 17) % 256);
  st->mutable_goal_info()->mutable_goal_id()->set_uuid(uuid);
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  st->mutable_goal_info()->mutable_stamp()->set_sec(
      static_cast<int32_t>(ns / 1000000000LL));
  st->mutable_goal_info()->mutable_stamp()->set_nanosec(
      static_cast<uint32_t>(ns % 1000000000LL));
  const int phase = static_cast<int>(t) % 20;
  if (phase > 16) {
    st->set_status(GS::STATUS_SUCCEEDED);
  } else if (phase > 14) {
    st->set_status(GS::STATUS_ABORTED);
  } else {
    st->set_status(GS::STATUS_EXECUTING);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::geometry_msgs::PoseStamped> MakeWaypoint(
    double t) {
  auto msg = std::make_shared<automsgs::msgs::geometry_msgs::PoseStamped>();
  StampHeader(msg->mutable_header());
  SetYawPose(msg->mutable_pose(), 3.0 * std::cos(t * 0.05),
             2.0 * std::sin(t * 0.05), 0.0, 0.0);
  return msg;
}

std::shared_ptr<automsgs::msgs::vision_msgs::Detection3DArray> MakeDetections3d(
    double t, bool people) {
  auto msg = std::make_shared<automsgs::msgs::vision_msgs::Detection3DArray>();
  StampHeader(msg->mutable_header());
  const char* labels_people[] = {"person"};
  const char* labels_obj[] = {"cup", "bottle", "chair"};
  const char** labels = people ? labels_people : labels_obj;
  const int n = people ? 1 : 3;
  for (int i = 0; i < n; ++i) {
    const double a = t * 0.4 + i * 2.0;
    auto* det = msg->add_detections();
    StampHeader(det->mutable_header());
    det->set_id(std::string(labels[i]) + "_" + std::to_string(i));
    det->mutable_bbox()->mutable_center()->mutable_position()->set_x(
        1.5 * std::cos(a));
    det->mutable_bbox()->mutable_center()->mutable_position()->set_y(
        1.5 * std::sin(a));
    det->mutable_bbox()->mutable_center()->mutable_position()->set_z(
        people ? 0.6 : 0.3);
    det->mutable_bbox()->mutable_center()->mutable_orientation()->set_w(1.0);
    det->mutable_bbox()->mutable_size()->set_x(0.4);
    det->mutable_bbox()->mutable_size()->set_y(0.4);
    det->mutable_bbox()->mutable_size()->set_z(people ? 1.6 : 0.4);
    auto* hyp = det->add_results();
    hyp->mutable_hypothesis()->set_class_id(labels[i]);
    hyp->mutable_hypothesis()->set_score(0.8 + 0.05 * i);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::sensor_msgs::Joy> MakeJoy(double t) {
  auto msg = std::make_shared<automsgs::msgs::sensor_msgs::Joy>();
  StampHeader(msg->mutable_header());
  msg->add_axes(static_cast<float>(std::sin(t)));
  msg->add_axes(static_cast<float>(std::cos(t * 0.7)));
  msg->add_axes(0.0f);
  msg->add_axes(0.0f);
  msg->add_buttons(std::sin(t) > 0 ? 1 : 0);
  msg->add_buttons(0);
  msg->add_buttons(0);
  msg->add_buttons(0);
  return msg;
}

std::shared_ptr<automsgs::msgs::diagnostic_msgs::DiagnosticArray>
MakeDiagnostics(double t) {
  using DS = automsgs::msgs::diagnostic_msgs::DiagnosticStatus;
  auto msg =
      std::make_shared<automsgs::msgs::diagnostic_msgs::DiagnosticArray>();
  StampHeader(msg->mutable_header());
  struct Row {
    DS::Level level;
    const char* name;
    std::string message;
  };
  const Row rows[] = {
      {DS::OK, "power:battery",
       std::to_string(static_cast<int>(75 + 5 * std::sin(t * 0.05))) + "%"},
      {std::sin(t) > 0.85 ? DS::WARN : DS::OK, "sensor:lidar", "streaming"},
      {DS::OK, "nav:controller", "tracking"},
      {static_cast<int>(t) % 25 > 22 ? DS::ERROR : DS::OK, "network:wifi",
       "ok"},
  };
  for (const auto& r : rows) {
    auto* st = msg->add_status();
    st->set_level(r.level);
    st->set_name(r.name);
    st->set_message(r.message);
    st->set_hardware_id(r.name);
  }
  return msg;
}

std::shared_ptr<automsgs::msgs::visualization_msgs::MarkerArray> MakeMarkers(
    double t, const std::string& ns) {
  using M = automsgs::msgs::visualization_msgs::Marker;
  auto arr =
      std::make_shared<automsgs::msgs::visualization_msgs::MarkerArray>();
  for (int i = 0; i < 4; ++i) {
    auto* m = arr->add_markers();
    m->set_ns(ns);
    m->set_id(i);
    m->set_type(ns == "tf_debug" ? M::SPHERE : M::ARROW);
    m->set_action(M::ADD);
    StampHeader(m->mutable_header());
    const double a = t + i;
    m->mutable_pose()->mutable_position()->set_x(std::cos(a));
    m->mutable_pose()->mutable_position()->set_y(std::sin(a));
    m->mutable_pose()->mutable_position()->set_z(0.2 * i);
    m->mutable_pose()->mutable_orientation()->set_w(1.0);
    if (m->type() == M::ARROW) {
      m->mutable_scale()->set_x(0.4);
      m->mutable_scale()->set_y(0.05);
      m->mutable_scale()->set_z(0.05);
    } else {
      m->mutable_scale()->set_x(0.15);
      m->mutable_scale()->set_y(0.15);
      m->mutable_scale()->set_z(0.15);
    }
    m->mutable_color()->set_r(0.2f * i);
    m->mutable_color()->set_g(0.8f);
    m->mutable_color()->set_b(0.3f);
    m->mutable_color()->set_a(0.9f);
  }
  return arr;
}

std::string MakeLog(double t, int i) {
  const int levels[] = {1, 2, 3, 4};
  const char* names[] = {"nav", "perception", "control", "system"};
  const auto now = std::chrono::system_clock::now().time_since_epoch();
  const auto ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
  return std::string("{\"timestamp\":{\"sec\":") +
         std::to_string(ns / 1000000000LL) + ",\"nsec\":" +
         std::to_string(ns % 1000000000LL) + "},\"level\":" +
         std::to_string(levels[i % 4]) + ",\"message\":\"home_bot tick=" +
         std::to_string(i) + " t=" + std::to_string(t) + "\",\"name\":\"" +
         names[i % 4] +
         "\",\"file\":\"18_tutorial_robot_all_data_sim.cpp\",\"line\":1}";
}

}  // namespace

int main(int argc, char** argv) {
  const double rate_hz = ParseRate(argc, argv, 10.0);
  const std::string urdf = ReadFile(ResolveUrdfPath());

  if (!autolink::Init(argv[0])) return 1;
  auto node = autolink::CreateNode("/autoviz/robot_all");

  auto w_rgb = MakeWriter<automsgs::msgs::sensor_msgs::Image>(
      node, "/camera/image_raw");
  auto w_depth = MakeWriter<automsgs::msgs::sensor_msgs::Image>(
      node, "/camera/depth/image_raw");
  auto w_scan =
      MakeWriter<automsgs::msgs::sensor_msgs::LaserScan>(node, "/scan");
  auto w_cloud =
      MakeWriter<automsgs::msgs::sensor_msgs::PointCloud2>(node, "/points");
  auto w_imu =
      MakeWriter<automsgs::msgs::sensor_msgs::Imu>(node, "/imu/data");
  auto w_gps =
      MakeWriter<automsgs::msgs::sensor_msgs::NavSatFix>(node, "/gps/fix");
  auto w_batt = MakeWriter<automsgs::msgs::sensor_msgs::BatteryState>(
      node, "/battery_state");
  auto w_us = MakeWriter<automsgs::msgs::sensor_msgs::Range>(
      node, "/ultrasonic/distance");
  auto w_js = MakeWriter<automsgs::msgs::sensor_msgs::JointState>(
      node, "/joint_states");
  auto w_wrench = MakeWriter<automsgs::msgs::geometry_msgs::WrenchStamped>(
      node, "/wrench");

  auto w_odom =
      MakeWriter<automsgs::msgs::nav_msgs::Odometry>(node, "/odom");
  auto w_tf = MakeWriter<automsgs::msgs::tf2_msgs::TFMessage>(node, "/tf");
  auto w_map =
      MakeWriter<automsgs::msgs::map_msgs::OccupancyGrid>(node, "/map");
  auto w_amcl =
      MakeWriter<automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped>(
          node, "/amcl_pose");
  auto w_urdf =
      MakeWriter<automsgs::msgs::std_msgs::String>(node, "/robot_description");

  auto w_cmd =
      MakeWriter<automsgs::msgs::geometry_msgs::Twist>(node, "/cmd_vel");
  auto w_plan = MakeWriter<automsgs::msgs::nav_msgs::Path>(node, "/plan");
  auto w_local =
      MakeWriter<automsgs::msgs::nav_msgs::Path>(node, "/local_plan");
  auto w_navst = MakeWriter<automsgs::msgs::action_msgs::GoalStatusArray>(
      node, "/move_base/status");
  auto w_wp = MakeWriter<automsgs::msgs::geometry_msgs::PoseStamped>(
      node, "/waypoint");

  auto w_det = MakeWriter<automsgs::msgs::vision_msgs::Detection3DArray>(
      node, "/detected_objects");
  auto w_people = MakeWriter<automsgs::msgs::vision_msgs::Detection3DArray>(
      node, "/people");
  auto w_sem = MakeWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
      node, "/semantic_map");

  auto w_joy = MakeWriter<automsgs::msgs::sensor_msgs::Joy>(node, "/joy");
  auto w_asr =
      MakeWriter<automsgs::msgs::std_msgs::String>(node, "/speech_recognition");
  auto w_tts = MakeWriter<automsgs::msgs::std_msgs::String>(node, "/tts");
  auto w_grip = MakeWriter<automsgs::msgs::std_msgs::Float64>(
      node, "/gripper/command");

  auto w_diag = MakeWriter<automsgs::msgs::diagnostic_msgs::DiagnosticArray>(
      node, "/diagnostics");
  auto w_rstate =
      MakeWriter<automsgs::msgs::std_msgs::String>(node, "/robot_state");
  auto w_rosout = MakeRawWriter(node, "/rosout", "foxglove.Log");

  auto w_task =
      MakeWriter<automsgs::msgs::std_msgs::String>(node, "/task_status");
  auto w_bt = MakeWriter<automsgs::msgs::std_msgs::String>(
      node, "/behavior_tree/status");

  auto w_dbg_img =
      MakeWriter<automsgs::msgs::sensor_msgs::Image>(node, "/debug_image");
  auto w_cost = MakeWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
      node, "/costmap_debug");
  auto w_path_dbg =
      MakeWriter<automsgs::msgs::visualization_msgs::MarkerArray>(
          node, "/path_debug");
  auto w_tf_dbg = MakeWriter<automsgs::msgs::visualization_msgs::MarkerArray>(
      node, "/tf_debug");

  auto desc = std::make_shared<automsgs::msgs::std_msgs::String>();
  desc->set_data(urdf);

  autolink::Rate rate(rate_hz);
  std::cout << "robot_all @ " << rate_hz << " Hz — home-service topic set\n";

  const double t0 = NowSec();
  int phase = 0;
  int i = 0;
  while (autolink::OK()) {
    const double t = NowSec() - t0;
    const char* task = kTasks[static_cast<int>(t / 4) % kNumTasks];

    w_rgb->Write(MakeRgb(phase));
    w_depth->Write(MakeDepth());
    w_scan->Write(MakeScan(t));
    w_cloud->Write(MakePc2(t));
    w_imu->Write(MakeImu(t));
    w_gps->Write(MakeGps(t));
    w_batt->Write(MakeBattery(t));
    w_us->Write(MakeRange(t));
    w_js->Write(MakeJs(t));
    w_wrench->Write(MakeWrench(t));

    w_odom->Write(MakeOdom(t));
    w_tf->Write(MakeTf(t));
    w_amcl->Write(MakeAmcl(t));
    w_urdf->Write(desc);
    if (i % 20 == 0) {
      w_map->Write(MakeGrid(0));
      w_sem->Write(MakeGrid(3));
      w_cost->Write(MakeGrid(5));
    }

    w_cmd->Write(MakeCmd(t));
    w_plan->Write(MakePath(t, 30, false));
    w_local->Write(MakePath(t, 12, true));
    w_navst->Write(MakeNavStatus(t));
    w_wp->Write(MakeWaypoint(t));

    w_det->Write(MakeDetections3d(t, false));
    w_people->Write(MakeDetections3d(t, true));

    w_joy->Write(MakeJoy(t));
    auto asr = std::make_shared<automsgs::msgs::std_msgs::String>();
    asr->set_data(i % 30 < 3 ? "go to kitchen" : "");
    w_asr->Write(asr);
    auto tts = std::make_shared<automsgs::msgs::std_msgs::String>();
    tts->set_data(std::string("current task ") + task);
    w_tts->Write(tts);
    auto grip = std::make_shared<automsgs::msgs::std_msgs::Float64>();
    grip->set_data(0.5 + 0.5 * std::sin(t));
    w_grip->Write(grip);

    w_diag->Write(MakeDiagnostics(t));
    auto rstate = std::make_shared<automsgs::msgs::std_msgs::String>();
    rstate->set_data(std::string("{\"cpu_temp_c\":") +
                     std::to_string(45 + 5 * std::sin(t * 0.2)) +
                     ",\"uptime_s\":" + std::to_string(static_cast<int>(t)) +
                     ",\"task\":\"" + task + "\"}");
    w_rstate->Write(rstate);
    auto log =
        std::make_shared<autolink::message::RawMessage>(MakeLog(t, i));
    w_rosout->Write(log);

    auto task_msg = std::make_shared<automsgs::msgs::std_msgs::String>();
    task_msg->set_data(task);
    w_task->Write(task_msg);
    auto bt = std::make_shared<automsgs::msgs::std_msgs::String>();
    bt->set_data(std::string("NavigateToPose:") + task + "|CheckBattery:OK");
    w_bt->Write(bt);

    w_dbg_img->Write(MakeRgb(phase + 40, "camera"));
    w_path_dbg->Write(MakeMarkers(t, "path_debug"));
    w_tf_dbg->Write(MakeMarkers(t, "tf_debug"));

    phase = (phase + 3) % 256;
    ++i;
    rate.Sleep();
  }
  return 0;
}
