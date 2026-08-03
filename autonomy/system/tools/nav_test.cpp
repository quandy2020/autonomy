/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Offline end-to-end navigation: system::Autonomy + BT (or direct plan/follow)
 * with diff-drive odometry simulation. No ROS required.
 *
 * Example:
 *   export AUTONOMY_BT_PLUGIN_PATH=/workspace/autonomy/build/lib
 *   autonomy_nav_test \
 *     --configuration_directory=/workspace/autonomy/src/autonomy/config \
 *     --start_x=1 --start_y=1 --goal_x=5 --goal_y=5 --use_bt=true
 */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

#include "autonomy/common/gflags.hpp"
#include "autonomy/common/math/math_utils.hpp"
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/nav_msgs/odometry.pb.h>
#include "autonomy/control/controller_server.hpp"
#include "autonomy/system/autonomy.hpp"
#include "autonomy/system/options.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/geometry_msgs/transform_stamped.h"

DEFINE_double(start_x, 1.0, "Robot start x [m] in global_frame.");
DEFINE_double(start_y, 1.0, "Robot start y [m] in global_frame.");
DEFINE_double(start_yaw, 0.0, "Robot start yaw [rad].");
DEFINE_double(goal_x, 5.0, "Goal x [m] in global_frame.");
DEFINE_double(goal_y, 5.0, "Goal y [m] in global_frame.");
DEFINE_double(goal_yaw, 0.0, "Goal yaw [rad].");
DEFINE_bool(use_bt, true, "true: behavior-tree navigation; false: direct plan+follow.");
DEFINE_double(timeout_sec, 120.0, "Navigation timeout [s].");
DEFINE_double(sim_dt, 0.1, "Odometry simulation period [s].");
DEFINE_string(global_frame, "map", "Planning / TF global frame.");
DEFINE_string(base_frame, "base_link", "Robot base frame for TF.");

namespace autonomy {
namespace system {
namespace tools {
namespace {

automsgs::msgs::geometry_msgs::PoseStamped MakePose(const std::string& frame, double x,
                                             double y, double yaw) {
    automsgs::msgs::geometry_msgs::PoseStamped pose;
    pose.mutable_header()->set_frame_id(frame);
    pose.mutable_pose()->mutable_position()->set_x(x);
    pose.mutable_pose()->mutable_position()->set_y(y);
    const double half = yaw * 0.5;
    pose.mutable_pose()->mutable_orientation()->set_z(std::sin(half));
    pose.mutable_pose()->mutable_orientation()->set_w(std::cos(half));
    return pose;
}

void IntegrateDiffDrive(const automsgs::msgs::geometry_msgs::Twist& cmd, double dt,
                        double& x, double& y, double& yaw) {
    const double v = cmd.linear().x();
    const double w = cmd.angular().z();
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    yaw = autonomy::common::math::NormalizeAngle(yaw + w * dt);
}

void PublishSimState(autonomy::control::ControllerServer* controller,
                     const std::string& global_frame,
                     const std::string& base_frame, double x, double y,
                     double yaw, const automsgs::msgs::geometry_msgs::Twist& twist) {
    automsgs::msgs::planning_msgs::Odometry odom;
    odom.mutable_header()->set_frame_id(global_frame);
    odom.set_child_frame_id(base_frame);
    odom.mutable_pose()->mutable_pose()->mutable_pose()->mutable_position()->set_x(x);
    odom.mutable_pose()->mutable_pose()->mutable_pose()->mutable_position()->set_y(y);
    const double half = yaw * 0.5;
    odom.mutable_pose()->mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
        std::sin(half));
    odom.mutable_pose()->mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
        std::cos(half));
    *odom.mutable_twist()->mutable_twist() = twist;
    controller->UpdateOdometry(odom);

    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = global_frame;
    tf.child_frame_id = base_frame;
    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.rotation.z = std::sin(half);
    tf.transform.rotation.w = std::cos(half);
    autonomy::transform::Buffer::Instance()->setTransform(
        tf, "autonomy_nav_test", false);
}

}  // namespace

int RunNavTest(int argc, char** argv) {
    (void)argc;
    (void)argv;

    if (autonomy::common::FLAGS_configuration_directory.empty()) {
        LOG(ERROR) << "Set --configuration_directory to the autonomy config root "
                      "(e.g. src/autonomy/config).";
        return EXIT_FAILURE;
    }

    const auto options = autonomy::system::CreateOptions(
        autonomy::common::FLAGS_configuration_directory,
        autonomy::common::FLAGS_configuration_basename.empty()
            ? "autonomy.lua"
            : autonomy::common::FLAGS_configuration_basename);

    auto autonomy = autonomy::system::CreateAutonomy(options);
    autonomy->Start();

    autonomy::system::RuntimeOptions runtime;
    runtime.config_directory = autonomy::common::FLAGS_configuration_directory;
    runtime.enable_bt_tasks = true;
    runtime.use_bt_navigation = FLAGS_use_bt;
    runtime.global_frame = FLAGS_global_frame;
    autonomy->Configure(runtime);

    if (!autonomy->IsReady()) {
        LOG(ERROR) << "Autonomy not ready (navigator/BT configure failed). "
                      "Check AUTONOMY_BT_PLUGIN_PATH and config/navigator/navigator.lua.";
        return EXIT_FAILURE;
    }

    auto* controller = autonomy->GetController();
    if (!controller) {
        LOG(ERROR) << "ControllerServer not available.";
        return EXIT_FAILURE;
    }

    double x = FLAGS_start_x;
    double y = FLAGS_start_y;
    double yaw = FLAGS_start_yaw;
    automsgs::msgs::geometry_msgs::Twist zero;
    PublishSimState(controller, FLAGS_global_frame, FLAGS_base_frame, x, y, yaw,
                    zero);

    const auto goal =
        MakePose(FLAGS_global_frame, FLAGS_goal_x, FLAGS_goal_y, FLAGS_goal_yaw);

    LOG(INFO) << "Navigate (" << FLAGS_start_x << ", " << FLAGS_start_y << ") -> ("
              << FLAGS_goal_x << ", " << FLAGS_goal_y << ") bt="
              << (FLAGS_use_bt ? "on" : "off");

    std::atomic<bool> stop_sim{false};
    std::thread sim([&]() {
        const auto period = std::chrono::duration<double>(FLAGS_sim_dt);
        while (!stop_sim.load()) {
            const auto cmd = autonomy->GetLastControlCommand().twist();
            IntegrateDiffDrive(cmd, FLAGS_sim_dt, x, y, yaw);
            PublishSimState(controller, FLAGS_global_frame, FLAGS_base_frame, x,
                            y, yaw, cmd);
            std::this_thread::sleep_for(
                std::chrono::duration_cast<std::chrono::milliseconds>(period));
        }
    });

    const bool ok =
        autonomy->NavigateToPose(goal, []() { return false; }, []() { return true; },
                               FLAGS_timeout_sec);

    stop_sim = true;
    sim.join();
    autonomy->Shutdown();

    if (const auto path = autonomy->GetLastPath()) {
        LOG(INFO) << "Last path poses: " << path->poses_size();
    }

    if (ok) {
        LOG(INFO) << "Navigation succeeded.";
        return EXIT_SUCCESS;
    }
    LOG(ERROR) << "Navigation failed.";
    return EXIT_FAILURE;
}

}  // namespace tools
}  // namespace system
}  // namespace autonomy

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return autonomy::system::tools::RunNavTest(argc, argv);
}
