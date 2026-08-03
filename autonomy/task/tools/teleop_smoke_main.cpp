/*
 * Copyright 2026 The Openbot Authors
 *
 * Minimal teleop harness: TaskServer + SubmitTeleopGoal (Bridge SendTeleop 未接时用).
 */

#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autolink/time/rate.hpp"
#include "autonomy/task/proto/teleop.pb.h"
#include "autonomy/task/task_server.hpp"

DEFINE_string(config_directory, "config",
              "Config root (task/teleop_assist.lua under this path).");
DEFINE_double(linear_x, 0.25, "Commanded linear.x for VELOCITY frames (m/s).");
DEFINE_double(angular_z, 0.0, "Commanded angular.z for VELOCITY frames (rad/s).");
DEFINE_double(duration_sec, 5.0, "How long to stream VELOCITY after START.");
DEFINE_double(rate_hz, 20.0, "VELOCITY command rate (Hz).");
DEFINE_double(watchdog_timeout_sec, 1.0,
              "Teleop watchdog; must exceed 1/rate_hz.");

namespace {

::autonomy::task::proto::TeleopGoal MakeTeleopGoal(
    ::autonomy::task::proto::TeleopCommand command, double linear_x,
    double angular_z, double watchdog_sec) {
    ::autonomy::task::proto::TeleopGoal goal;
    goal.set_command(command);
    goal.set_max_linear_speed(0.5f);
    goal.set_max_angular_speed(1.0f);
    goal.set_watchdog_timeout_sec(static_cast<float>(watchdog_sec));
    goal.set_disable_collision_checks(false);
    if (command == ::autonomy::task::proto::TELEOP_CMD_VELOCITY ||
        command == ::autonomy::task::proto::TELEOP_CMD_START) {
        goal.mutable_velocity()->mutable_twist()->mutable_linear()->set_x(
            static_cast<float>(linear_x));
        goal.mutable_velocity()->mutable_twist()->mutable_angular()->set_z(
            static_cast<float>(angular_z));
    }
    return goal;
}

}  // namespace

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }

    auto server = std::make_shared<autonomy::task::TaskServer>();
    auto options = autonomy::task::TaskServer::DefaultOptions();
    options.set_config_directory(FLAGS_config_directory);
    if (!server->Configure(options)) {
        LOG(ERROR) << "TaskServer configure failed";
        return EXIT_FAILURE;
    }
    if (!server->Start()) {
        LOG(ERROR) << "TaskServer start failed";
        return EXIT_FAILURE;
    }

    const auto start = MakeTeleopGoal(
        ::autonomy::task::proto::TELEOP_CMD_START, FLAGS_linear_x,
        FLAGS_angular_z, FLAGS_watchdog_timeout_sec);
    if (!server->SubmitTeleopGoal(start)) {
        LOG(ERROR) << "SubmitTeleopGoal(START) failed";
        return EXIT_FAILURE;
    }
    LOG(INFO) << "Teleop START ok; streaming VELOCITY for " << FLAGS_duration_sec
              << " s (watch /cmd_vel with autolink_channel echo)";

    autolink::Rate rate(FLAGS_rate_hz);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(FLAGS_duration_sec);
    size_t velocity_ok = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        auto vel = MakeTeleopGoal(
            ::autonomy::task::proto::TELEOP_CMD_VELOCITY, FLAGS_linear_x,
            FLAGS_angular_z, FLAGS_watchdog_timeout_sec);
        if (server->SubmitTeleopGoal(vel)) {
            ++velocity_ok;
        }
        rate.Sleep();
    }

    const auto stop = MakeTeleopGoal(
        ::autonomy::task::proto::TELEOP_CMD_STOP, 0.0, 0.0,
        FLAGS_watchdog_timeout_sec);
    server->SubmitTeleopGoal(stop);

    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    LOG(INFO) << "Teleop STOP submitted; VELOCITY accepts=" << velocity_ok;
    LOG(INFO) << "If assist enabled: expect cmd_vel != command when obstacles "
                 "present; zero cmd when cloud stale.";

    server->Shutdown();
    return EXIT_SUCCESS;
}
