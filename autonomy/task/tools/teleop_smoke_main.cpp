/*
 * Copyright 2026 The Openbot Authors
 *
 * Minimal teleop harness: TaskServer + SubmitTeleopGoal (Bridge SendTeleop 未接时用).
 */

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autolink/time/rate.hpp"
#include <automsgs/task/teleop.pb.h>
#include "autonomy/common/cli_options.hpp"
#include "autonomy/task/task_server.hpp"

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
    CLI::App app{"autonomy.task.teleop_smoke"};
    std::string config_directory;
    double linear_x = 0.25;
    double angular_z = 0.0;
    double duration_sec = 5.0;
    double rate_hz = 20.0;
    double watchdog_timeout_sec = 1.0;

    app.add_option("--config_directory", config_directory,
                   "Task conf root (default: autonomy/task/conf).");
    app.add_option("--linear_x", linear_x,
                   "Commanded linear.x for VELOCITY frames (m/s).")
        ->capture_default_str();
    app.add_option("--angular_z", angular_z,
                   "Commanded angular.z for VELOCITY frames (rad/s).")
        ->capture_default_str();
    app.add_option("--duration_sec", duration_sec,
                   "How long to stream VELOCITY after START.")
        ->capture_default_str();
    app.add_option("--rate_hz", rate_hz, "VELOCITY command rate (Hz).")
        ->capture_default_str();
    app.add_option("--watchdog_timeout_sec", watchdog_timeout_sec,
                   "Teleop watchdog; must exceed 1/rate_hz.")
        ->capture_default_str();
    autonomy::common::ParseOrExit(app, argc, argv);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }

    auto server = std::make_shared<autonomy::task::TaskServer>();
    auto options = autonomy::task::TaskServer::DefaultOptions();
    // Empty flag must not wipe the path resolved by BtDefaults::Apply.
    if (!config_directory.empty()) {
        options.set_config_directory(config_directory);
    }
    if (!server->Configure(options)) {
        LOG(ERROR) << "TaskServer configure failed";
        return EXIT_FAILURE;
    }
    if (!server->Start()) {
        LOG(ERROR) << "TaskServer start failed";
        return EXIT_FAILURE;
    }

    const auto start = MakeTeleopGoal(
        ::autonomy::task::proto::TELEOP_CMD_START, linear_x, angular_z,
        watchdog_timeout_sec);
    if (!server->SubmitTeleopGoal(start)) {
        LOG(ERROR) << "SubmitTeleopGoal(START) failed";
        return EXIT_FAILURE;
    }
    LOG(INFO) << "Teleop START ok; streaming VELOCITY for " << duration_sec
              << " s (watch /cmd_vel with autolink_channel echo)";

    autolink::Rate rate(rate_hz);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(duration_sec);
    size_t velocity_ok = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        auto vel = MakeTeleopGoal(::autonomy::task::proto::TELEOP_CMD_VELOCITY,
                                  linear_x, angular_z, watchdog_timeout_sec);
        if (server->SubmitTeleopGoal(vel)) {
            ++velocity_ok;
        }
        rate.Sleep();
    }

    const auto stop = MakeTeleopGoal(::autonomy::task::proto::TELEOP_CMD_STOP,
                                     0.0, 0.0, watchdog_timeout_sec);
    server->SubmitTeleopGoal(stop);

    std::this_thread::sleep_for(std::chrono::milliseconds(400));

    LOG(INFO) << "Teleop STOP submitted; VELOCITY accepts=" << velocity_ok;
    LOG(INFO) << "If assist enabled: expect cmd_vel != command when obstacles "
                 "present; zero cmd when cloud stale.";

    server->Shutdown();
    return EXIT_SUCCESS;
}
