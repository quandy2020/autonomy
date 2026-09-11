/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file main.cpp
 * @brief Process entry: CLI → LoadConfig → Publisher → SensorManager →
 *        PoseFeeder → ChassisManager.
 *
 * Usage: autodriver [options] [configuration_directory] [configuration_file]
 * See: autodriver --help
 */

#include <atomic>
#include <csignal>
#include <string>

#include "options.hpp"

#include "autodriver/bridge/pose_feeder.hpp"
#include "autodriver/bridge/publisher.hpp"
#include "autodriver/config_loader.hpp"
#include "autodriver/sensor_manager.hpp"
#include "chassis/chassis_manager.hpp"
#include "autolink/common/log.hpp"
#include "autolink/init.hpp"
#include "autolink/time/duration.hpp"

namespace {
/** @brief Process run flag; cleared by HandleSignal. */
std::atomic<bool> g_running{true};

/**
 * @brief SIGINT/SIGTERM handler; requests a clean shutdown.
 */
void HandleSignal(int) { g_running = false; }

/**
 * @brief Applies non-empty CLI overrides onto a loaded Config.
 */
void ApplyCliOverrides(autodriver::Config* config,
                       const autodriver::Options& opts) {
    if (config == nullptr) {
        return;
    }
    if (opts.disable_udev) {
        config->hotplug.udev = false;
    }
}

/**
 * @brief Load config, start managers, and block until signal.
 * @return Process exit code (0 on clean stop).
 */
int Run(const autodriver::Options& opts) {
    autodriver::Config config =
        autodriver::LoadConfig(opts.config_directory, opts.config_file);
    ApplyCliOverrides(&config, opts);

    if (opts.dry_run) {
        AINFO << "dry-run: node_name=" << config.node_name
              << " sensors=" << config.sensors.size()
              << " chassis.enable=" << config.chassis.enable
              << " (not starting hardware)";
        return 0;
    }

    autodriver::bridge::Publisher publisher(config.node_name);
    if (!publisher.Initialize()) {
        AERROR << "autolink publisher failed";
        return 1;
    }
    autodriver::SensorManager manager(config);
    manager.SetSampleSink(&publisher);
    if (!manager.Initialize() || !manager.Start()) {
        AERROR << "SensorManager failed";
        return 1;
    }
    autodriver::bridge::PoseFeeder pose_feeder;
    if (!pose_feeder.Start(publisher.GetNode(), &manager, config)) {
        AERROR << "PoseFeeder failed";
        manager.Stop();
        return 1;
    }
    autodriver::chassis::ChassisManager chassis;
    if (!chassis.Start(publisher.GetNode(), config)) {
        AERROR << "ChassisManager failed";
        pose_feeder.Stop();
        manager.Stop();
        return 1;
    }
    AINFO << "autodriver running (Ctrl+C to stop)";
    while (g_running.load()) {
        autolink::Duration(100'000'000).Sleep();
    }
    AINFO << "autodriver shutting down";
    chassis.Stop();
    pose_feeder.Stop();
    manager.Stop();
    return 0;
}

}  // namespace

/**
 * @brief autodriver process entry.
 * @param[in] argc Argument count.
 * @param[in] argv Argument vector.
 * @return Process exit code.
 */
int main(int argc, char** argv) {
    autodriver::Options opts;
    const autodriver::ParseStatus status =
        autodriver::ParseCommandLine(argc, argv, &opts);
    if (status == autodriver::ParseStatus::kExitOk) {
        return 0;
    }
    if (status == autodriver::ParseStatus::kExitError) {
        return 1;
    }

    autolink::Init(argv[0]);
    AINFO << autodriver::VersionString();

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    int exit_code = 1;
    try {
        exit_code = Run(opts);
    } catch (const std::exception& ex) {
        AERROR << "autodriver failed: " << ex.what();
        exit_code = 1;
    }
    autolink::Clear();
    return exit_code;
}
