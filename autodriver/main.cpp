/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file main.cpp
 * @brief Process entry: LoadConfig → Publisher → SensorManager → PoseFeeder.
 *
 * Usage: autodriver [configuration_directory] [configuration_file]
 * Default config basename: autodriver_hardware.yaml
 */

#include <atomic>
#include <csignal>
#include <string>

#include "autodriver/bridge/pose_feeder.hpp"
#include "autodriver/bridge/publisher.hpp"
#include "autodriver/config_loader.hpp"
#include "autodriver/sensor_manager.hpp"
#include "autolink/common/log.hpp"
#include "autolink/init.hpp"
#include "autolink/time/duration.hpp"

namespace {
std::atomic<bool> g_running{true};
void HandleSignal(int) { g_running = false; }
}  // namespace

int main(int argc, char** argv) {
    autolink::Init(argv[0]);
    std::string configuration_directory;
    std::string configuration_file = autodriver::kDefaultConfigBasename;
    if (argc > 1) {
        configuration_directory = argv[1];
    }
    if (argc > 2) {
        configuration_file = argv[2];
    }
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    try {
        autodriver::Config config = autodriver::LoadConfig(
            configuration_directory, configuration_file);
        autodriver::bridge::Publisher publisher(config.node_name);
        if (!publisher.Initialize()) {
            AERROR << "autolink publisher failed";
            autolink::Clear();
            return 1;
        }
        autodriver::SensorManager manager(config);
        manager.SetSink(&publisher);
        if (!manager.Initialize() || !manager.Start()) {
            AERROR << "SensorManager failed";
            autolink::Clear();
            return 1;
        }
        autodriver::bridge::PoseFeeder pose_feeder;
        if (!pose_feeder.Start(publisher.node(), &manager, config)) {
            AERROR << "PoseFeeder failed";
            manager.Stop();
            autolink::Clear();
            return 1;
        }
        AINFO << "autodriver running (Ctrl+C to stop)";
        while (g_running.load()) {
            autolink::Duration(100'000'000).Sleep();
        }
        AINFO << "autodriver shutting down";
        pose_feeder.Stop();
        manager.Stop();
    } catch (const std::exception& ex) {
        AERROR << "autodriver failed: " << ex.what();
        autolink::Clear();
        return 1;
    }
    autolink::Clear();
    return 0;
}
