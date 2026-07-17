/*
 * Copyright 2026 The Openbot Authors
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

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <thread>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/system/monitor/monitor_options.hpp"
#include "autonomy/system/monitor/monitor_registry.hpp"

DEFINE_string(configuration_directory, "config",
              "Directory containing monitor.lua and other config files.");
DEFINE_string(configuration_basename, "system/monitor.lua",
              "Monitor Lua configuration (relative to configuration_directory).");

namespace autonomy::system::monitor {
namespace {

std::chrono::milliseconds CollectInterval(const MonitorOptions& opts) {
    const double sec = opts.collect_interval_sec > 0.0 ? opts.collect_interval_sec
                                                       : 1.0;
    return std::chrono::milliseconds(
        static_cast<int64_t>(sec * 1000.0));
}

}  // namespace
}  // namespace autonomy::system::monitor

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }

    auto opts = autonomy::system::monitor::LoadMonitorOptions(
        FLAGS_configuration_directory, FLAGS_configuration_basename);
    auto node =
        autolink::CreateNode("system_monitor", "/autonomy/system/monitor");
    auto registry =
        std::make_unique<autonomy::system::monitor::MonitorRegistry>(opts);
    if (node) {
        registry->AttachAutolinkNode(node);
    } else {
        LOG(WARNING) << "autolink node not created; channel/latency/MRM disabled";
    }
    registry->Start();

    std::atomic<bool> collecting{true};
    std::thread collector([&]() {
        const auto interval =
            autonomy::system::monitor::CollectInterval(registry->options());
        while (collecting.load()) {
            registry->CollectAll();
            std::this_thread::sleep_for(interval);
        }
    });

    LOG(INFO) << "monitor_main running"
              << (opts.enable_prometheus
                      ? (" prometheus=" + opts.prometheus_bind_address)
                      : "");

    autolink::WaitForShutdown();

    collecting.store(false);
    if (collector.joinable())
        collector.join();
    registry->Stop();
    return EXIT_SUCCESS;
}
