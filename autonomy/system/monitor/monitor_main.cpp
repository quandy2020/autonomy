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
#include "autonomy/common/gflags.hpp"
#include "autonomy/system/logging/event_bundler.hpp"
#include "autonomy/system/monitor/monitor_options.hpp"
#include "autonomy/system/monitor/monitor_registry.hpp"
#include "autonomy/system/monitor/restart_request_watcher.hpp"
#include "autonomy/system/safety/safety_latch.hpp"

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
    google::ParseCommandLineFlags(&argc, &argv, false);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }

    const std::string conf = autonomy::common::FLAGS_conf.empty()
                                 ? std::string("monitor.pb.txt")
                                 : autonomy::common::FLAGS_conf;
    auto opts = autonomy::system::monitor::LoadMonitorOptions(conf);
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
    autonomy::system::logging::EventBundler bundler;
    std::thread collector([&]() {
        const auto interval =
            autonomy::system::monitor::CollectInterval(registry->options());
        while (collecting.load()) {
            registry->CollectAll();
            registry->PublishSnapshotIfConfigured();
            autonomy::system::monitor::PollRestartModuleRequest();
            const auto snap = registry->Snapshot();
            if (snap.emergency_stop_latched ||
                autonomy::system::safety::SafetyLatch{}.IsLatched()) {
                bundler.MaybeBundle("emergency_stop", snap);
            }
            if (snap.mrm_active) {
                bundler.MaybeBundle("mrm_active", snap);
            }
            if (snap.hazard_level ==
                autonomy::system::monitor::HazardLevel::kError) {
                bundler.MaybeBundle("hazard_error", snap);
            }
            for (const auto& p : snap.processes) {
                if (!p.alive) {
                    bundler.MaybeBundle("critical_process_down", snap);
                    break;
                }
            }
            std::this_thread::sleep_for(interval);
        }
    });

    LOG(INFO) << "monitor_main running"
              << (opts.enable_prometheus
                      ? (" prometheus=" + opts.prometheus_bind_address)
                      : "")
              << " (health snapshot is the sole GetHealth source for Bridge)";

    autolink::WaitForShutdown();

    collecting.store(false);
    if (collector.joinable())
        collector.join();
    registry->Stop();
    return EXIT_SUCCESS;
}
