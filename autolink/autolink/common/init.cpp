/**
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autolink/common/init.hpp"

#include <libgen.h>
#include <sys/types.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <ctime>
// TODO: Uncomment when C++17 filesystem is properly configured
// #include <filesystem>
#include <memory>
#include <mutex>
#include <string>

// TODO: Uncomment when dependencies are implemented
// #include "gflags/gflags.h"
// #include "autolink/proto/clock.pb.h"
#include "autolink/common/binary.hpp"
// #include "autolink/common/file.hpp"
// #include "autolink/common/global_data.hpp"
// #include "autolink/data/data_dispatcher.hpp"
// #include "autolink/logger/async_logger.hpp"
// #include "autolink/node/node.hpp"
// #include "autolink/scheduler/scheduler.hpp"
// #include "autolink/service_discovery/topology_manager.hpp"
// #include "autolink/statistics/statistics.hpp"
// #include "autolink/sysmo/sysmo.hpp"
// #include "autolink/task/task.hpp"
// #include "autolink/time/clock.hpp"
// #include "autolink/timer/timing_wheel.hpp"
// #include "autolink/transport/transport.hpp"

namespace autolink {

namespace {

// TODO: Uncomment when dependencies are implemented
// const std::string& kClockChannel = "/clock";
// const std::string& kClockNode = "clock";

bool g_atexit_registered = false;
std::mutex g_mutex;
// TODO: Uncomment when Node is implemented
// std::unique_ptr<Node> clock_node;

// TODO: Uncomment when logger is implemented
// logger::AsyncLogger* async_logger = nullptr;

void InitLogger(const char* binary_name) {
    const char* slash = strrchr(binary_name, '/');
    if (slash) {
        ::autolink::binary::SetName(slash + 1);
    } else {
        ::autolink::binary::SetName(binary_name);
    }

    // TODO: Uncomment when glog is configured
    // google::InitGoogleLogging(binary_name);
    // google::SetLogDestination(google::ERROR, "");
    // google::SetLogDestination(google::WARNING, "");
    // google::SetLogDestination(google::FATAL, "");

    // TODO: Uncomment when async logger is implemented
    // async_logger = new ::autolink::logger::AsyncLogger(
    //     google::base::GetLogger(FLAGS_minloglevel));
    // google::base::SetLogger(FLAGS_minloglevel, async_logger);
    // async_logger->Start();
}

void StopLogger() {
    // TODO: Uncomment when logger is implemented
    // delete async_logger;
}

}  // namespace

void OnShutdown(int sig) {
    (void)sig;
    if (GetState() != STATE_SHUTDOWN) {
        SetState(STATE_SHUTTING_DOWN);
    }
}

void ExitHandle() {
    Clear();
}

bool Init(const char* binary_name, const std::string& dag_info) {
    (void)dag_info;  // Suppress unused parameter warning

    // TODO: Uncomment when C++17 filesystem is properly configured
    // const char* autolink_runtime_path = std::getenv("AUTOLINK_RUNTIME_PATH");
    // if (autolink_runtime_path != nullptr) {
    //     if (std::filesystem::is_directory(
    //             std::filesystem::status(autolink_runtime_path))) {
    //         std::filesystem::current_path(autolink_runtime_path);
    //     }
    // }

    {
        std::lock_guard<std::mutex> lg(g_mutex);
        if (GetState() != STATE_UNINITIALIZED) {
            return false;
        }
    }
    // Lock is released here, before calling scheduler

    InitLogger(binary_name);
    // TODO: Uncomment when scheduler and logger are implemented
    // auto thread = const_cast<std::thread*>(async_logger->LogThread());
    // scheduler::Instance()->SetInnerThreadAttr("async_log", thread);
    // SysMo::Instance();
    std::signal(SIGINT, OnShutdown);
    // Register exit handlers
    if (!g_atexit_registered) {
        if (std::atexit(ExitHandle) != 0) {
            // TODO: Uncomment when logging is configured
            // AERROR << "Register exit handle failed";
            return false;
        }
        // TODO: Uncomment when logging is configured
        // AINFO << "Register exit handle succ.";
        g_atexit_registered = true;
    }
    {
        std::lock_guard<std::mutex> lg(g_mutex);
        SetState(STATE_INITIALIZED);
    }

    // TODO: Uncomment when global_data and proto are implemented
    // auto global_data = GlobalData::Instance();
    // if (global_data->IsMockTimeMode()) {
    //     auto node_name = kClockNode + std::to_string(getpid());
    //     clock_node = std::unique_ptr<Node>(new Node(node_name));
    //     auto cb = [](const std::shared_ptr<const autolink::proto::Clock>&
    //     msg) {
    //         Clock::Instance()->SetNow(Time(msg->clock()));
    //     };
    //     clock_node->CreateReader<autolink::proto::Clock>(kClockChannel, cb);
    // }

    // TODO: Uncomment when statistics are implemented
    // if (dag_info != "") {
    //     std::string dump_path;
    //     if (dag_info.length() > 200) {
    //         std::string truncated = dag_info.substr(0, 200);
    //         dump_path = common::GetEnv("AUTOLINK_ENV_WORKROOT", "/autolink")
    //         +
    //                     "/dumps/" + truncated;
    //     } else {
    //         dump_path = common::GetEnv("AUTOLINK_ENV_WORKROOT", "/autolink")
    //         +
    //                     "/dumps/" + dag_info;
    //     }
    //     google::SetCommandLineOption("bvar_dump_file", dump_path.c_str());
    // } else {
    //     statistics::Statistics::Instance()->DisableChanVar();
    // }
    // google::SetCommandLineOption("bvar_dump_exclude", "*qps");
    // google::SetCommandLineOption("bvar_dump", "true");

    return true;
}

void Clear() {
    std::lock_guard<std::mutex> lg(g_mutex);
    if (GetState() == STATE_SHUTDOWN || GetState() == STATE_UNINITIALIZED) {
        return;
    }
    // TODO: Uncomment when these modules are implemented
    // SysMo::CleanUp();
    // TaskManager::CleanUp();
    // TimingWheel::CleanUp();
    // scheduler::CleanUp();
    // service_discovery::TopologyManager::CleanUp();
    // transport::Transport::CleanUp();
    StopLogger();
    SetState(STATE_SHUTDOWN);
}

}  // namespace autolink