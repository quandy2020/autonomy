/*
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

#include <signal.h>
#include <unistd.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/version.hpp"
#include "autonomy/visualization/bridge/foxglove_bridge.hpp"
#include "autonomy/visualization/transport/auto_discovery.hpp"

#include <chrono>
#include <thread>

namespace autonomy {
namespace visualization {
namespace {

void Run() {
    // register signal handler with lambda
    signal(SIGINT, [](int) { exit(0); });
    signal(SIGTERM, [](int) { exit(0); });

    // configure FoxgloveBridge
    FoxgloveBridge::Options options;
    options.host = "0.0.0.0";
    options.port = 8765;

    // start FoxgloveBridge
    auto bridge = std::make_shared<FoxgloveBridge>(options);
    if (!bridge->Start()) {
        AERROR << "Failed to start visualization server";
        return;
    } else {
        AINFO << "Visualization server started successfully";
    }

    // wait for signal
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

}  // namespace
}  // namespace visualization
}  // namespace autonomy

int main(int argc, char** argv) {
    autolink::Init(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    if (autonomy::common::FLAGS_verbose) {
        autonomy::common::ShowVersion();
        return 0;
    }
    autonomy::visualization::Run();
    google::ShutDownCommandLineFlags();
    return 0;
}
