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

#include <cstdlib>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/exploration/exploration_options.hpp"
#include "autonomy/exploration/exploration_server.hpp"
#include "autonomy/system/options.hpp"
#include "autonomy/system/proto/autonomy_options.pb.h"

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed";
        return EXIT_FAILURE;
    }

    autonomy::exploration::proto::ExplorationOptions exploration_options =
        autonomy::exploration::DefaultOptions();

    const auto options = autonomy::system::CreateOptions(
        autonomy::common::FLAGS_configuration_directory,
        autonomy::common::FLAGS_configuration_basename);
    if (options.has_exploration_options()) {
        exploration_options = options.exploration_options();
    } else {
        // Fall back to dedicated exploration.lua if present.
        exploration_options = autonomy::exploration::CreateOptions(
            autonomy::common::FLAGS_configuration_directory);
        AWARN << "exploration_main: using DefaultOptions / exploration.lua "
                 "(no exploration_options in autonomy config)";
    }

    auto server = std::make_shared<autonomy::exploration::ExplorationServer>(
        exploration_options);
    server->Start();

    LOG(INFO) << "exploration_main running (ExplorationServer)";
    autolink::WaitForShutdown();

    server->Shutdown();
    return EXIT_SUCCESS;
}
