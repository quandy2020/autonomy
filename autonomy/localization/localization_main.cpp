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

#include <csignal>
#include <cstdlib>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/gflags.hpp"
#include "autonomy/localization/cartographer/node/cartographer_node_runner.hpp"

namespace autonomy {
namespace localization {
namespace {

constexpr char kDefaultConfigBasename[] =
    "localization/cartographer/backpack_2d.lua";

void SigintHandler(int /*sig*/) { autolink::AsyncShutdown(); }

std::string ConfigurationDirectory() {
    if (!common::FLAGS_configuration_directory.empty()) {
        return common::FLAGS_configuration_directory;
    }
    return "config";
}

std::string ConfigurationBasename() {
    if (!common::FLAGS_configuration_basename.empty()) {
        return common::FLAGS_configuration_basename;
    }
    return std::string(kDefaultConfigBasename);
}

}  // namespace
}  // namespace localization
}  // namespace autonomy

DEFINE_string(load_state_filename, "",
              "If non-empty, load SLAM state from this .pbstream file.");
DEFINE_bool(load_frozen_state, true,
            "Load saved state as frozen trajectories.");
DEFINE_bool(start_trajectory_with_default_topics, true,
            "Start the first trajectory with default sensor topics.");
DEFINE_string(save_state_filename, "",
              "If non-empty, serialize state to this file on shutdown.");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed.";
        return EXIT_FAILURE;
    }

    signal(SIGINT, autonomy::localization::SigintHandler);
    signal(SIGTERM, autonomy::localization::SigintHandler);

    autonomy::localization::cartographer::node::CartographerNodeFlags flags;
    flags.configuration_directory =
        autonomy::localization::ConfigurationDirectory();
    flags.configuration_basename =
        autonomy::localization::ConfigurationBasename();
    flags.load_state_filename = FLAGS_load_state_filename;
    flags.load_frozen_state = FLAGS_load_frozen_state;
    flags.start_trajectory_with_default_topics =
        FLAGS_start_trajectory_with_default_topics;
    flags.save_state_filename = FLAGS_save_state_filename;

    const int exit_code =
        autonomy::localization::cartographer::node::RunCartographerNode(flags);

    autolink::Clear();
    google::ShutdownGoogleLogging();
    return exit_code;
}
