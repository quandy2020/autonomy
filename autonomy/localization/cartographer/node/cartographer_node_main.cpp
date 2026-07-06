/*
 * Copyright 2016 The Cartographer Authors
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

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/localization/cartographer/node/cartographer_node_runner.hpp"

DEFINE_string(configuration_directory, "",
              "Directory in which configuration files are searched.");
DEFINE_string(configuration_basename, "",
              "Basename of the configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, load SLAM state from this .pbstream file.");
DEFINE_bool(load_frozen_state, true,
            "Load saved state as frozen trajectories.");
DEFINE_bool(start_trajectory_with_default_topics, true,
            "Start the first trajectory with default sensor topics.");
DEFINE_string(save_state_filename, "",
              "If non-empty, serialize state to this file on shutdown.");

namespace {

void SigintHandler(int /*sig*/) { autolink::AsyncShutdown(); }

}  // namespace

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed.";
        return EXIT_FAILURE;
    }

    signal(SIGINT, SigintHandler);
    signal(SIGTERM, SigintHandler);

    autonomy::localization::cartographer::node::CartographerNodeFlags flags;
    flags.configuration_directory = FLAGS_configuration_directory;
    flags.configuration_basename = FLAGS_configuration_basename;
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
