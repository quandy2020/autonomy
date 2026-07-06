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
#include "autonomy/localization/cartographer/node/occupancy_grid_node.hpp"

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");

namespace {

void SigintHandler(int /*sig*/) { autolink::AsyncShutdown(); }

}  // namespace

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);

    CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
        << "Ignoring both frozen and unfrozen submaps makes no sense.";

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed.";
        return EXIT_FAILURE;
    }

    signal(SIGINT, SigintHandler);
    signal(SIGTERM, SigintHandler);

    auto node = autolink::CreateNode("cartographer_occupancy_grid_node");
    autonomy::localization::cartographer::node::OccupancyGridNode grid_node(
        FLAGS_resolution, FLAGS_publish_period_sec, FLAGS_include_frozen_submaps,
        FLAGS_include_unfrozen_submaps);
    if (!grid_node.Init(node)) {
        LOG(ERROR) << "Failed to initialize occupancy grid node.";
        autolink::Clear();
        return EXIT_FAILURE;
    }

    autolink::WaitForShutdown();
    autolink::Clear();
    google::ShutdownGoogleLogging();
    return EXIT_SUCCESS;
}
