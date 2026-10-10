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

#include <cstdlib>

#include <CLI/CLI.hpp>
#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/common/cli_options.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"
#include "autonomy/localization/cartographer/node/occupancy_grid_node.hpp"

int main(int argc, char** argv) {
    double resolution = 0.05;
    double publish_period_sec = 1.0;
    bool include_frozen_submaps = true;
    bool include_unfrozen_submaps = true;

    CLI::App app{"Cartographer occupancy grid publisher."};
    app.add_option("--resolution", resolution,
                   "Resolution of a grid cell in the published occupancy grid.")
        ->capture_default_str();
    app.add_option("--publish_period_sec", publish_period_sec, "OccupancyGrid publishing period.")
        ->capture_default_str();
    app.add_option("--include_frozen_submaps", include_frozen_submaps,
                   "Include frozen submaps in the occupancy grid.")
        ->default_val(true);
    app.add_option("--include_unfrozen_submaps", include_unfrozen_submaps,
                   "Include unfrozen submaps in the occupancy grid.")
        ->default_val(true);

    autonomy::common::ParseOrExit(app, argc, argv);

    CHECK(include_frozen_submaps || include_unfrozen_submaps)
        << "Ignoring both frozen and unfrozen submaps makes no sense.";

    if (!autolink::Init(argv[0])) {
        LOG(ERROR) << "autolink::Init failed.";
        return EXIT_FAILURE;
    }

    autonomy::localization::cartographer::node::RegisterAutolinkShutdownHandlers();

    auto node = autolink::CreateNode("cartographer_occupancy_grid_node");
    autonomy::localization::cartographer::node::OccupancyGridNode grid_node(
        resolution, publish_period_sec, include_frozen_submaps, include_unfrozen_submaps);
    if (!grid_node.Init(node)) {
        LOG(ERROR) << "Failed to initialize occupancy grid node.";
        autolink::Clear();
        return EXIT_FAILURE;
    }

    autolink::WaitForShutdown();
    autolink::Clear();
    return EXIT_SUCCESS;
}
