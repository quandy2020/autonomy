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

#include "autonomy/localization/cartographer/node/cartographer_node_runner.hpp"

#include <cstdlib>

#include <glog/logging.h>

#include "autolink/autolink.hpp"
#include "autonomy/localization/cartographer/mapping/map_builder.hpp"
#include "autonomy/localization/cartographer/node/cartographer_node.hpp"
#include "autonomy/localization/cartographer/node/node_options.hpp"
#include "autonomy/localization/cartographer/node/node_utils.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/static_transform_publisher.hpp"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

int RunCartographerNode(const CartographerNodeFlags& flags) {
    if (flags.configuration_directory.empty()) {
        LOG(ERROR) << "configuration_directory is required.";
        return EXIT_FAILURE;
    }
    if (flags.configuration_basename.empty()) {
        LOG(ERROR) << "configuration_basename is required.";
        return EXIT_FAILURE;
    }

    transform::Buffer::Instance()->Init();

    const std::string static_tf_yaml = ResolveStaticTransformYamlPath(
        flags.configuration_directory, flags.configuration_basename);
    transform::StaticTransformPublisher static_tf_publisher;
    if (static_tf_publisher.LoadFromFile(static_tf_yaml)) {
        static_tf_publisher.ApplyToBuffer(transform::Buffer::Instance());
    }

    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) =
        LoadOptions(flags.configuration_directory, flags.configuration_basename);

    auto map_builder =
        ::cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
    CartographerNode cartographer_node(node_options, std::move(map_builder));

    auto node = autolink::CreateNode("cartographer_node");
    if (!node || !cartographer_node.Init(node)) {
        LOG(ERROR) << "Failed to initialize CartographerNode.";
        return EXIT_FAILURE;
    }

    if (static_tf_publisher.IsLoaded()) {
        static_tf_publisher.Publish(node);
    }

    if (!flags.load_state_filename.empty()) {
        cartographer_node.LoadState(flags.load_state_filename,
                                    flags.load_frozen_state);
    }

    // Drop stale sensor messages buffered when subscribers are first created.
    if (flags.start_trajectory_with_default_topics) {
        node->ClearData();
        cartographer_node.StartTrajectoryWithDefaultTopics(trajectory_options);
    }

    LOG(INFO) << "Cartographer node running.";
    autolink::WaitForShutdown();

    cartographer_node.FinishAllTrajectories();
    cartographer_node.RunFinalOptimization();

    if (!flags.save_state_filename.empty()) {
        cartographer_node.SerializeState(flags.save_state_filename, true);
    }

    return EXIT_SUCCESS;
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
