/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/map/grid_map/grid_map_wrapper.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"

namespace autonomy {
namespace map {
namespace grid_map {

GridMapWrapper::GridMapWrapper(const proto::GridMapOptions& options,
                               const std::string& name)
    : options_{options},
      name_{name.empty()
                ? (options.name().empty() ? "grid_map" : options.name())
                : name},
      stopped_{true},
      paused_{false} {
    // Extract layers from options
    std::vector<std::string> layers;
    if (options.layers_size() > 0) {
        for (int i = 0; i < options.layers_size(); ++i) {
            layers.push_back(options.layers(i));
        }
    } else {
        // Default layers
        layers = {"elevation"};
    }

    // Create grid map
    grid_map_ = std::make_shared<::grid_map::GridMap>(layers);

    // Set frame ID if provided
    if (!options.frame_id().empty()) {
        grid_map_->setFrameId(options.frame_id());
    }

    // Set geometry if provided
    if (options.resolution() > 0 && options.length_x() > 0 &&
        options.length_y() > 0) {
        ::grid_map::Length length(options.length_x(), options.length_y());
        ::grid_map::Position position(options.position_x(),
                                      options.position_y());
        grid_map_->setGeometry(length, options.resolution(), position);
    }

    // Set basic layers if provided
    if (options.basic_layers_size() > 0) {
        std::vector<std::string> basic_layers;
        for (int i = 0; i < options.basic_layers_size(); ++i) {
            basic_layers.push_back(options.basic_layers(i));
        }
        grid_map_->setBasicLayers(basic_layers);
    }

    LOG(INFO) << "[GridMapWrapper] GridMapWrapper initialized: " << name_;
}

GridMapWrapper::~GridMapWrapper() {
    Stop();
    LOG(INFO) << "[GridMapWrapper] GridMapWrapper destroyed: " << name_;
}

void GridMapWrapper::Start() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!stopped_) {
        LOG(WARNING) << "[GridMapWrapper] GridMap is already started: "
                     << name_;
        return;
    }

    stopped_ = false;
    paused_ = false;

    LOG(INFO) << "[GridMapWrapper] GridMap started: " << name_;
}

void GridMapWrapper::Stop() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (stopped_) {
        return;
    }

    stopped_ = true;
    paused_ = false;

    LOG(INFO) << "[GridMapWrapper] GridMap stopped: " << name_;
}

void GridMapWrapper::Pause() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (stopped_) {
        LOG(WARNING) << "[GridMapWrapper] Cannot pause stopped GridMap: "
                     << name_;
        return;
    }

    if (paused_) {
        LOG(WARNING) << "[GridMapWrapper] GridMap is already paused: " << name_;
        return;
    }

    paused_ = true;
    LOG(INFO) << "[GridMapWrapper] GridMap paused: " << name_;
}

void GridMapWrapper::Resume() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (stopped_) {
        LOG(WARNING) << "[GridMapWrapper] Cannot resume stopped GridMap: "
                     << name_;
        return;
    }

    if (!paused_) {
        LOG(WARNING) << "[GridMapWrapper] GridMap is not paused: " << name_;
        return;
    }

    paused_ = false;
    LOG(INFO) << "[GridMapWrapper] GridMap resumed: " << name_;
}

bool GridMapWrapper::loadMap(const std::string& filename) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (filename.empty()) {
        LOG(ERROR) << "[GridMapWrapper] Filename is empty.";
        return false;
    }

    ::grid_map::GridMap loaded;
    if (!::grid_map::GridMapConverter::loadFromFile(filename, loaded)) {
        LOG(ERROR) << "[GridMapWrapper] Failed to load grid map from "
                   << filename;
        return false;
    }
    *grid_map_ = std::move(loaded);
    options_.set_map_file(filename);
    LOG(INFO) << "[GridMapWrapper] Loaded grid map from " << filename;
    return true;
}

void GridMapWrapper::publishMap() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (stopped_) {
        LOG(WARNING) << "[GridMapWrapper] Cannot publish stopped GridMap: "
                     << name_;
        return;
    }

    if (!grid_map_) {
        LOG(WARNING) << "[GridMapWrapper] No grid map to publish: " << name_;
        return;
    }

    // Convert to protobuf message. Autolink writer wiring is left to callers
    // that own a Writer<GridMap>; this keeps a ready-to-send snapshot path.
    last_published_message_.Clear();
    ::grid_map::GridMapConverter::toMessage(*grid_map_, last_published_message_);
    has_published_message_ = true;
    LOG(INFO) << "[GridMapWrapper] Prepared GridMap protobuf ("
              << last_published_message_.layers_size() << " layers): " << name_;
}

proto::GridMapOptions CreateGridMapOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::GridMapOptions options;

    if (parameter_dictionary->HasKey("map_file")) {
        options.set_map_file(parameter_dictionary->GetString("map_file"));
    }

    if (parameter_dictionary->HasKey("frame_id")) {
        options.set_frame_id(parameter_dictionary->GetString("frame_id"));
    }

    if (parameter_dictionary->HasKey("name")) {
        options.set_name(parameter_dictionary->GetString("name"));
    }

    if (parameter_dictionary->HasKey("resolution")) {
        options.set_resolution(parameter_dictionary->GetDouble("resolution"));
    }

    if (parameter_dictionary->HasKey("length_x")) {
        options.set_length_x(parameter_dictionary->GetDouble("length_x"));
    }

    if (parameter_dictionary->HasKey("length_y")) {
        options.set_length_y(parameter_dictionary->GetDouble("length_y"));
    }

    if (parameter_dictionary->HasKey("position_x")) {
        options.set_position_x(parameter_dictionary->GetDouble("position_x"));
    }

    if (parameter_dictionary->HasKey("position_y")) {
        options.set_position_y(parameter_dictionary->GetDouble("position_y"));
    }

    if (parameter_dictionary->HasKey("layers")) {
        auto layers_dict = parameter_dictionary->GetDictionary("layers");
        // Assuming layers is a list/array in Lua
        // This would need to be implemented based on the LuaParameterDictionary
        // API
    }

    if (parameter_dictionary->HasKey("basic_layers")) {
        auto basic_layers_dict =
            parameter_dictionary->GetDictionary("basic_layers");
        // Similar to layers
    }

    return options;
}

}  // namespace grid_map
}  // namespace map
}  // namespace autonomy
