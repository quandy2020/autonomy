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

#include "autonomy/map/map_server.hpp"

#include <chrono>
#include <cmath>

#include "autonomy/common/logging.hpp"
#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autonomy/map/constants.hpp"
#include "autonomy/map/costmap_2d/map_io.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"
#include "autonomy/map/costmap_2d/utils/validate_messages.hpp"
#include "autonomy/map/utils/data_loader_utils.hpp"

namespace autonomy {
namespace map {
namespace {

bool IsAbsolutePath(const std::string& path) {
    return !path.empty() && path.front() == '/';
}

std::string DefaultMapTopic() {
    return "map";
}

std::string DefaultFrameId() {
    return "map";
}

}  // namespace

MapServer::MapServer(const proto::MapOptions& options,
                     const std::string& node_name)
    : options_{options} {
    node_name_ = node_name.empty() ? kMapServerNodeName : node_name;

    if (options_.map_topic().empty()) {
        options_.set_map_topic(DefaultMapTopic());
    }

    if (!options_.map_file().empty()) {
        static_map_name_ =
            options_.map_name().empty() ? "map" : options_.map_name();
        AINFO << "MapServer[" << node_name_ << "]: static map map_file="
              << options_.map_file()
              << " resolved=" << resolveMapFilePath()
              << " map_name=" << static_map_name_;
    } else {
        AWARN << "MapServer[" << node_name_ << "]: map_file is not configured";
    }

    AINFO << "MapServer[" << node_name_ << "]: topic=" << options_.map_topic()
          << " frame_id="
          << (options_.frame_id().empty() ? DefaultFrameId()
                                          : options_.frame_id())
          << " publish_frequency=" << options_.publish_frequency() << " Hz";
}

MapServer::~MapServer() {
    if (running_.load()) {
        Shutdown();
    }
    AINFO << "MapServer[" << node_name_ << "] destroyed";
}

void MapServer::applyMapHeader(
    automsgs::msgs::map_msgs::OccupancyGrid& map) const {
    if (!options_.frame_id().empty()) {
        map.mutable_header()->set_frame_id(options_.frame_id());
    } else if (map.header().frame_id().empty()) {
        map.mutable_header()->set_frame_id(DefaultFrameId());
    }
    *map.mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
}

std::string MapServer::resolveMapFilePath() const {
    if (options_.map_file().empty()) {
        return "";
    }
    if (IsAbsolutePath(options_.map_file())) {
        return options_.map_file();
    }
    return utils::GetMapDataFilesDirectory() + options_.map_file();
}

bool MapServer::loadMapFromFile(const std::string& map_file_path) {
    if (map_file_path.empty()) {
        AWARN << "MapServer[" << node_name_ << "]: empty map file path";
        return false;
    }

    auto map_msg = std::make_shared<automsgs::msgs::map_msgs::OccupancyGrid>();
    const auto status =
        costmap_2d::loadMapFromYaml(map_file_path, *map_msg);
    if (status != costmap_2d::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
        AERROR << "MapServer[" << node_name_
               << "]: failed to load map from " << map_file_path
               << " (status=" << static_cast<int>(status) << ")";
        return false;
    }

    if (!costmap_2d::utils::validateMsg(*map_msg)) {
        AERROR << node_name_ << ": loaded map is malformed: " << map_file_path;
        return false;
    }

    applyMapHeader(*map_msg);

    std::lock_guard<std::mutex> lock(map_mutex_);
    static_map_msg_ = std::move(map_msg);

    size_t occupied = 0;
    size_t unknown = 0;
    for (int16_t cell : static_map_msg_->data()) {
        if (cell == costmap_2d::utils::OCC_GRID_OCCUPIED) {
            ++occupied;
        } else if (cell < 0) {
            ++unknown;
        }
    }
    AINFO << "MapServer[" << node_name_ << "]: loaded map "
          << static_map_msg_->info().width()<< "x"
          << static_map_msg_->info().height()<< " @ "
          << static_map_msg_->info().resolution()<< " m/cell, frame="
          << static_map_msg_->header().frame_id()<< " occ=" << occupied
          << " unknown=" << unknown;
    return true;
}

bool MapServer::loadMapFromFileLocked() {
    return loadMapFromFile(resolveMapFilePath());
}

void MapServer::SetMapPublishCallback(MapPublishCallback callback) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map_publish_callback_ = std::move(callback);
}

bool MapServer::HasStaticMap() const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return static_map_msg_ != nullptr;
}

std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid> MapServer::GetStaticMapShared()
    const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    return static_map_msg_;
}

bool MapServer::GetRawStaticMap(
    automsgs::msgs::map_msgs::OccupancyGrid& static_map) const {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (static_map_msg_) {
        static_map = *static_map_msg_;
        return true;
    }

    const std::string map_file = resolveMapFilePath();
    if (map_file.empty()) {
        AWARN << "MapServer[" << node_name_ << "]: no map file configured";
        return false;
    }

    const auto status = costmap_2d::loadMapFromYaml(map_file, static_map);
    if (status != costmap_2d::LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
        AERROR << "MapServer[" << node_name_
               << "]: failed to load map from " << map_file;
        return false;
    }

    applyMapHeader(static_map);
    return true;
}

bool MapServer::SetStaticMap(const automsgs::msgs::map_msgs::OccupancyGrid& map) {
    if (!costmap_2d::utils::validateMsg(map)) {
        AERROR << "MapServer[" << node_name_ << "]: rejected invalid map";
        return false;
    }

    auto map_msg = std::make_shared<automsgs::msgs::map_msgs::OccupancyGrid>(map);
    applyMapHeader(*map_msg);

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        static_map_msg_ = std::move(map_msg);
    }

    AINFO << "MapServer[" << node_name_ << "]: static map set externally ("
          << static_map_msg_->info().width()<< "x"
          << static_map_msg_->info().height()<< ")";
    return true;
}

bool MapServer::ReloadMap() {
    if (options_.map_file().empty()) {
        AWARN << "MapServer[" << node_name_
              << "]: cannot reload without map_file";
        return false;
    }
    return loadMapFromFileLocked();
}

bool MapServer::PublishMap() {
    std::shared_ptr<automsgs::msgs::map_msgs::OccupancyGrid> map_copy;
    MapPublishCallback callback;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!static_map_msg_) {
            AWARN << "MapServer[" << node_name_
                  << "]: no map to publish on topic " << options_.map_topic();
            return false;
        }
        map_copy = static_map_msg_;
        *map_copy->mutable_header()->mutable_stamp() = automsgs::msgs::builtin_interfaces::TimeNow();
        callback = map_publish_callback_;
    }

    if (callback) {
        callback(map_copy);
        return true;
    }

    ADEBUG << "MapServer[" << node_name_ << "]: map ready on topic "
           << options_.map_topic()
           << " (no MapPublishCallback registered)";
    return true;
}

void MapServer::publishLoop() {
    const double hz = options_.publish_frequency();
    if (hz <= 0.0 || !std::isfinite(hz)) {
        return;
    }

    const auto period = std::chrono::duration<double>(1.0 / hz);
    while (running_.load()) {
        PublishMap();
        std::this_thread::sleep_for(
            std::chrono::duration_cast<std::chrono::milliseconds>(period));
    }
}

void MapServer::Start() {
    if (running_.load()) {
        AWARN << "MapServer[" << node_name_ << "]: already running";
        return;
    }

    running_.store(true);

    if (!options_.map_file().empty()) {
        if (!loadMapFromFileLocked()) {
            AWARN << "MapServer[" << node_name_ << "]: failed to load static map";
        }
    }

    PublishMap();

    const double hz = options_.publish_frequency();
    if (hz > 0.0 && std::isfinite(hz)) {
        publish_thread_ = std::thread(&MapServer::publishLoop, this);
        AINFO << "MapServer[" << node_name_ << "]: publishing at " << hz
              << " Hz on " << options_.map_topic();
    } else {
        AINFO << "MapServer[" << node_name_
              << "]: publish_frequency<=0, map published on Start() only";
    }
}

void MapServer::Shutdown() {
    if (!running_.load()) {
        return;
    }

    running_.store(false);

    if (publish_thread_.joinable()) {
        publish_thread_.join();
    }

    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        static_map_msg_.reset();
    }

    AINFO << "MapServer[" << node_name_ << "]: shutdown";
}

std::string MapServer::GetStaticMapFile() const {
    return options_.map_file();
}

std::string MapServer::GetResolvedMapFilePath() const {
    return resolveMapFilePath();
}

std::string MapServer::GetMapTopic() const {
    return options_.map_topic().empty() ? DefaultMapTopic()
                                      : options_.map_topic();
}

std::string MapServer::GetFrameId() const {
    return options_.frame_id().empty() ? DefaultFrameId() : options_.frame_id();
}

}  // namespace map
}  // namespace autonomy
