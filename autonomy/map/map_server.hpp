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

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/map/proto/map_options.pb.h"

namespace autonomy {
namespace map {

/**
 * @class MapServer
 * @brief Loads, caches, and publishes static OccupancyGrid maps (Nav2 map_server style).
 *
 * Map data is loaded from a YAML map file or injected at runtime. Consumers receive
 * updates through MapPublishCallback; periodic publishing is optional via publish_frequency.
 */
class MapServer
{
public:
    /** Callback invoked when a map is published (once or periodically). */
    using MapPublishCallback = std::function<void(
        const commsgs::map_msgs::OccupancyGrid::SharedPtr& map)>;

    AUTONOMY_SMART_PTR_DEFINITIONS(MapServer)

    /**
     * @brief Construct MapServer with configuration options.
     * @param options Map server options (file path, topic, frame, publish rate).
     * @param node_name Optional node name for logging; defaults to "map_server".
     */
    MapServer(const proto::MapOptions& options,
              const std::string& node_name = "");

    ~MapServer();

    MapServer(const MapServer&) = delete;
    MapServer& operator=(const MapServer&) = delete;

    /** @brief Load map (if configured), publish once, and start periodic publish thread. */
    void Start();

    /** @brief Stop publish thread and release cached map data. */
    void Shutdown();

    /** @brief True after Start() until Shutdown(). */
    bool IsRunning() const {
        return running_.load();
    }

    /** @brief True if a static map is loaded or injected. */
    bool HasStaticMap() const;

    /**
     * @brief Returns the cached static map (shared ownership with MapServer).
     */
    commsgs::map_msgs::OccupancyGrid::SharedPtr GetStaticMapShared() const;

    /**
     * @brief Copies the static map into @p static_map (uses cache when available).
     * @return True on success.
     */
    bool GetRawStaticMap(commsgs::map_msgs::OccupancyGrid& static_map) const;

    /**
     * @brief Reloads the map from the configured YAML file path.
     * @return True on success.
     */
    bool ReloadMap();

    /**
     * @brief Injects an external map (e.g. from SLAM), replacing the file-based cache.
     * @return True if the message is valid and accepted.
     */
    bool SetStaticMap(const commsgs::map_msgs::OccupancyGrid& map);

    /**
     * @brief Publishes the current map once via MapPublishCallback (refreshes header stamp).
     * @return True if a map is available to publish.
     */
    bool PublishMap();

    /** @brief Registers the callback used by PublishMap() and the publish thread. */
    void SetMapPublishCallback(MapPublishCallback callback);

    /** @brief Returns the configuration options passed at construction. */
    const proto::MapOptions& GetOptions() const {
        return options_;
    }

    /** @brief Configured map file path as in MapOptions (may be relative). */
    std::string GetStaticMapFile() const;

    /** @brief Fully resolved path to the map YAML file on disk. */
    std::string GetResolvedMapFilePath() const;

    /** @brief Topic name used for map publication (default: "map"). */
    std::string GetMapTopic() const;

    /** @brief Frame ID applied to the map header (default: "map"). */
    std::string GetFrameId() const;

    /** @brief Logical map name from configuration. */
    std::string GetStaticMapName() const {
        return static_map_name_;
    }

    /** @brief Node name used for logging. */
    std::string GetNodeName() const {
        return node_name_;
    }

private:
    /** @brief Resolves map_file to an absolute or data-directory-relative path. */
    std::string resolveMapFilePath() const;

    /** @brief Sets frame_id and stamp on the map header from options. */
    void applyMapHeader(commsgs::map_msgs::OccupancyGrid& map) const;

    /** @brief Loads map from file under map_mutex_. */
    bool loadMapFromFileLocked();

    /** @brief Loads and caches map from @p map_file_path. */
    bool loadMapFromFile(const std::string& map_file_path);

    /** @brief Background loop for periodic PublishMap() when publish_frequency > 0. */
    void publishLoop();

    std::string static_map_name_;
    std::string node_name_;
    proto::MapOptions options_;

    commsgs::map_msgs::OccupancyGrid::SharedPtr static_map_msg_{nullptr};
    MapPublishCallback map_publish_callback_;

    std::atomic<bool> running_{false};
    mutable std::mutex map_mutex_;
    std::thread publish_thread_;
};

}  // namespace map
}  // namespace autonomy
