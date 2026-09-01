/*
 * Copyright 2026 The Openbot Authors (duyongquan)
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
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <automsgs/msgs/nav_msgs/path.pb.h>
#include <automsgs/msgs/visualization_msgs/marker_array.pb.h>
#include "autolink/node/node.hpp"
#include "autonomy/task/teleop/path_selector.hpp"
#include "autonomy/task/teleop/obstacle_grid.hpp"

namespace autonomy::task::teleop {

/**
 * @class teleop::TeleopPathVisualizer
 * @brief Publishes selected path and path-library markers for Autoviz / RViz
 *
 * Topics (base_link):
 * - /autonomy/task/teleop/path — best follow path (nav_msgs/Path)
 * - /autonomy/task/teleop/free_path_markers — library fan (LINE_STRIP markers)
 */
class TeleopPathVisualizer
{
public:
    // Callback invoked by discovery thread to refresh live preview.
    using PreviewCallback = std::function<void()>;

    /**
     * @brief Destructor; stops discovery publisher thread
     */
    ~TeleopPathVisualizer();

    /**
     * @brief Create writers for path and marker topics
     * @param node Autolink node
     * @param frame_id TF frame for all published geometry (base_link)
     * @param enabled When false, all publish calls are no-ops
     */
    void Configure(const std::shared_ptr<autolink::Node>& node,
                   const std::string& frame_id, bool enabled = true);

    /**
     * @brief Publish library markers and selected path in one shot
     */
    void Publish(const IntentPathSelector& selector,
                 const PathSelectionResult& path_selection,
                 const std::string& frame_id);

    /**
     * @brief Publish the MPPI follow path (yellow highlight in viz)
     */
    void PublishSelected(const PathSelectionResult& path_selection,
                         const std::string& frame_id,
                         const PathObstacleGrid* grid = nullptr);

    /**
     * @brief Clear path topic and DELETEALL free_path markers
     */
    void PublishEmpty(const std::string& frame_id);

    /**
     * @brief Short stub path so Autoviz can discover / bind the Path channel
     */
    void PublishPathStub(const std::string& frame_id);

    /**
     * @brief Unfiltered path library (no obstacle clipping)
     */
    void PublishLibrary(const IntentPathSelector& selector,
                               const std::string& frame_id);

    /**
     * @brief Library fan clipped at costmap obstacles; best_index highlighted
     */
    void PublishClipped(const IntentPathSelector& selector,
                               const PathObstacleGrid& grid,
                               const std::string& frame_id,
                               std::optional<int> best_index = std::nullopt,
                               const PathSelectionResult* selection = nullptr);

    /**
     * @brief Path library used when discovery thread has no live preview callback
     */
    void SetLibrarySource(const IntentPathSelector* selector);

    /**
     * @brief Called by discovery thread to refresh path preview from latest joy
     */
    void SetPreviewCallback(PreviewCallback callback);

    /**
     * @brief Background thread republishing preview for late subscribers
     * @param period Publish interval
     */
    void StartDiscovery(
        std::chrono::milliseconds period = std::chrono::seconds(1));

    /**
     * @brief Join discovery publisher thread
     */
    void StopDiscovery();

private:
    /**
     * @brief Publish stub path for channel discovery
     */
    void PublishStub(const std::string& frame_id);

    /**
     * @brief Build and publish free_path_markers MarkerArray
     */
    void PublishMarkers(const IntentPathSelector& selector,
                                const PathObstacleGrid* grid,
                                const std::string& frame_id,
                                std::optional<int> best_index = std::nullopt,
                                const PathSelectionResult* selection = nullptr);

    /**
     * @brief Thread-safe write to free_path_markers topic
     */
    void WriteMarkers(
        automsgs::msgs::visualization_msgs::MarkerArray array);

    // When false, publishing is disabled.
    bool enabled_{false};
    // Default frame_id for published geometry.
    std::string frame_id_;
    // Autolink node for writers.
    std::shared_ptr<autolink::Node> node_;
    // Fallback library when preview callback is not set.
    const IntentPathSelector* preview_selector_{nullptr};
    // Live preview callback from TeleopMppiAssist.
    PreviewCallback preview_callback_;
    // Writer for /autonomy/task/teleop/path.
    std::shared_ptr<autolink::Writer<automsgs::msgs::nav_msgs::Path>>
        selected_path_writer_;
    // Writer for /autonomy/task/teleop/free_path_markers.
    std::shared_ptr<
        autolink::Writer<automsgs::msgs::visualization_msgs::MarkerArray>>
        free_path_markers_writer_;

    // Background discovery republish thread.
    std::thread discovery_thread_;
    // Flag to stop discovery thread.
    std::atomic<bool> discovery_running_{false};
    // Discovery publish period.
    std::chrono::milliseconds discovery_period_{66};
    // Serialize concurrent publishes.
    std::mutex publish_mutex_;
    // Last marker count (for DELETE trailing markers).
    int last_free_path_marker_count_{0};
};

}  // namespace autonomy::task::teleop
