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

#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <opencv2/core/mat.hpp>

#include "autolink/autolink.hpp"
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autonomy/localization/atlas/map/cloud_map.hpp"
#include "autonomy/localization/atlas/map/elevation_map.hpp"
#include "autonomy/localization/atlas/map/local_grid.hpp"
#include "autonomy/localization/atlas/map/local_grid_maker.hpp"
#include "autonomy/localization/atlas/map/occupancy_grid_map.hpp"
#include "autonomy/localization/atlas/map/octree_map.hpp"
#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/atlas/type.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace map {

/**
 * RTAB-Map-inspired dense mapping for Atlas RGB-D:
 *   RGB-D + pose → LocalGrid (+ cache) → Cloud / Occupancy / Octree / Elevation
 *
 * Topics (defaults):
 *   /atlas/cloud_{map,ground,obstacles}
 *   /map, /atlas/grid_prob_map
 *   /atlas/octomap_{occupied,ground,empty,grid}
 *   /atlas/elevation_{cloud,map}
 */
class DenseMapBuilder {
public:
    struct Options {
        std::string map_frame = "map";
        std::string cloud_topic = "/atlas/cloud_map";
        std::string cloud_ground_topic = "/atlas/cloud_ground";
        std::string cloud_obstacles_topic = "/atlas/cloud_obstacles";
        std::string grid_topic = "/map";
        std::string grid_prob_topic = "/atlas/grid_prob_map";
        std::string octomap_occupied_topic = "/atlas/octomap_occupied";
        std::string octomap_ground_topic = "/atlas/octomap_ground";
        std::string octomap_empty_topic = "/atlas/octomap_empty";
        std::string octomap_grid_topic = "/atlas/octomap_grid";
        std::string elevation_cloud_topic = "/atlas/elevation_cloud";
        std::string elevation_map_topic = "/atlas/elevation_map";

        bool enabled = true;
        bool publish_cloud = true;
        bool publish_cloud_layers = true;
        bool publish_grid = true;
        bool publish_grid_prob = false;
        bool publish_octomap = true;
        bool publish_octomap_empty = false;
        bool publish_octomap_grid = true;
        bool publish_elevation = true;

        int integrate_stride = 2;
        int publish_stride = 5;
        /** Motion gate when keyframe_trigger=false (RGBD Linear/AngularUpdate). */
        double min_translation_m = 0.05;
        double min_rotation_rad = 0.05;
        /**
         * Only integrate when Atlas keyframe count increases (preferred).
         * Enables loop-closure reassembly via keyframe pose updates.
         */
        bool keyframe_trigger = true;
        /**
         * If any cached node pose moves more than this (m), clear assembled
         * maps and reproject all LocalGrids (GridGlobal/UpdateError).
         */
        double update_error_m = 0.01;
        /** Keep at most this many LocalGrids in cache (0 = unlimited). */
        int max_cached_nodes = 200;

        LocalGridMaker::Options local;
        OccupancyGridMap::Options grid;
        CloudMap::Options cloud;
        OctreeMap::Options octree;
        ElevationMap::Options elevation;
    };

    DenseMapBuilder(system* slam, Options options);
    ~DenseMapBuilder();

    DenseMapBuilder(const DenseMapBuilder&) = delete;
    DenseMapBuilder& operator=(const DenseMapBuilder&) = delete;

    bool Start(const std::shared_ptr<autolink::Node>& node);
    void Stop();

    void Integrate(const cv::Mat& rgb, const cv::Mat& depth_m,
                   double timestamp_sec,
                   const std::shared_ptr<Mat44_t>& cam_pose_wc);

    void Clear();

private:
    struct CacheEntry {
        LocalGrid grid;
        Mat44_t assembled_pose = Mat44_t::Identity();
    };

    bool MotionGate(const Mat44_t& T_wc);
    bool KeyframeGate(Mat44_t* T_wc_out, unsigned int* keyframe_id_out);
    bool FullUpdateNeededLocked();
    void SyncKeyframePosesLocked();
    void ReassembleLocked();
    void AssembleOne(const LocalGrid& local);
    void Publish(double timestamp_sec);
    void TrimCacheLocked();

    system* slam_ = nullptr;
    Options options_;
    std::shared_ptr<autolink::Node> node_;
    std::atomic<bool> running_{false};

    LocalGridMaker local_maker_;
    OccupancyGridMap grid_map_;
    CloudMap cloud_map_;
    OctreeMap octree_map_;
    ElevationMap elevation_map_;

    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::PointCloud2>>
        cloud_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::PointCloud2>>
        cloud_ground_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::PointCloud2>>
        cloud_obstacles_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::OccupancyGrid>>
        grid_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::OccupancyGrid>>
        grid_prob_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::PointCloud2>>
        octomap_occupied_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::PointCloud2>>
        octomap_ground_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::PointCloud2>>
        octomap_empty_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::OccupancyGrid>>
        octomap_grid_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::sensor_msgs::PointCloud2>>
        elevation_cloud_writer_;
    std::shared_ptr<autolink::Writer<automsgs::msgs::map_msgs::OccupancyGrid>>
        elevation_map_writer_;

    std::mutex mutex_;
    std::unordered_map<uint64_t, CacheEntry> cache_;
    bool have_last_pose_ = false;
    Mat44_t last_integrate_pose_ = Mat44_t::Identity();
    uint64_t next_id_ = 1;
    uint64_t frame_count_ = 0;
    uint64_t integrate_count_ = 0;
    unsigned int last_keyframe_count_ = 0;
};

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
