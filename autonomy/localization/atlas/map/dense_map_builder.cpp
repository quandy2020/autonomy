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

#include "autonomy/localization/atlas/map/dense_map_builder.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Geometry>

#include "autonomy/common/logging.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/publish/map_publisher.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace map {
namespace {

DenseMapBuilder::Options NormalizeOptions(DenseMapBuilder::Options options) {
    if (options.integrate_stride < 1) {
        options.integrate_stride = 1;
    }
    if (options.publish_stride < 1) {
        options.publish_stride = 1;
    }
    if (options.update_error_m < 0.0) {
        options.update_error_m = 0.01;
    }
    options.grid.cell_size = options.local.cell_size;
    options.cloud.voxel_size = options.local.cell_size;
    options.octree.voxel_size = options.local.cell_size;
    options.elevation.cell_size = options.local.cell_size;
    return options;
}

double RotationAngle(const Mat33_t& R) {
    const Eigen::AngleAxisd aa(R);
    return std::abs(aa.angle());
}

}  // namespace

DenseMapBuilder::DenseMapBuilder(system* slam, Options options)
    : slam_(slam),
      options_(NormalizeOptions(std::move(options))),
      local_maker_(options_.local),
      grid_map_(options_.grid),
      cloud_map_(options_.cloud),
      octree_map_(options_.octree),
      elevation_map_(options_.elevation) {}

DenseMapBuilder::~DenseMapBuilder() { Stop(); }

bool DenseMapBuilder::Start(const std::shared_ptr<autolink::Node>& node) {
    if (!options_.enabled) {
        AINFO << "Atlas DenseMapBuilder disabled.";
        return true;
    }
    if (!slam_ || !node) {
        AERROR << "DenseMapBuilder: missing system or node.";
        return false;
    }
    if (running_) {
        return true;
    }
    node_ = node;
    if (options_.publish_cloud) {
        cloud_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
                options_.cloud_topic);
    }
    if (options_.publish_cloud_layers) {
        cloud_ground_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
                options_.cloud_ground_topic);
        cloud_obstacles_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
                options_.cloud_obstacles_topic);
    }
    if (options_.publish_grid) {
        grid_writer_ =
            node_->CreateWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
                options_.grid_topic);
    }
    if (options_.publish_grid_prob) {
        grid_prob_writer_ =
            node_->CreateWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
                options_.grid_prob_topic);
    }
    if (options_.publish_octomap) {
        octomap_occupied_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
                options_.octomap_occupied_topic);
        octomap_ground_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
                options_.octomap_ground_topic);
        if (options_.publish_octomap_empty) {
            octomap_empty_writer_ =
                node_->CreateWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
                    options_.octomap_empty_topic);
        }
        if (options_.publish_octomap_grid) {
            octomap_grid_writer_ =
                node_->CreateWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
                    options_.octomap_grid_topic);
        }
    }
    if (options_.publish_elevation) {
        elevation_cloud_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
                options_.elevation_cloud_topic);
        elevation_map_writer_ =
            node_->CreateWriter<automsgs::msgs::map_msgs::OccupancyGrid>(
                options_.elevation_map_topic);
    }
    running_ = true;
    AINFO << "Atlas DenseMapBuilder started: cloud=" << options_.cloud_topic
          << " grid=" << options_.grid_topic
          << " cell=" << options_.local.cell_size << "m"
          << " keyframe_trigger=" << options_.keyframe_trigger
          << " normals=" << options_.local.normals_segmentation
          << " update_error=" << options_.update_error_m << "m";
    return true;
}

void DenseMapBuilder::Stop() {
    running_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    cloud_writer_.reset();
    cloud_ground_writer_.reset();
    cloud_obstacles_writer_.reset();
    grid_writer_.reset();
    grid_prob_writer_.reset();
    octomap_occupied_writer_.reset();
    octomap_ground_writer_.reset();
    octomap_empty_writer_.reset();
    octomap_grid_writer_.reset();
    elevation_cloud_writer_.reset();
    elevation_map_writer_.reset();
    node_.reset();
}

void DenseMapBuilder::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    grid_map_.Clear();
    cloud_map_.Clear();
    octree_map_.Clear();
    elevation_map_.Clear();
    cache_.clear();
    have_last_pose_ = false;
    next_id_ = 1;
    frame_count_ = 0;
    integrate_count_ = 0;
    last_keyframe_count_ = 0;
}

bool DenseMapBuilder::MotionGate(const Mat44_t& T_wc) {
    if (!have_last_pose_) {
        return true;
    }
    const Eigen::Vector3d dt =
        T_wc.block<3, 1>(0, 3) - last_integrate_pose_.block<3, 1>(0, 3);
    if (dt.norm() >= options_.min_translation_m) {
        return true;
    }
    const Mat33_t dR = last_integrate_pose_.block<3, 3>(0, 0).transpose() *
                       T_wc.block<3, 3>(0, 0);
    return RotationAngle(dR) >= options_.min_rotation_rad;
}

bool DenseMapBuilder::KeyframeGate(Mat44_t* T_wc_out,
                                   unsigned int* keyframe_id_out) {
    if (T_wc_out == nullptr || keyframe_id_out == nullptr || slam_ == nullptr) {
        return false;
    }
    const auto mp = slam_->get_map_publisher();
    if (!mp) {
        return false;
    }
    std::vector<std::shared_ptr<data::keyframe>> keyframes;
    const unsigned int count = mp->get_keyframes(keyframes);
    if (count == 0 || keyframes.empty()) {
        return false;
    }
    if (count <= last_keyframe_count_) {
        return false;
    }
    // Newest keyframe by id.
    auto newest = keyframes.front();
    for (const auto& kf : keyframes) {
        if (kf && (!newest || kf->id_ > newest->id_)) {
            newest = kf;
        }
    }
    if (!newest) {
        return false;
    }
    // Skip if already cached for this keyframe.
    if (cache_.find(static_cast<uint64_t>(newest->id_)) != cache_.end()) {
        last_keyframe_count_ = count;
        return false;
    }
    *T_wc_out = newest->get_pose_wc();
    *keyframe_id_out = newest->id_;
    last_keyframe_count_ = count;
    return true;
}

void DenseMapBuilder::TrimCacheLocked() {
    if (options_.max_cached_nodes <= 0) {
        return;
    }
    while (static_cast<int>(cache_.size()) > options_.max_cached_nodes) {
        auto oldest = cache_.begin();
        for (auto it = cache_.begin(); it != cache_.end(); ++it) {
            if (it->first < oldest->first) {
                oldest = it;
            }
        }
        cache_.erase(oldest);
    }
}

bool DenseMapBuilder::FullUpdateNeededLocked() {
    if (cache_.empty() || options_.update_error_m <= 0.0) {
        return false;
    }
    const auto mp = slam_ ? slam_->get_map_publisher() : nullptr;
    if (!mp) {
        return false;
    }
    std::vector<std::shared_ptr<data::keyframe>> keyframes;
    mp->get_keyframes(keyframes);
    std::unordered_map<unsigned int, Mat44_t> poses;
    poses.reserve(keyframes.size());
    for (const auto& kf : keyframes) {
        if (kf) {
            poses[kf->id_] = kf->get_pose_wc();
        }
    }
    const double thr2 =
        options_.update_error_m * options_.update_error_m;
    int matched = 0;
    for (const auto& kv : cache_) {
        if (kv.second.grid.keyframe_id == 0) {
            continue;
        }
        auto it = poses.find(kv.second.grid.keyframe_id);
        if (it == poses.end()) {
            continue;
        }
        ++matched;
        const double d2 =
            (it->second.block<3, 1>(0, 3) -
             kv.second.assembled_pose.block<3, 1>(0, 3))
                .squaredNorm();
        if (d2 > thr2) {
            return true;
        }
    }
    // Graph changed: had keyed nodes but none remain in current graph.
    if (!cache_.empty() && matched == 0) {
        for (const auto& kv : cache_) {
            if (kv.second.grid.keyframe_id != 0) {
                return true;
            }
        }
    }
    return false;
}

void DenseMapBuilder::SyncKeyframePosesLocked() {
    const auto mp = slam_ ? slam_->get_map_publisher() : nullptr;
    if (!mp) {
        return;
    }
    std::vector<std::shared_ptr<data::keyframe>> keyframes;
    mp->get_keyframes(keyframes);
    std::unordered_map<unsigned int, Mat44_t> poses;
    for (const auto& kf : keyframes) {
        if (kf) {
            poses[kf->id_] = kf->get_pose_wc();
        }
    }
    for (auto& kv : cache_) {
        if (kv.second.grid.keyframe_id == 0) {
            continue;
        }
        auto it = poses.find(kv.second.grid.keyframe_id);
        if (it != poses.end()) {
            kv.second.grid.T_wc_opencv = it->second;
        }
    }
}

void DenseMapBuilder::AssembleOne(const LocalGrid& local) {
    grid_map_.Integrate(local);
    cloud_map_.Integrate(local);
    if (options_.publish_octomap) {
        octree_map_.Integrate(local);
    }
    if (options_.publish_elevation) {
        elevation_map_.Integrate(local);
    }
}

void DenseMapBuilder::ReassembleLocked() {
    SyncKeyframePosesLocked();
    grid_map_.Clear();
    cloud_map_.Clear();
    octree_map_.Clear();
    elevation_map_.Clear();
    std::vector<uint64_t> ids;
    ids.reserve(cache_.size());
    for (const auto& kv : cache_) {
        ids.push_back(kv.first);
    }
    std::sort(ids.begin(), ids.end());
    for (uint64_t id : ids) {
        auto& entry = cache_[id];
        AssembleOne(entry.grid);
        entry.assembled_pose = entry.grid.T_wc_opencv;
    }
    AINFO << "Atlas DenseMapBuilder: reassembled " << cache_.size()
          << " LocalGrids after pose update.";
}

void DenseMapBuilder::Integrate(
    const cv::Mat& rgb, const cv::Mat& depth_m, double timestamp_sec,
    const std::shared_ptr<Mat44_t>& cam_pose_wc) {
    if (!running_ || !options_.enabled || !slam_ || !cam_pose_wc) {
        return;
    }

    uint64_t id = 0;
    unsigned int keyframe_id = 0;
    Mat44_t T_wc = *cam_pose_wc;
    bool do_integrate = false;
    bool need_reassemble = false;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++frame_count_;

        if (FullUpdateNeededLocked()) {
            need_reassemble = true;
        }

        if (options_.keyframe_trigger) {
            Mat44_t kf_pose = Mat44_t::Identity();
            unsigned int kf_id = 0;
            if (KeyframeGate(&kf_pose, &kf_id)) {
                T_wc = kf_pose;
                keyframe_id = kf_id;
                id = static_cast<uint64_t>(kf_id);
                do_integrate = true;
                last_integrate_pose_ = T_wc;
                have_last_pose_ = true;
            }
        } else {
            if (frame_count_ % static_cast<uint64_t>(options_.integrate_stride) ==
                    0 &&
                MotionGate(*cam_pose_wc)) {
                id = next_id_++;
                T_wc = *cam_pose_wc;
                do_integrate = true;
                last_integrate_pose_ = T_wc;
                have_last_pose_ = true;
            }
        }

        if (need_reassemble) {
            ReassembleLocked();
        }
    }

    if (!do_integrate) {
        if (need_reassemble) {
            Publish(timestamp_sec);
        }
        return;
    }

    LocalGrid local = local_maker_.Create(rgb, depth_m, T_wc,
                                          slam_->get_camera(), timestamp_sec,
                                          id);
    local.keyframe_id = keyframe_id;
    if (local.empty_grid()) {
        return;
    }

    AssembleOne(local);

    uint64_t integ = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        CacheEntry entry;
        entry.grid = local;
        entry.assembled_pose = local.T_wc_opencv;
        cache_[id] = std::move(entry);
        TrimCacheLocked();
        ++integrate_count_;
        integ = integrate_count_;
    }
    if (integ % static_cast<uint64_t>(options_.publish_stride) == 0) {
        Publish(timestamp_sec);
    }
}

void DenseMapBuilder::Publish(double timestamp_sec) {
    if (cloud_writer_ && options_.publish_cloud) {
        automsgs::msgs::sensor_msgs::PointCloud2 cloud;
        if (cloud_map_.ToMessage(options_.map_frame, timestamp_sec,
                                CloudMap::Layer::kMerged, &cloud)) {
            cloud_writer_->Write(cloud);
        }
    }
    if (options_.publish_cloud_layers) {
        if (cloud_ground_writer_) {
            automsgs::msgs::sensor_msgs::PointCloud2 cloud;
            if (cloud_map_.ToMessage(options_.map_frame, timestamp_sec,
                                    CloudMap::Layer::kGround, &cloud)) {
                cloud_ground_writer_->Write(cloud);
            }
        }
        if (cloud_obstacles_writer_) {
            automsgs::msgs::sensor_msgs::PointCloud2 cloud;
            if (cloud_map_.ToMessage(options_.map_frame, timestamp_sec,
                                    CloudMap::Layer::kObstacles, &cloud)) {
                cloud_obstacles_writer_->Write(cloud);
            }
        }
    }
    if (grid_writer_ && options_.publish_grid) {
        automsgs::msgs::map_msgs::OccupancyGrid grid;
        if (grid_map_.ToMessage(options_.map_frame, timestamp_sec, &grid)) {
            grid_writer_->Write(grid);
        }
    }
    if (grid_prob_writer_ && options_.publish_grid_prob) {
        automsgs::msgs::map_msgs::OccupancyGrid grid;
        if (grid_map_.ToProbMessage(options_.map_frame, timestamp_sec, &grid)) {
            grid_prob_writer_->Write(grid);
        }
    }
    if (options_.publish_octomap) {
        if (octomap_occupied_writer_) {
            automsgs::msgs::sensor_msgs::PointCloud2 cloud;
            if (octree_map_.ToCloudMessage(options_.map_frame, timestamp_sec,
                                           OctreeMap::Layer::kOccupied,
                                           &cloud)) {
                octomap_occupied_writer_->Write(cloud);
            }
        }
        if (octomap_ground_writer_) {
            automsgs::msgs::sensor_msgs::PointCloud2 cloud;
            if (octree_map_.ToCloudMessage(options_.map_frame, timestamp_sec,
                                           OctreeMap::Layer::kGround, &cloud)) {
                octomap_ground_writer_->Write(cloud);
            }
        }
        if (octomap_empty_writer_) {
            automsgs::msgs::sensor_msgs::PointCloud2 cloud;
            if (octree_map_.ToCloudMessage(options_.map_frame, timestamp_sec,
                                           OctreeMap::Layer::kEmpty, &cloud)) {
                octomap_empty_writer_->Write(cloud);
            }
        }
        if (octomap_grid_writer_) {
            automsgs::msgs::map_msgs::OccupancyGrid grid;
            if (octree_map_.ToProjectionMessage(options_.map_frame,
                                                timestamp_sec, &grid)) {
                octomap_grid_writer_->Write(grid);
            }
        }
    }
    if (options_.publish_elevation) {
        if (elevation_cloud_writer_) {
            automsgs::msgs::sensor_msgs::PointCloud2 cloud;
            if (elevation_map_.ToCloudMessage(options_.map_frame, timestamp_sec,
                                              &cloud)) {
                elevation_cloud_writer_->Write(cloud);
            }
        }
        if (elevation_map_writer_) {
            automsgs::msgs::map_msgs::OccupancyGrid grid;
            if (elevation_map_.ToGridMessage(options_.map_frame, timestamp_sec,
                                             &grid)) {
                elevation_map_writer_->Write(grid);
            }
        }
    }
    if (integrate_count_ == static_cast<uint64_t>(options_.publish_stride) ||
        integrate_count_ % 50 == 0) {
        AINFO << "Atlas DenseMapBuilder: cloud=" << cloud_map_.size()
              << " (g=" << cloud_map_.ground_size()
              << " o=" << cloud_map_.obstacle_size() << ")"
              << " grid=" << grid_map_.width() << "x" << grid_map_.height()
              << " octree_occ=" << octree_map_.occupied_size()
              << " elev=" << elevation_map_.size()
              << " cache=" << cache_.size();
    }
}

}  // namespace map
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
