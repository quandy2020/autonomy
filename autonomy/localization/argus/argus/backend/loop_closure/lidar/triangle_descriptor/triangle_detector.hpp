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

/**
 * @file triangle_detector.hpp
 * @brief BTC STD place recognition as LoopDetectorBase (Voxel-SLAM port).
 */

#ifndef ARGUS_MATCHING_TRIANGLE_DETECTOR_HPP_
#define ARGUS_MATCHING_TRIANGLE_DETECTOR_HPP_

#include <mutex>
#include <unordered_map>
#include <utility>
#include <vector>

#include "backend/loop_closure/lidar/triangle_descriptor/triangle_types.hpp"
#include "backend/loop_closure/loop_types.hpp"
#include "common/transform/rigid_transform.hpp"
#include "sensor/point_types.hpp"

namespace argus {
namespace vxslam {
namespace loop_closure {
namespace triangle {

/**
 * @brief Triangle STD database + SearchLoop (Voxel-SLAM STDescManager).
 *
 * Pipeline: voxel plane extract → binary corners → STD hash →
 * candidate_selector → candidate_verify (triangle vote + plane ICP).
 */
class TriangleDetector : public LoopDetectorBase {
 public:
  ARGUS_SHARED_PTR_DEFINITIONS(TriangleDetector);

  explicit TriangleDetector(TriangleConfig config = {});

  std::vector<LoopCandidate> Detect(const KeyFrame& kf) override;

  /** @brief Persist / reload descriptor DB for multi-session. */
  void AddDescriptors(const std::vector<TriangleDescriptor>& stds);
  void AddDescriptors(const std::vector<TriangleDescriptor>& stds,
                      const sensor::PointCloudXYZI& plane_cloud);
  void GenerateDescriptors(const sensor::PointCloudXYZI& cloud, int frame_id,
                           std::vector<TriangleDescriptor>* stds) const;
  void GenerateDescriptors(const sensor::PointCloudXYZI& cloud, int frame_id,
                           std::vector<TriangleDescriptor>* stds,
                           sensor::PointCloudXYZI* plane_cloud) const;

  /** @brief SearchLoop result (Voxel-SLAM pair + plane DB index). */
  struct SearchLoopResult {
    int match_db_index = -1; /**< @brief Index into plane_clouds_. */
    int match_frame_id = -1; /**< @brief Keyframe / scan id. */
    double score = 0.0;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> transform = {
        Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()};
    bool ok() const { return match_db_index >= 0; }
  };

  /**
   * @brief SearchLoop: candidate_selector + candidate_verify.
   *        Does not index the query; caller AddDescriptors after multi-session
   * search.
   */
  SearchLoopResult SearchLoop(const std::vector<TriangleDescriptor>& query,
                              const sensor::PointCloudXYZI& plane_src,
                              int query_frame_id);

  const sensor::PointCloudXYZI* PlaneCloud(int db_index) const;

  const TriangleConfig& config() const { return config_; }
  int database_size() const { return static_cast<int>(frame_ids_.size()); }

  /** @brief After loading a prior session, skip near-frame rejection. */
  void DisableNearSkip() { config_.skip_near_num = -(database_size() + 10); }

 private:
  TriangleConfig config_;
  std::mutex mutex_;
  std::vector<int> frame_ids_;
  std::vector<transform::Rigid3d> poses_;
  std::vector<std::vector<TriangleDescriptor>> descriptors_per_frame_;
  std::vector<sensor::PointCloudXYZI> plane_clouds_;
  // Hash: quantized triangle sides → list of (frame, std index).
  std::unordered_map<int64_t, std::vector<std::pair<int, int>>> hash_table_;

  void ExtractPlanes(const sensor::PointCloudXYZI& cloud,
                     std::vector<TrianglePlane>* planes) const;
  void IndexFrame(int frame_id, const transform::Rigid3d& pose,
                  const std::vector<TriangleDescriptor>& stds,
                  const sensor::PointCloudXYZI& plane);
};

}  // namespace triangle
}  // namespace loop_closure
}  // namespace vxslam
}  // namespace argus

#endif  // ARGUS_MATCHING_TRIANGLE_DETECTOR_HPP_
