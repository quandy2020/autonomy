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
 * @file preprocess.hpp
 * @brief LiDAR front-end on unified Argus point clouds (blind / downsample /
 *        optional feature extract). Hardware decode belongs in
 * sensor::FromProto.
 */

#ifndef ARGUS_FRONTEND_PREPROCESS_HPP_
#define ARGUS_FRONTEND_PREPROCESS_HPP_

#include <memory>
#include <vector>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <Eigen/Core>

#include "argus/common/macros.hpp"
#include "argus/sensor/point_types.hpp"

namespace argus {
namespace frontend {

/** @brief Per-point geometric label used by optional feature extraction. */
enum LiDARFeature {
    kNormal,
    kPossiblePlane,
    kRealPlane,
    kEdgeJump,
    kEdgePlane,
    kWire,
    kZeroPoint
};

/** @brief Neighbor direction when scoring edge jumps. */
enum Surround { kPrev, kNext };

/** @brief Edge-jump classification along a scan line. */
enum E_jump { kJumpNormal, kJumpZero, kJump180, kJumpInf, kJumpBlind };

/**
 * @brief Scratch geometry for one point during feature extraction.
 */
struct PointGeometry {
    double range = 0; /**< @brief Range from sensor origin (m). */
    double next_dist_sq = 0; /**< @brief Squared distance to next point. */
    double angle[2] = {0, 0}; /**< @brief Local angles to prev/next. */
    double intersect = 2; /**< @brief Intersection score for plane test. */
    E_jump edge_jump[2] = {kJumpNormal,
                           kJumpNormal}; /**< @brief Edge jump prev/next. */
    LiDARFeature feature = kNormal; /**< @brief Final feature label. */
};

/**
 * @brief Front-end filter on `sensor::PointCloudXYZI`.
 *
 * Expects `PointType::curvature` to hold relative time in milliseconds
 * (as produced by `sensor::FromProto`).
 */
class Preprocess
{
public:
    ARGUS_SHARED_PTR_DEFINITIONS(Preprocess);

    using PointCloud2 = ::automsgs::msgs::sensor_msgs::PointCloud2;
    using PointCloud2ConstPtr = std::shared_ptr<const PointCloud2>;

    Preprocess();
    ~Preprocess();

    /**
     * @brief Filter an already-decoded Argus cloud.
     * @param cloud_in Input body-frame cloud.
     * @param[out] cloud_out Filtered result (cleared then filled).
     */
    void Process(const sensor::PointCloudXYZI& cloud_in,
                 sensor::PointCloudXYZI* cloud_out);

    /**
     * @brief Decode via `sensor::FromProto`, then filter.
     * @param msg PointCloud2 protobuf message.
     * @param[out] cloud_out Filtered Argus cloud.
     */
    void Process(const PointCloud2& msg, sensor::PointCloudXYZI* cloud_out);

    /**
     * @brief Decode + filter into a shared cloud pointer.
     * @param msg Shared PointCloud2 message.
     * @param[out] cloud_out Destination pointer (created if null).
     */
    void Process(const PointCloud2ConstPtr& msg,
                 sensor::PointCloudXYZI::Ptr& cloud_out);

    /**
     * @brief Configure feature extraction and range gating.
     * @param enable_features Enable edge/plane feature extract.
     * @param blind_radius Blind radius (m); points closer are dropped.
     * @param filter_stride Keep every N-th point after blind filter (>=1).
     */
    void Set(bool enable_features, double blind_radius, int filter_stride);

    sensor::PointCloudXYZI cloud_full; /**< @brief Full filtered cloud. */
    sensor::PointCloudXYZI cloud_corners; /**< @brief Corner / edge features. */
    sensor::PointCloudXYZI cloud_surf; /**< @brief Surf / plane features. */

    int point_filter_num = 1;     /**< @brief Keep every N-th point. */
    double blind = 0.01;          /**< @brief Blind radius (m). */
    double blind_sqr = 0.0001;    /**< @brief Cached `blind * blind`. */
    bool feature_enabled = false; /**< @brief Run ExtractFeatures when true. */

private:
    void FilterCloud(const sensor::PointCloudXYZI& cloud_in);
    void ExtractFeatures(const sensor::PointCloudXYZI& cloud_in);
    void GiveFeature(sensor::PointCloudXYZI& pl, std::vector<PointGeometry>& types);
    void MarkPlanes(sensor::PointCloudXYZI& pl, std::vector<PointGeometry>& types,
                    unsigned int head, int plsize2);
    void MarkEdgeJumps(sensor::PointCloudXYZI& pl, std::vector<PointGeometry>& types,
                       unsigned int head, int plsize2);
    void MarkSmallPlanes(std::vector<PointGeometry>& types, unsigned int head,
                         int plsize2);
    void EmitSurfAndCorn(sensor::PointCloudXYZI& pl,
                         std::vector<PointGeometry>& types, unsigned int head,
                         int plsize);
    int PlaneJudge(const sensor::PointCloudXYZI& pl,
                   std::vector<PointGeometry>& types, unsigned int i_cur,
                   unsigned int& i_nex, Eigen::Vector3d& curr_direct);
    int CollectPlaneGroup(const sensor::PointCloudXYZI& pl,
                          std::vector<PointGeometry>& types, unsigned int i_cur,
                          unsigned int& i_nex, double group_dis,
                          std::vector<double>* disarr, double* two_dis);
    bool EdgeJumpJudge(const sensor::PointCloudXYZI& pl,
                       std::vector<PointGeometry>& types, unsigned int i,
                       Surround nor_dir);

    int group_size_ = 8;
    double dis_a_ = 0.1;
    double dis_b_ = 0.0;
    double inf_bound_ = 10;
    double limit_maxmin_ = 3.24;
    double p2l_ratio_ = 225;
    double jump_up_limit_ = 0;
    double jump_down_limit_ = 0;
    double cos160_ = 0;
    double edge_a_ = 2;
    double edge_b_ = 0.1;
    double smallp_intersect_ = 0;
    double smallp_ratio_ = 1.2;
    double vx_ = 0;
    double vy_ = 0;
    double vz_ = 0;
};

}  // namespace frontend
}  // namespace argus

#endif  // ARGUS_FRONTEND_PREPROCESS_HPP_
