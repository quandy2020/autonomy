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

#include "argus/map/voxel_map.hpp"

#include <cmath>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "argus/common/math/so3.hpp"
#include "argus/map/voxel_body_cov.hpp"

namespace argus {
namespace map {
namespace {

VoxelLocation NearVoxelPosition(const VoxelLocation& position,
                                const float loc_xyz[3],
                                const VoxelOctoTree* current_octo) {
    VoxelLocation near_position = position;
    if (loc_xyz[0] >
        (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
        near_position.x = near_position.x + 1;
    } else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
                             current_octo->quater_length_)) {
        near_position.x = near_position.x - 1;
    }
    if (loc_xyz[1] >
        (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
        near_position.y = near_position.y + 1;
    } else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
                             current_octo->quater_length_)) {
        near_position.y = near_position.y - 1;
    }
    if (loc_xyz[2] >
        (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
        near_position.z = near_position.z + 1;
    } else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
                             current_octo->quater_length_)) {
        near_position.z = near_position.z - 1;
    }
    return near_position;
}

}  // namespace

using frontend::StatesGroup;

VoxelMapManager::VoxelMapManager(
    VoxelMapConfig& config_setting,
    std::unordered_map<VoxelLocation, VoxelOctoTree*>& voxel_map)
    : config_setting_(config_setting),
      voxel_map_(voxel_map),
      frame_ptr_(&owned_frame_) {
    current_frame_id_ = 0;
    owned_frame_.EnsureDefaultClouds();
}

void VoxelMapManager::BindFrameBuffers(LioFrameBuffers* external) {
    if (external == nullptr) {
        frame_ptr_ = &owned_frame_;
        owned_frame_.EnsureDefaultClouds();
        return;
    }
    external->EnsureDefaultClouds();
    frame_ptr_ = external;
}

void VoxelMapManager::TransformLidar(
    const Eigen::Matrix3d rot, const Eigen::Vector3d t,
    const sensor::PointCloudXYZI::Ptr& input_cloud,
    sensor::PointCloud<sensor::PointXYZI>::Ptr& trans_cloud) {
    if (!trans_cloud) {
        trans_cloud = sensor::PointCloud<sensor::PointXYZI>::make_shared();
    }
    const std::size_t n = input_cloud->size();
    trans_cloud->clear();
    trans_cloud->resize(n);
    for (std::size_t i = 0; i < n; i++) {
        const sensor::PointType& p_c = input_cloud->points[i];
        Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
        p = (rot * (extR_ * p + extT_) + t);
        sensor::PointXYZI& pi = trans_cloud->points[i];
        pi.x = static_cast<float>(p(0));
        pi.y = static_cast<float>(p(1));
        pi.z = static_cast<float>(p(2));
        pi.intensity = p_c.intensity;
    }
}

void VoxelMapManager::BuildVoxelMap() {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    float voxel_size = config_setting_.max_voxel_size_;
    float planer_threshold = config_setting_.planner_threshold_;
    int max_layer = config_setting_.max_layer_;
    int max_points_num = config_setting_.max_points_num_;
    std::vector<int> layer_init_num = config_setting_.layer_init_num_;

    auto& fb = frame();
    auto& input_points = fb.pv_list;
    input_points.clear();
    input_points.reserve(fb.feats_down_world->size());

    for (size_t i = 0; i < fb.feats_down_world->size(); i++) {
        PointWithVar pv;
        pv.point_w << fb.feats_down_world->points[i].x,
            fb.feats_down_world->points[i].y, fb.feats_down_world->points[i].z;
        Eigen::Vector3d point_this(fb.feats_down_body->points[i].x,
                                   fb.feats_down_body->points[i].y,
                                   fb.feats_down_body->points[i].z);
        Eigen::Matrix3d var;
        calcBodyCov(point_this, config_setting_.dept_err_,
                    config_setting_.beam_err_, var);
        Eigen::Matrix3d point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);
        var = (state_.rot_end * extR_) * var *
                  (state_.rot_end * extR_).transpose() +
              (-point_crossmat) * state_.cov.block<3, 3>(0, 0) *
                  (-point_crossmat).transpose() +
              state_.cov.block<3, 3>(3, 3);
        pv.var = var;
        input_points.push_back(pv);
    }

    uint plsize = static_cast<uint>(input_points.size());
    for (uint i = 0; i < plsize; i++) {
        const PointWithVar& p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point_w[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VoxelLocation position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                               (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            voxel_map_[position]->temp_points_.push_back(p_v);
            voxel_map_[position]->new_points_++;
        } else {
            VoxelOctoTree* octo_tree =
                new VoxelOctoTree(max_layer, 0, layer_init_num[0],
                                  max_points_num, planer_threshold);
            voxel_map_[position] = octo_tree;
            voxel_map_[position]->quater_length_ = voxel_size / 4;
            voxel_map_[position]->voxel_center_[0] =
                (0.5 + position.x) * voxel_size;
            voxel_map_[position]->voxel_center_[1] =
                (0.5 + position.y) * voxel_size;
            voxel_map_[position]->voxel_center_[2] =
                (0.5 + position.z) * voxel_size;
            voxel_map_[position]->temp_points_.push_back(p_v);
            voxel_map_[position]->new_points_++;
            voxel_map_[position]->layer_init_num_ = layer_init_num;
        }
    }
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) {
        iter->second->init_octo_tree();
    }
}

Eigen::Vector3f VoxelMapManager::RGBFromVoxel(
    const Eigen::Vector3d& input_point) {
    int64_t loc_xyz[3];
    for (int j = 0; j < 3; j++) {
        loc_xyz[j] = floor(input_point[j] / config_setting_.max_voxel_size_);
    }

    VoxelLocation position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                           (int64_t)loc_xyz[2]);
    int64_t ind = loc_xyz[0] + loc_xyz[1] + loc_xyz[2];
    uint k((ind + 100000) % 3);
    Eigen::Vector3f RGB((k == 0) * 255.0, (k == 1) * 255.0, (k == 2) * 255.0);
    // cout<<"RGB: "<<RGB.transpose()<<std::endl;
    return RGB;
}

void VoxelMapManager::UpdateVoxelMap(
    const std::vector<PointWithVar>& input_points) {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    float voxel_size = config_setting_.max_voxel_size_;
    float planer_threshold = config_setting_.planner_threshold_;
    int max_layer = config_setting_.max_layer_;
    int max_points_num = config_setting_.max_points_num_;
    std::vector<int> layer_init_num = config_setting_.layer_init_num_;
    uint plsize = input_points.size();
    last_touched_.clear();
    last_touched_.reserve(plsize);
    for (uint i = 0; i < plsize; i++) {
        const PointWithVar p_v = input_points[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = p_v.point_w[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VoxelLocation position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                               (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            voxel_map_[position]->UpdateOctoTree(p_v);
        } else {
            VoxelOctoTree* octo_tree =
                new VoxelOctoTree(max_layer, 0, layer_init_num[0],
                                  max_points_num, planer_threshold);
            voxel_map_[position] = octo_tree;
            voxel_map_[position]->layer_init_num_ = layer_init_num;
            voxel_map_[position]->quater_length_ = voxel_size / 4;
            voxel_map_[position]->voxel_center_[0] =
                (0.5 + position.x) * voxel_size;
            voxel_map_[position]->voxel_center_[1] =
                (0.5 + position.y) * voxel_size;
            voxel_map_[position]->voxel_center_[2] =
                (0.5 + position.z) * voxel_size;
            voxel_map_[position]->UpdateOctoTree(p_v);
        }
        last_touched_.push_back(position);
    }
}

int VoxelMapManager::EraseVoxels(const std::vector<VoxelLocation>& locations) {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    int erased = 0;
    for (const VoxelLocation& loc : locations) {
        auto it = voxel_map_.find(loc);
        if (it == voxel_map_.end()) {
            continue;
        }
        delete it->second;
        voxel_map_.erase(it);
        ++erased;
    }
    return erased;
}

void VoxelMapManager::BuildResidualListOMP(
    std::vector<PointWithVar>& pv_list, std::vector<PointToPlane>& ptpl_list) {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    double voxel_size = config_setting_.max_voxel_size_;
    ptpl_list.clear();
    const std::size_t n = pv_list.size();
    auto& residual_ptpl_scratch = frame().residual_ptpl_scratch;
    auto& residual_useful_scratch = frame().residual_useful_scratch;
    residual_ptpl_scratch.resize(n);
    residual_useful_scratch.assign(n, false);
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < static_cast<int>(n); i++) {
        PointWithVar& pv = pv_list[i];
        float loc_xyz[3];
        for (int j = 0; j < 3; j++) {
            loc_xyz[j] = pv.point_w[j] / voxel_size;
            if (loc_xyz[j] < 0) {
                loc_xyz[j] -= 1.0;
            }
        }
        VoxelLocation position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                               (int64_t)loc_xyz[2]);
        auto iter = voxel_map_.find(position);
        if (iter != voxel_map_.end()) {
            VoxelOctoTree* current_octo = iter->second;
            PointToPlane single_ptpl;
            bool is_sucess = false;
            double prob = 0;
            BuildSingleResidual(pv, current_octo, 0, is_sucess, prob,
                                single_ptpl);
            if (!is_sucess) {
                const VoxelLocation near_position =
                    NearVoxelPosition(position, loc_xyz, current_octo);
                auto iter_near = voxel_map_.find(near_position);
                if (iter_near != voxel_map_.end()) {
                    BuildSingleResidual(pv, iter_near->second, 0, is_sucess,
                                        prob, single_ptpl);
                }
            }
            // Index i is thread-private; no mutex needed.
            residual_useful_scratch[i] = is_sucess;
            if (is_sucess) {
                residual_ptpl_scratch[i] = single_ptpl;
            }
        }
    }
    std::size_t useful_n = 0;
    for (bool u : residual_useful_scratch) {
        if (u) {
            ++useful_n;
        }
    }
    ptpl_list.reserve(useful_n);
    for (size_t i = 0; i < n; i++) {
        if (residual_useful_scratch[i]) {
            ptpl_list.push_back(residual_ptpl_scratch[i]);
        }
    }
}

void VoxelMapManager::BuildSingleResidual(PointWithVar& pv,
                                          const VoxelOctoTree* current_octo,
                                          const int current_layer,
                                          bool& is_sucess, double& prob,
                                          PointToPlane& single_ptpl) {
    int max_layer = config_setting_.max_layer_;
    double sigma_num = config_setting_.sigma_num_;

    double radius_k = 3;
    Eigen::Vector3d p_w = pv.point_w;
    if (current_octo->plane_ptr_->is_plane_) {
        VoxelPlane& plane = *current_octo->plane_ptr_;
        Eigen::Vector3d p_world_to_center = p_w - plane.center_;
        float dis_to_plane =
            std::fabs(plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) +
                      plane.normal_(2) * p_w(2) + plane.d_);
        float dis_to_center =
            (plane.center_(0) - p_w(0)) * (plane.center_(0) - p_w(0)) +
            (plane.center_(1) - p_w(1)) * (plane.center_(1) - p_w(1)) +
            (plane.center_(2) - p_w(2)) * (plane.center_(2) - p_w(2));
        float range_dis =
            std::sqrt(dis_to_center - dis_to_plane * dis_to_plane);

        if (range_dis <= radius_k * plane.radius_) {
            Eigen::Matrix<double, 1, 6> J_nq;
            J_nq.block<1, 3>(0, 0) = p_w - plane.center_;
            J_nq.block<1, 3>(0, 3) = -plane.normal_;
            double sigma_l = J_nq * plane.plane_var_ * J_nq.transpose();
            sigma_l += plane.normal_.transpose() * pv.var * plane.normal_;
            if (dis_to_plane < sigma_num * std::sqrt(sigma_l)) {
                is_sucess = true;
                double this_prob =
                    1.0 / (std::sqrt(sigma_l)) *
                    exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
                if (this_prob > prob) {
                    prob = this_prob;
                    pv.normal = plane.normal_;
                    single_ptpl.body_cov_ = pv.body_var;
                    single_ptpl.point_b_ = pv.point_b;
                    single_ptpl.point_w_ = pv.point_w;
                    single_ptpl.plane_var_ = plane.plane_var_;
                    single_ptpl.normal_ = plane.normal_;
                    single_ptpl.center_ = plane.center_;
                    single_ptpl.d_ = plane.d_;
                    single_ptpl.layer_ = current_layer;
                    single_ptpl.dis_to_plane_ =
                        plane.normal_(0) * p_w(0) + plane.normal_(1) * p_w(1) +
                        plane.normal_(2) * p_w(2) + plane.d_;
                }
                return;
            } else {
                // is_sucess = false;
                return;
            }
        } else {
            // is_sucess = false;
            return;
        }
    } else {
        if (current_layer < max_layer) {
            for (size_t leafnum = 0; leafnum < 8; leafnum++) {
                if (current_octo->leaves_[leafnum] != nullptr) {
                    VoxelOctoTree* leaf_octo = current_octo->leaves_[leafnum];
                    BuildSingleResidual(pv, leaf_octo, current_layer + 1,
                                        is_sucess, prob, single_ptpl);
                }
            }
            return;
        } else {
            return;
        }
    }
}

void VoxelMapManager::GetUpdatePlane(
    const VoxelOctoTree* current_octo, int pub_max_voxel_layer,
    std::vector<VoxelPlane>& plane_list) const {
    if (current_octo->layer_ > pub_max_voxel_layer) {
        return;
    }
    if (current_octo->plane_ptr_->is_update_) {
        plane_list.push_back(*current_octo->plane_ptr_);
    }
    if (current_octo->layer_ < current_octo->max_layer_) {
        if (!current_octo->plane_ptr_->is_plane_) {
            for (size_t i = 0; i < 8; i++) {
                if (current_octo->leaves_[i] != nullptr) {
                    GetUpdatePlane(current_octo->leaves_[i],
                                   pub_max_voxel_layer, plane_list);
                }
            }
        }
    }
    return;
}

void VoxelMapManager::MapJet(double v, double vmin, double vmax, uint8_t& r,
                             uint8_t& g, uint8_t& b) const {
    r = 255;
    g = 255;
    b = 255;

    if (v < vmin) {
        v = vmin;
    }

    if (v > vmax) {
        v = vmax;
    }

    double dr, dg, db;

    if (v < 0.1242) {
        db = 0.504 + ((1. - 0.504) / 0.1242) * v;
        dg = dr = 0.;
    } else if (v < 0.3747) {
        db = 1.;
        dr = 0.;
        dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
    } else if (v < 0.6253) {
        db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
        dg = 1.;
        dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
    } else if (v < 0.8758) {
        db = 0.;
        dr = 1.;
        dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
    } else {
        db = 0.;
        dg = 0.;
        dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
    }

    r = (uint8_t)(255 * dr);
    g = (uint8_t)(255 * dg);
    b = (uint8_t)(255 * db);
}

void VoxelMapManager::CollectUpdatePlanes(
    std::vector<VoxelPlane>* planes) const {
    if (planes == nullptr)
        return;
    planes->clear();
    for (auto iter = voxel_map_.begin(); iter != voxel_map_.end(); ++iter) {
        GetUpdatePlane(iter->second, config_setting_.max_layer_, *planes);
    }
}

void VoxelMapManager::MapSliding() {
    if ((position_last_ - last_slide_position_).norm() <
        config_setting_.sliding_thresh_) {
        return;
    }

    last_slide_position_ = position_last_;
    float loc_xyz[3];
    for (int j = 0; j < 3; j++) {
        loc_xyz[j] = position_last_[j] / config_setting_.max_voxel_size_;
        if (loc_xyz[j] < 0) {
            loc_xyz[j] -= 1.0;
        }
    }
    ClearMemOutOfMap((int64_t)loc_xyz[0] + config_setting_.half_map_size_,
                     (int64_t)loc_xyz[0] - config_setting_.half_map_size_,
                     (int64_t)loc_xyz[1] + config_setting_.half_map_size_,
                     (int64_t)loc_xyz[1] - config_setting_.half_map_size_,
                     (int64_t)loc_xyz[2] + config_setting_.half_map_size_,
                     (int64_t)loc_xyz[2] - config_setting_.half_map_size_);
}

void VoxelMapManager::ClearMemOutOfMap(const int& x_max, const int& x_min,
                                       const int& y_max, const int& y_min,
                                       const int& z_max, const int& z_min) {
    std::unique_lock<std::shared_mutex> lock(map_mutex_);
    for (auto it = voxel_map_.begin(); it != voxel_map_.end();) {
        const VoxelLocation& loc = it->first;
        bool should_remove = loc.x > x_max || loc.x < x_min || loc.y > y_max ||
                             loc.y < y_min || loc.z > z_max || loc.z < z_min;
        if (should_remove) {
            delete it->second;
            it = voxel_map_.erase(it);
        } else {
            ++it;
        }
    }
}
}  // namespace map
}  // namespace argus
