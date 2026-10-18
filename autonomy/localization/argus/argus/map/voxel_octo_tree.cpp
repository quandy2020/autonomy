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

#include "argus/map/voxel_octo_tree.hpp"

#include <cmath>
#include <vector>

namespace argus {
namespace map {

namespace {

int g_voxel_plane_id = 0;

void AccumulatePlaneMoments(const std::vector<PointWithVar>& points,
                            VoxelPlane* plane) {
    plane->plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    plane->covariance_ = Eigen::Matrix3d::Zero();
    plane->center_ = Eigen::Vector3d::Zero();
    plane->normal_ = Eigen::Vector3d::Zero();
    plane->points_size_ = points.size();
    plane->radius_ = 0;
    for (auto pv : points) {
        plane->covariance_ += pv.point_w * pv.point_w.transpose();
        plane->center_ += pv.point_w;
    }
    plane->center_ = plane->center_ / plane->points_size_;
    plane->covariance_ = plane->covariance_ / plane->points_size_ -
                         plane->center_ * plane->center_.transpose();
}

void AccumulatePlaneVariance(const std::vector<PointWithVar>& points,
                             VoxelPlane* plane, const Eigen::Vector3cd& evals,
                             const Eigen::Matrix3cd& evecs,
                             Eigen::Matrix3d::Index evalsMin) {
    Eigen::Vector3d evalsReal = evals.real();
    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size_, 0, 0, 0, 1.0 / plane->points_size_, 0, 0,
        0, 1.0 / plane->points_size_;
    for (int i = 0; i < points.size(); i++) {
        Eigen::Matrix<double, 6, 3> J;
        Eigen::Matrix3d F;
        for (int m = 0; m < 3; m++) {
            if (m != (int)evalsMin) {
                Eigen::Matrix<double, 1, 3> F_m =
                    (points[i].point_w - plane->center_).transpose() /
                    ((plane->points_size_) *
                     (evalsReal[evalsMin] - evalsReal[m])) *
                    (evecs.real().col(m) *
                         evecs.real().col(evalsMin).transpose() +
                     evecs.real().col(evalsMin) *
                         evecs.real().col(m).transpose());
                F.row(m) = F_m;
            } else {
                Eigen::Matrix<double, 1, 3> F_m;
                F_m << 0, 0, 0;
                F.row(m) = F_m;
            }
        }
        J.block<3, 3>(0, 0) = evecs.real() * F;
        J.block<3, 3>(3, 0) = J_Q;
        plane->plane_var_ += J * points[i].var * J.transpose();
    }
}

void FillPlaneFromEigens(VoxelPlane* plane, const Eigen::Vector3d& evalsReal,
                         const Eigen::Matrix3cd& evecs,
                         Eigen::Matrix3d::Index evalsMin,
                         Eigen::Matrix3d::Index evalsMid,
                         Eigen::Matrix3d::Index evalsMax) {
    plane->normal_ << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
        evecs.real()(2, evalsMin);
    plane->y_normal_ << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
        evecs.real()(2, evalsMid);
    plane->x_normal_ << evecs.real()(0, evalsMax), evecs.real()(1, evalsMax),
        evecs.real()(2, evalsMax);
    plane->min_eigen_value_ = evalsReal(evalsMin);
    plane->mid_eigen_value_ = evalsReal(evalsMid);
    plane->max_eigen_value_ = evalsReal(evalsMax);
    plane->radius_ = std::sqrt(evalsReal(evalsMax));
    plane->d_ = -(plane->normal_(0) * plane->center_(0) +
                  plane->normal_(1) * plane->center_(1) +
                  plane->normal_(2) * plane->center_(2));
    plane->is_plane_ = true;
    plane->is_update_ = true;
    if (!plane->is_init_) {
        plane->id_ = g_voxel_plane_id;
        g_voxel_plane_id++;
        plane->is_init_ = true;
    }
}

}  // namespace

VoxelOctoTree::VoxelOctoTree(int max_layer, int layer,
                             int points_size_threshold, int max_points_num,
                             float planer_threshold)
    : max_layer_(max_layer),
      layer_(layer),
      points_size_threshold_(points_size_threshold),
      max_points_num_(max_points_num),
      planer_threshold_(planer_threshold) {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++) {
        leaves_[i] = nullptr;
    }
    plane_ptr_ = new VoxelPlane;
}

VoxelOctoTree::~VoxelOctoTree() {
    for (int i = 0; i < 8; i++) {
        delete leaves_[i];
    }
    delete plane_ptr_;
}

void VoxelOctoTree::init_plane(const std::vector<PointWithVar>& points,
                               VoxelPlane* plane) {
    AccumulatePlaneMoments(points, plane);
    Eigen::EigenSolver<Eigen::Matrix3d> es(plane->covariance_);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3f::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;
    // && evalsReal(evalsMid) > 0.05
    //&& evalsReal(evalsMid) > 0.01
    if (evalsReal(evalsMin) < planer_threshold_) {
        AccumulatePlaneVariance(points, plane, evals, evecs, evalsMin);
        FillPlaneFromEigens(plane, evalsReal, evecs, evalsMin, evalsMid,
                            evalsMax);
    } else {
        plane->is_update_ = true;
        plane->is_plane_ = false;
    }
}

void VoxelOctoTree::init_octo_tree() {
    if (temp_points_.size() > points_size_threshold_) {
        init_plane(temp_points_, plane_ptr_);
        if (plane_ptr_->is_plane_ == true) {
            octo_state_ = 0;
            // new added
            if (temp_points_.size() > max_points_num_) {
                update_enable_ = false;
                std::vector<PointWithVar>().swap(temp_points_);
                new_points_ = 0;
            }
        } else {
            octo_state_ = 1;
            cut_octo_tree();
        }
        init_octo_ = true;
        new_points_ = 0;
    }
}

void VoxelOctoTree::cut_octo_tree() {
    if (layer_ >= max_layer_) {
        octo_state_ = 0;
        return;
    }
    for (size_t i = 0; i < temp_points_.size(); i++) {
        int xyz[3] = {0, 0, 0};
        if (temp_points_[i].point_w[0] > voxel_center_[0]) {
            xyz[0] = 1;
        }
        if (temp_points_[i].point_w[1] > voxel_center_[1]) {
            xyz[1] = 1;
        }
        if (temp_points_[i].point_w[2] > voxel_center_[2]) {
            xyz[2] = 1;
        }
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] == nullptr) {
            leaves_[leafnum] = new VoxelOctoTree(
                max_layer_, layer_ + 1, layer_init_num_[layer_ + 1],
                max_points_num_, planer_threshold_);
            leaves_[leafnum]->layer_init_num_ = layer_init_num_;
            leaves_[leafnum]->voxel_center_[0] =
                voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] =
                voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] =
                voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
        }
        leaves_[leafnum]->temp_points_.push_back(temp_points_[i]);
        leaves_[leafnum]->new_points_++;
    }
    for (uint i = 0; i < 8; i++) {
        if (leaves_[i] != nullptr) {
            if (leaves_[i]->temp_points_.size() >
                leaves_[i]->points_size_threshold_) {
                init_plane(leaves_[i]->temp_points_, leaves_[i]->plane_ptr_);
                if (leaves_[i]->plane_ptr_->is_plane_) {
                    leaves_[i]->octo_state_ = 0;
                    // new added
                    if (leaves_[i]->temp_points_.size() >
                        leaves_[i]->max_points_num_) {
                        leaves_[i]->update_enable_ = false;
                        std::vector<PointWithVar>().swap(
                            leaves_[i]->temp_points_);
                        new_points_ = 0;
                    }
                } else {
                    leaves_[i]->octo_state_ = 1;
                    leaves_[i]->cut_octo_tree();
                }
                leaves_[i]->init_octo_ = true;
                leaves_[i]->new_points_ = 0;
            }
        }
    }
    // Points now live in children; free parent buffer (no longer needed).
    std::vector<PointWithVar>().swap(temp_points_);
}

void VoxelOctoTree::UpdateOctoTree(const PointWithVar& pv) {
    if (!init_octo_) {
        new_points_++;
        temp_points_.push_back(pv);
        if (temp_points_.size() > points_size_threshold_) {
            init_octo_tree();
        }
    } else {
        if (plane_ptr_->is_plane_) {
            if (update_enable_) {
                new_points_++;
                temp_points_.push_back(pv);
                if (new_points_ > update_size_threshold_) {
                    init_plane(temp_points_, plane_ptr_);
                    new_points_ = 0;
                }
                if (temp_points_.size() >= max_points_num_) {
                    update_enable_ = false;
                    std::vector<PointWithVar>().swap(temp_points_);
                    new_points_ = 0;
                }
            }
        } else {
            if (layer_ < max_layer_) {
                int xyz[3] = {0, 0, 0};
                if (pv.point_w[0] > voxel_center_[0]) {
                    xyz[0] = 1;
                }
                if (pv.point_w[1] > voxel_center_[1]) {
                    xyz[1] = 1;
                }
                if (pv.point_w[2] > voxel_center_[2]) {
                    xyz[2] = 1;
                }
                int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                if (leaves_[leafnum] != nullptr) {
                    leaves_[leafnum]->UpdateOctoTree(pv);
                } else {
                    leaves_[leafnum] = new VoxelOctoTree(
                        max_layer_, layer_ + 1, layer_init_num_[layer_ + 1],
                        max_points_num_, planer_threshold_);
                    leaves_[leafnum]->layer_init_num_ = layer_init_num_;
                    leaves_[leafnum]->voxel_center_[0] =
                        voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[1] =
                        voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
                    leaves_[leafnum]->voxel_center_[2] =
                        voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
                    leaves_[leafnum]->quater_length_ = quater_length_ / 2;
                    leaves_[leafnum]->UpdateOctoTree(pv);
                }
            } else {
                if (update_enable_) {
                    new_points_++;
                    temp_points_.push_back(pv);
                    if (new_points_ > update_size_threshold_) {
                        init_plane(temp_points_, plane_ptr_);
                        new_points_ = 0;
                    }
                    if (temp_points_.size() > max_points_num_) {
                        update_enable_ = false;
                        std::vector<PointWithVar>().swap(temp_points_);
                        new_points_ = 0;
                    }
                }
            }
        }
    }
}

VoxelOctoTree* VoxelOctoTree::find_correspond(Eigen::Vector3d pw) {
    if (!init_octo_ || plane_ptr_->is_plane_ || (layer_ >= max_layer_))
        return this;

    int xyz[3] = {0, 0, 0};
    xyz[0] = pw[0] > voxel_center_[0] ? 1 : 0;
    xyz[1] = pw[1] > voxel_center_[1] ? 1 : 0;
    xyz[2] = pw[2] > voxel_center_[2] ? 1 : 0;
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

    // printf("leafnum: %d. \n", leafnum);

    return (leaves_[leafnum] != nullptr) ? leaves_[leafnum]->find_correspond(pw)
                                         : this;
}

VoxelOctoTree* VoxelOctoTree::Insert(const PointWithVar& pv) {
    if ((!init_octo_) || (init_octo_ && plane_ptr_->is_plane_) ||
        (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ >= max_layer_))) {
        new_points_++;
        temp_points_.push_back(pv);
        return this;
    }

    if (init_octo_ && (!plane_ptr_->is_plane_) && (layer_ < max_layer_)) {
        int xyz[3] = {0, 0, 0};
        xyz[0] = pv.point_w[0] > voxel_center_[0] ? 1 : 0;
        xyz[1] = pv.point_w[1] > voxel_center_[1] ? 1 : 0;
        xyz[2] = pv.point_w[2] > voxel_center_[2] ? 1 : 0;
        int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
        if (leaves_[leafnum] != nullptr) {
            return leaves_[leafnum]->Insert(pv);
        } else {
            leaves_[leafnum] = new VoxelOctoTree(
                max_layer_, layer_ + 1, layer_init_num_[layer_ + 1],
                max_points_num_, planer_threshold_);
            leaves_[leafnum]->layer_init_num_ = layer_init_num_;
            leaves_[leafnum]->voxel_center_[0] =
                voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[1] =
                voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
            leaves_[leafnum]->voxel_center_[2] =
                voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
            leaves_[leafnum]->quater_length_ = quater_length_ / 2;
            return leaves_[leafnum]->Insert(pv);
        }
    }
    return nullptr;
}

}  // namespace map
}  // namespace argus
