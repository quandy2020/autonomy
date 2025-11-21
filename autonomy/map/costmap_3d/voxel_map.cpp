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

#include "autonomy/map/costmap_3d/voxel_map.hpp"
#include "autonomy/map/costmap_3d/voxel_dilater.hpp"

namespace autonomy {
namespace map {
namespace costmap_3d {

constexpr uint8 kConstantUnoccupied = 0;
constexpr uint8 kConstantOccupied = 1;
constexpr uint8 kConstantDilated = 2;

VoxelMap::VoxelMap(const Eigen::Vector3i& size, 
    const Eigen::Vector3d& origin, const double& vox_scale)
    : map_size_(size),
      o_(origin),
      scale_(vox_scale),
      resolution_(vox_scale),
      vox_num_(map_size_.prod()),
      step_(1, map_size_(0), map_size_(1) * map_size_(0)),
      oc_(o_ + Eigen::Vector3d::Constant(0.5 * scale_)),
      bounds_((map_size_.array() - 1) * step_.array()),
      step_scale_(step_.cast<double>().cwiseInverse() * scale_),
      voxels_(vox_num_, kConstantUnoccupied) 
{}

void VoxelMap::SetOccupied(const Eigen::Vector3d& pos)
{
    const Eigen::Vector3i id = ((pos - o_) / scale_).cast<int>();
    if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
        id(0) < map_size_(0) && id(1) < map_size_(1) && id(2) < map_size_(2))
    {
        voxels_[id.dot(step_)] = kConstantOccupied;
    }
}

void VoxelMap::SetOccupied(const Eigen::Vector3i& id)
{
    if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
        id(0) < map_size_(0) && id(1) < map_size_(1) && id(2) < map_size_(2))
    {
        voxels_[id.dot(step_)] = kConstantOccupied;
    }
}

void VoxelMap::Dilate(const int &r)
{
    if (r <= 0) {
        return;
    }
    
    std::vector<Eigen::Vector3i> lvec, cvec;
    lvec.reserve(vox_num_);
    cvec.reserve(vox_num_);
    int i, j, k, idx;
    bool check;
    for (int x = 0; x <= bounds_(0); x++)
    {
        for (int y = 0; y <= bounds_(1); y += step_(1))
        {
            for (int z = 0; z <= bounds_(2); z += step_(2))
            {
                if (voxels_[x + y + z] == kConstantOccupied)
                {
                    VOXEL_DILATER(i, j, k, x, y, z, step_(1), step_(2), 
                        bounds_(0), bounds_(1), bounds_(2), check, voxels_, idx, kConstantDilated, cvec)
                }
            }
        }
    }

    for (int loop = 1; loop < r; loop++)
    {
        std::swap(cvec, lvec);
        for (const Eigen::Vector3i& id : lvec)
        {
            VOXEL_DILATER(i, j, k, id(0), id(1), id(2), step_(1), step_(2),
                bounds_(0), bounds_(1), bounds_(2), check, voxels_, idx, kConstantDilated, cvec)
        }
        lvec.clear();
    }
    surf_ = cvec;
}

void VoxelMap::GetSurfInBox(const Eigen::Vector3i& center, const int& half_width, 
    std::vector<Eigen::Vector3d>& points) const
{
    for (const Eigen::Vector3i& id : surf_)
    {
        if (std::abs(id(0) - center(0)) <= half_width &&
            std::abs(id(1) / step_(1) - center(1)) <= half_width &&
            std::abs(id(2) / step_(2) - center(2)) <= half_width)
        {
            points.push_back(id.cast<double>().cwiseProduct(step_scale_) + oc_);
        }
    }
}

void VoxelMap::GetSurf(std::vector<Eigen::Vector3d>& points) const
{
    points.reserve(surf_.size());
    for (const Eigen::Vector3i& id : surf_)
    {
        points.push_back(id.cast<double>().cwiseProduct(step_scale_) + oc_);
    }
}

bool VoxelMap::Query(const Eigen::Vector3d& pos) const
{
    const Eigen::Vector3i id = ((pos - o_) / scale_).cast<int>();
    if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
        id(0) < map_size_(0) && id(1) < map_size_(1) && id(2) < map_size_(2))
    {
        return voxels_[id.dot(step_)];
    }
    return true;
}

bool VoxelMap::Query(const Eigen::Vector3i& id) const
{
    if (id(0) >= 0 && id(1) >= 0 && id(2) >= 0 &&
        id(0) < map_size_(0) && id(1) < map_size_(1) && id(2) < map_size_(2))
    {
        return voxels_[id.dot(step_)];
    }
    return true;
}

}  // namespace costmap_3d
}  // namespace map
}  // namespace autonomy