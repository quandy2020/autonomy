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

#include "map/voxel/voxel_plane_map.hpp"

#include <Eigen/Eigenvalues>
#include <array>
#include <cmath>
#include <unordered_map>
#include <vector>

#include "map/voxel/voxel_key_hash.hpp"

namespace argus {
namespace map {
namespace voxel {
namespace {

int ChildIndex(const Eigen::Vector3d& point, const Eigen::Vector3d& center) {
  const int x = point.x() > center.x() ? 1 : 0;
  const int y = point.y() > center.y() ? 1 : 0;
  const int z = point.z() > center.z() ? 1 : 0;
  return 4 * x + 2 * y + z;
}

VoxelKey KeyOf(const Eigen::Vector3d& point, double voxel_size) {
  VoxelKey key;
  const double coordinates[3] = {point.x(), point.y(), point.z()};
  int* slots[3] = {&key.x, &key.y, &key.z};
  for (int axis = 0; axis < 3; ++axis) {
    double loc = coordinates[axis] / voxel_size;
    if (loc < 0.0) {
      loc -= 1.0;
    }
    *slots[axis] = static_cast<int>(loc);
  }
  return key;
}

struct Node {
  int layer = 0;
  bool subdivided = false;
  bool is_plane = false;
  double quater_length = 0.0;
  double radius = 0.0;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d plane_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d plane_normal = Eigen::Vector3d::UnitZ();
  std::vector<Eigen::Vector3d> points;
  std::array<std::unique_ptr<Node>, 8> leaves;
};

}  // namespace

struct VoxelPlaneMap::Impl {
  Config config;
  std::unordered_map<VoxelKey, std::unique_ptr<Node>, VoxelKeyHash> roots;

  std::unique_ptr<Node> MakeChild(const Node& parent, int leaf) const {
    auto child = std::make_unique<Node>();
    child->layer = parent.layer + 1;
    child->quater_length = parent.quater_length * 0.5;
    const int x = (leaf >> 2) & 1;
    const int y = (leaf >> 1) & 1;
    const int z = leaf & 1;
    const int bits[3] = {x, y, z};
    child->center = parent.center;
    for (int axis = 0; axis < 3; ++axis) {
      child->center[axis] += (2 * bits[axis] - 1) * parent.quater_length;
    }
    return child;
  }

  void Fit(Node* node) const {
    node->is_plane = false;
    const int count = static_cast<int>(node->points.size());
    if (count <= config.min_points) {
      return;
    }
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const Eigen::Vector3d& point : node->points) {
      mean += point;
    }
    mean /= static_cast<double>(count);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (const Eigen::Vector3d& point : node->points) {
      const Eigen::Vector3d delta = point - mean;
      covariance += delta * delta.transpose();
    }
    covariance /= static_cast<double>(count);
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    const Eigen::Vector3d eigenvalues = solver.eigenvalues();
    if (eigenvalues[2] <= 1e-12) {
      return;
    }
    const double ratio = config.plane_ratio;
    if (eigenvalues[0] >= config.min_eigenvalue ||
        eigenvalues[0] / eigenvalues[2] >= ratio) {
      return;
    }
    node->is_plane = true;
    node->plane_center = mean;
    node->plane_normal = solver.eigenvectors().col(0);
    node->radius = eigenvalues[2];
  }

  void CommitNode(Node* node) {
    if (node->subdivided) {
      for (const std::unique_ptr<Node>& child : node->leaves) {
        if (child) {
          CommitNode(child.get());
        }
      }
      return;
    }
    Fit(node);
    if (node->is_plane || node->layer >= config.max_layer) {
      return;
    }
    if (static_cast<int>(node->points.size()) <= config.min_points) {
      return;
    }
    node->subdivided = true;
    node->is_plane = false;
    for (const Eigen::Vector3d& point : node->points) {
      const int leaf = ChildIndex(point, node->center);
      if (!node->leaves[static_cast<std::size_t>(leaf)]) {
        node->leaves[static_cast<std::size_t>(leaf)] = MakeChild(*node, leaf);
      }
      node->leaves[static_cast<std::size_t>(leaf)]->points.push_back(point);
    }
    node->points.clear();
    for (const std::unique_ptr<Node>& child : node->leaves) {
      if (child) {
        CommitNode(child.get());
      }
    }
  }

  void InsertNode(Node* node, const Eigen::Vector3d& point) {
    if (!node->subdivided) {
      node->points.push_back(point);
      node->is_plane = false;
      return;
    }
    const int leaf = ChildIndex(point, node->center);
    std::unique_ptr<Node>& child = node->leaves[static_cast<std::size_t>(leaf)];
    if (!child) {
      child = MakeChild(*node, leaf);
    }
    InsertNode(child.get(), point);
  }

  bool QueryNode(const Node& node, const Eigen::Vector3d& point,
                 Eigen::Vector3d* center, Eigen::Vector3d* normal) const {
    if (node.subdivided) {
      const int leaf = ChildIndex(point, node.center);
      const Node* child = node.leaves[static_cast<std::size_t>(leaf)].get();
      if (child == nullptr) {
        return false;
      }
      return QueryNode(*child, point, center, normal);
    }
    if (!node.is_plane) {
      return false;
    }
    const double distance =
        std::abs(node.plane_normal.dot(point - node.plane_center));
    if (distance > config.max_plane_distance) {
      return false;
    }
    const double center_distance = (point - node.plane_center).squaredNorm();
    const double range = center_distance - distance * distance;
    if (range > 9.0 * node.radius) {
      return false;
    }
    *center = node.plane_center;
    *normal = node.plane_normal;
    return true;
  }

  std::size_t CountPlanes(const Node& node) const {
    if (node.subdivided) {
      std::size_t count = 0;
      for (const std::unique_ptr<Node>& child : node.leaves) {
        if (child) {
          count += CountPlanes(*child);
        }
      }
      return count;
    }
    return node.is_plane ? 1u : 0u;
  }
};

VoxelPlaneMap::VoxelPlaneMap() : VoxelPlaneMap(Config{}) {}

VoxelPlaneMap::VoxelPlaneMap(Config config) : impl_(std::make_unique<Impl>()) {
  impl_->config = config;
}

VoxelPlaneMap::~VoxelPlaneMap() = default;

VoxelPlaneMap::VoxelPlaneMap(VoxelPlaneMap&&) noexcept = default;

VoxelPlaneMap& VoxelPlaneMap::operator=(VoxelPlaneMap&&) noexcept = default;

void VoxelPlaneMap::Insert(const Eigen::Vector3d& point) {
  if (impl_->config.voxel_size <= 0.0) {
    return;
  }
  const VoxelKey key = KeyOf(point, impl_->config.voxel_size);
  std::unique_ptr<Node>& root = impl_->roots[key];
  if (!root) {
    root = std::make_unique<Node>();
    root->layer = 0;
    root->quater_length = impl_->config.voxel_size / 4.0;
    root->center = (Eigen::Vector3d(key.x, key.y, key.z).array() + 0.5) *
                   impl_->config.voxel_size;
  }
  impl_->InsertNode(root.get(), point);
}

void VoxelPlaneMap::Commit() {
  for (auto& entry : impl_->roots) {
    impl_->CommitNode(entry.second.get());
  }
}

bool VoxelPlaneMap::Query(const Eigen::Vector3d& point, Eigen::Vector3d* center,
                          Eigen::Vector3d* normal) const {
  if (center == nullptr || normal == nullptr ||
      impl_->config.voxel_size <= 0.0) {
    return false;
  }
  const auto found = impl_->roots.find(KeyOf(point, impl_->config.voxel_size));
  if (found == impl_->roots.end()) {
    return false;
  }
  return impl_->QueryNode(*found->second, point, center, normal);
}

std::size_t VoxelPlaneMap::plane_count() const {
  std::size_t count = 0;
  for (const auto& entry : impl_->roots) {
    count += impl_->CountPlanes(*entry.second);
  }
  return count;
}

}  // namespace voxel
}  // namespace map
}  // namespace argus
