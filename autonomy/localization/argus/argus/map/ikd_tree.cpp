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

#include "argus/map/ikd_tree.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <utility>

namespace argus {
namespace map {

IkdTree::IkdTree(IkdTreeConfig config) : config_(std::move(config)) {}

IkdTree::~IkdTree() {
    Clear();
}

Eigen::Vector3d IkdTree::ToVec(const sensor::PointType& p) {
    return Eigen::Vector3d(p.x, p.y, p.z);
}

void IkdTree::Destroy(Node* n) {
    if (n == nullptr) {
        return;
    }
    Destroy(n->left);
    Destroy(n->right);
    delete n;
}

void IkdTree::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    Destroy(root_);
    root_ = nullptr;
    size_ = 0;
    size_at_build_ = 0;
}

std::size_t IkdTree::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return size_;
}

void IkdTree::Flatten(const Node* n, std::vector<Eigen::Vector3d>* out) const {
    if (n == nullptr || out == nullptr) {
        return;
    }
    out->push_back(n->point);
    Flatten(n->left, out);
    Flatten(n->right, out);
}

IkdTree::Node* IkdTree::BuildRecursive(std::vector<Eigen::Vector3d>* pts,
                                       int begin, int end, int depth) {
    if (begin >= end) {
        return nullptr;
    }
    const int axis = depth % 3;
    const int mid = begin + (end - begin) / 2;
    std::nth_element(
        pts->begin() + begin, pts->begin() + mid, pts->begin() + end,
        [axis](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
            return a[axis] < b[axis];
        });
    Node* node = new Node;
    node->point = (*pts)[static_cast<std::size_t>(mid)];
    node->axis = axis;
    node->left = BuildRecursive(pts, begin, mid, depth + 1);
    node->right = BuildRecursive(pts, mid + 1, end, depth + 1);
    return node;
}

void IkdTree::Build(const sensor::PointCloudXYZI& cloud) {
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        pts.push_back(ToVec(cloud[i]));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    Destroy(root_);
    root_ = nullptr;
    size_ = pts.size();
    root_ = BuildRecursive(&pts, 0, static_cast<int>(pts.size()), 0);
    size_at_build_ = size_;
}

void IkdTree::AddPoints(const sensor::PointCloudXYZI& cloud) {
    std::vector<Eigen::Vector3d> incoming;
    incoming.reserve(cloud.size());

    if (config_.downsample_resolution > 1e-6) {
        const double res = config_.downsample_resolution;
        std::unordered_set<int64_t> seen;
        auto key = [res](const Eigen::Vector3d& p) {
            const int64_t ix = static_cast<int64_t>(std::floor(p.x() / res));
            const int64_t iy = static_cast<int64_t>(std::floor(p.y() / res));
            const int64_t iz = static_cast<int64_t>(std::floor(p.z() / res));
            return (ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791);
        };
        for (std::size_t i = 0; i < cloud.size(); ++i) {
            const Eigen::Vector3d p = ToVec(cloud[i]);
            if (seen.insert(key(p)).second) {
                incoming.push_back(p);
            }
        }
    } else {
        for (std::size_t i = 0; i < cloud.size(); ++i) {
            incoming.push_back(ToVec(cloud[i]));
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (root_ == nullptr) {
        root_ =
            BuildRecursive(&incoming, 0, static_cast<int>(incoming.size()), 0);
        size_ = incoming.size();
        size_at_build_ = size_;
        return;
    }
    // Append via rebuild of flattened + new (simple incremental strategy).
    std::vector<Eigen::Vector3d> all;
    all.reserve(size_ + incoming.size());
    Flatten(root_, &all);
    all.insert(all.end(), incoming.begin(), incoming.end());
    Destroy(root_);
    root_ = BuildRecursive(&all, 0, static_cast<int>(all.size()), 0);
    size_ = all.size();
    size_at_build_ = size_;
    (void)config_.max_leaf_size;
}

void IkdTree::NearestRecursive(const Node* n, const Eigen::Vector3d& q, int k,
                               std::vector<HeapItem>* heap) const {
    if (n == nullptr || heap == nullptr) {
        return;
    }
    const float d2 = static_cast<float>((n->point - q).squaredNorm());
    if (static_cast<int>(heap->size()) < k) {
        heap->push_back(HeapItem{d2, n->point});
        std::push_heap(heap->begin(), heap->end());
    } else if (d2 < heap->front().dist_sq) {
        std::pop_heap(heap->begin(), heap->end());
        heap->back() = HeapItem{d2, n->point};
        std::push_heap(heap->begin(), heap->end());
    }

    const double diff = q[n->axis] - n->point[n->axis];
    const Node* first = diff < 0 ? n->left : n->right;
    const Node* second = diff < 0 ? n->right : n->left;
    NearestRecursive(first, q, k, heap);
    const float worst = heap->empty()
                            ? std::numeric_limits<float>::infinity()
                            : (static_cast<int>(heap->size()) < k
                                   ? std::numeric_limits<float>::infinity()
                                   : heap->front().dist_sq);
    if (diff * diff < worst) {
        NearestRecursive(second, q, k, heap);
    }
}

int IkdTree::NearestSearch(const Eigen::Vector3d& query, int k,
                           std::vector<Eigen::Vector3d>* points,
                           std::vector<float>* dists_sq) const {
    if (points != nullptr) {
        points->clear();
    }
    if (dists_sq != nullptr) {
        dists_sq->clear();
    }
    if (k <= 0) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<HeapItem> heap;
    heap.reserve(static_cast<std::size_t>(k));
    NearestRecursive(root_, query, k, &heap);
    std::sort_heap(heap.begin(), heap.end());
    // sort_heap leaves ascending; we used max-heap so reverse for
    // nearest-first.
    std::sort(heap.begin(), heap.end(),
              [](const HeapItem& a, const HeapItem& b) {
                  return a.dist_sq < b.dist_sq;
              });
    for (const HeapItem& h : heap) {
        if (points != nullptr) {
            points->push_back(h.point);
        }
        if (dists_sq != nullptr) {
            dists_sq->push_back(h.dist_sq);
        }
    }
    return static_cast<int>(heap.size());
}

void IkdTree::RadiusRecursive(const Node* n, const Eigen::Vector3d& q, float r2,
                              std::vector<HeapItem>* out) const {
    if (n == nullptr || out == nullptr) {
        return;
    }
    const float d2 = static_cast<float>((n->point - q).squaredNorm());
    if (d2 <= r2) {
        out->push_back(HeapItem{d2, n->point});
    }
    const double diff = q[n->axis] - n->point[n->axis];
    const Node* first = diff < 0 ? n->left : n->right;
    const Node* second = diff < 0 ? n->right : n->left;
    RadiusRecursive(first, q, r2, out);
    if (diff * diff <= r2) {
        RadiusRecursive(second, q, r2, out);
    }
}

int IkdTree::RadiusSearch(const Eigen::Vector3d& query, float radius,
                          std::vector<Eigen::Vector3d>* points,
                          std::vector<float>* dists_sq) const {
    if (points != nullptr) {
        points->clear();
    }
    if (dists_sq != nullptr) {
        dists_sq->clear();
    }
    if (radius < 0.f) {
        return 0;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<HeapItem> hits;
    RadiusRecursive(root_, query, radius * radius, &hits);
    std::sort(hits.begin(), hits.end(),
              [](const HeapItem& a, const HeapItem& b) {
                  return a.dist_sq < b.dist_sq;
              });
    for (const HeapItem& h : hits) {
        if (points != nullptr) {
            points->push_back(h.point);
        }
        if (dists_sq != nullptr) {
            dists_sq->push_back(h.dist_sq);
        }
    }
    return static_cast<int>(hits.size());
}

void IkdTree::ExportPoints(std::vector<Eigen::Vector3d>* points) const {
    if (points == nullptr) {
        return;
    }
    points->clear();
    std::lock_guard<std::mutex> lock(mutex_);
    points->reserve(size_);
    Flatten(root_, points);
}

}  // namespace map
}  // namespace argus
