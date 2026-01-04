/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autonomy/planning/planner/egvg/dbscan2d.hpp"

#include <queue>

namespace autonomy {
namespace planning {
namespace planner {
namespace egvg {

// 运行聚类，返回聚类总数
int DBScan2D::run(std::vector<std::vector<int>>& clusters) {
    int clusterId = 0;
    for (size_t i = 0; i < points_.size(); ++i) {
        if (labels_[i] == UNCLASSIFIED) {
            if (expandCluster(static_cast<int>(i), clusterId)) {
                ++clusterId;
            }
        }
    }
    // 输出每个聚类包含的点编号
    clusters.clear();
    clusters.resize(clusterId);
    for (size_t i = 0; i < labels_.size(); ++i) {
        if (labels_[i] >= 0) {
            clusters[labels_[i]].push_back(static_cast<int>(i));
        }
    }
    return clusterId;
}

// 计算欧氏距离平方（只使用 x,y）
inline float DBScan2D::dist2(const Point& a, const Point& b) const {
    const float dx = static_cast<float>(a.x - b.x);
    const float dy = static_cast<float>(a.y - b.y);
    return dx * dx + dy * dy;
}

// 找到邻域内所有点的索引
std::vector<int> DBScan2D::regionQuery(int idx) const {
    std::vector<int> neighbors;
    for (size_t i = 0; i < points_.size(); ++i) {
        if (dist2(points_[idx], points_[i]) <= eps_ * eps_) {
            neighbors.push_back(static_cast<int>(i));
        }
    }
    return neighbors;
}

// 尝试扩展一个聚类
bool DBScan2D::expandCluster(int idx, int clusterId) {
    auto seeds = regionQuery(idx);
    if (static_cast<int>(seeds.size()) < minPts_) {
        labels_[idx] = NOISE;
        return false;
    }
    for (int seedIdx : seeds) {
        labels_[seedIdx] = clusterId;
    }
    std::queue<int> q;
    for (int seedIdx : seeds) {
        if (seedIdx != idx) {
            q.push(seedIdx);
        }
    }
    while (!q.empty()) {
        int curr = q.front();
        q.pop();
        auto neighbors = regionQuery(curr);
        if (static_cast<int>(neighbors.size()) >= minPts_) {
            for (int nIdx : neighbors) {
                if (labels_[nIdx] == UNCLASSIFIED || labels_[nIdx] == NOISE) {
                    if (labels_[nIdx] == UNCLASSIFIED) {
                        q.push(nIdx);
                    }
                    labels_[nIdx] = clusterId;
                }
            }
        }
    }
    return true;
}

}  // namespace egvg
}  // namespace planner
}  // namespace planning
}  // namespace autonomy