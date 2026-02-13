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

#pragma once

#include <cmath>
#include <queue>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace egvg {

// Use geometry_msgs::Point as the 2D sample type for clustering.
using Point = autonomy::commsgs::geometry_msgs::Point;

enum DBScanLabel { UNCLASSIFIED = -2, NOISE = -1 };

class DBScan2D
{
public:
    DBScan2D(const std::vector<Point>& points, float eps, int minPts)
        : points_(points), eps_(eps), minPts_(minPts), labels_(points.size(), UNCLASSIFIED) {}

    // 运行聚类，返回聚类总数
    int run(std::vector<std::vector<int>>& clusters);

    // 获取每个点的聚类标签（-1为噪声）
    const std::vector<int>& getLabels() const {
        return labels_;
    }

private:
    const std::vector<Point>& points_;
    float eps_;
    int minPts_;
    std::vector<int> labels_;

    // 计算欧氏距离平方（仅使用 x,y）
    inline float dist2(const Point& a, const Point& b) const;

    // 找到邻域内所有点的索引
    std::vector<int> regionQuery(int idx) const;

    // 尝试扩展一个聚类
    bool expandCluster(int idx, int clusterId);
};

}  // namespace egvg
}  // namespace planner
}  // namespace planning
}  // namespace autonomy