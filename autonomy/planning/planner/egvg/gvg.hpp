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

// https://github.com/qimao7213/TGH-Planner/tree/main

#pragma once

#include <algorithm>
#include <boost/functional/hash.hpp>
#include <cassert>
#include <iostream>
#include <map>
#include <memory>
#include <queue>
#include <stack>
#include <unordered_map>
#include <vector>

// OpenCV is only needed for optional visualization helpers.
// Guard it behind USE_OPENCV to avoid build failures when OpenCV dev headers
// are incomplete or not installed.
#ifdef USE_OPENCV
#include <opencv4/opencv2/core.hpp>
#endif

#include <omp.h>

#include "autolink/common/log.hpp"
#include "autonomy/planning/planner/egvg/dbscan2d.hpp"
#include "autonomy/planning/planner/egvg/dynamic_voronoi.hpp"

// #define VERBOSE
// # define DEBUG_IMG
// int stage_ = 0; // 全局变量，用于调试
// cv::Mat voronoi_img(800, 800, CV_8UC3, cv::Scalar(255, 255, 255)); //
// 创建白色背景的图像 cv::Mat voronoi_img2(800, 800, CV_8UC3, cv::Scalar(255,
// 255, 255)); // 创建白色背景的图像 int total_iter_ = 0; // DFS的iter，用于调试

// 简单的 2D 整数网格点类型，用于 GVG 规划。
struct IntPoint {
    int x;
    int y;

    IntPoint() : x(0), y(0) {}
    IntPoint(int x_, int y_) : x(x_), y(y_) {}

    IntPoint operator+(const IntPoint& other) const {
        return IntPoint(x + other.x, y + other.y);
    }

    IntPoint operator-(const IntPoint& other) const {
        return IntPoint(x - other.x, y - other.y);
    }

    bool operator==(const IntPoint& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const IntPoint& other) const {
        return !(*this == other);
    }

    // 供 std::sort / std::map 使用的严格弱序
    bool operator<(const IntPoint& other) const {
        if (x < other.x)
            return true;
        if (x > other.x)
            return false;
        return y < other.y;
    }
};

// 定义哈希函数，用于 unordered_map
namespace std {
template <>
struct hash<IntPoint> {
    size_t operator()(const IntPoint& p) const noexcept {
        // 简单组合 x,y 的哈希
        return (static_cast<size_t>(p.x) * 73856093u) ^
               (static_cast<size_t>(p.y) * 19349663u);
    }
};
}  // namespace std

namespace autonomy {
namespace planning {
namespace planner {
namespace egvg {

class SimpleTimer
{
public:
    SimpleTimer(const std::string& name = "") : name_(name) {
        start_ = std::chrono::steady_clock::now();
    }
    void reset() {
        start_ = std::chrono::steady_clock::now();
    }
    double elapsedMs() const {
        auto end = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(end - start_).count();
    }
    double elapsedSec() const {
        auto end = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(end - start_).count();
    }
    void print(const std::string& msg = " ") const {
        AINFO << " [TimeCost]: " << (name_.empty() ? " " : name_ + " ") << msg
              << " " << elapsedMs() << " ms.";
    }

private:
    std::chrono::steady_clock::time_point start_;
    std::string name_;
};

// 定义邻接方向
enum Direction {
    UP = 0b0001,     // 上
    DOWN = 0b0010,   // 下
    RIGHT = 0b0100,  // 右
    LEFT = 0b1000,   // 左
    NONE = 0b0000    // 无效
};

// 设置某个方向为邻接
static void setDirection(uint8_t& adjacency, Direction dir) {
    adjacency |= dir;  // 按位或，设置对应位为 1
}

// 检查某个方向是否邻接
static bool isDirectionSet(uint8_t adjacency, Direction dir) {
    return adjacency & dir;  // 按位与，检查对应位是否为 1
}
// 清除某个方向的邻接
static void clearDirection(uint8_t& adjacency, Direction dir) {
    adjacency &= ~dir;  // 按位与和按位取反，清除对应位
}

// 获取方向的逆方向
static Direction getOppositeDirection(Direction dir) {
    switch (dir) {
        case UP:
            return DOWN;
        case DOWN:
            return UP;
        case RIGHT:
            return LEFT;
        case LEFT:
            return RIGHT;
        default:
            return NONE;  // 如果传入无效方向，返回 NONE
    }
}

// 解析方向编码，返回邻接方向列表
static std::vector<Direction> parseDirection(uint8_t direction_code) {
    std::vector<Direction> directions;

    if (direction_code & UP) {
        directions.emplace_back(UP);
    }
    if (direction_code & DOWN) {
        directions.emplace_back(DOWN);
    }
    if (direction_code & RIGHT) {
        directions.emplace_back(RIGHT);
    }
    if (direction_code & LEFT) {
        directions.emplace_back(LEFT);
    }
    return directions;
}

static IntPoint getDirection(Direction dir) {
    IntPoint direction(0, 0);
    if (dir & UP) {
        direction.y = 1;
    }
    if (dir & DOWN) {
        direction.y = -1;
    }
    if (dir & RIGHT) {
        direction.x = 1;
    }
    if (dir & LEFT) {
        direction.x = -1;
    }
    return direction;
}

// 定义路径
struct Path {
    std::vector<IntPoint> path;  // 路径上的点，不包含起点和终点
    float path_length;           // 路径长度，从起点到终点的实际距离 - 1
    float path_length_x;
    float path_length_y;
};

// 定义图节点
struct GraphNode {
    using Ptr = std::shared_ptr<GraphNode>;
    using WeakPtr = std::weak_ptr<GraphNode>;
    enum NODE_TYPE { None = 0, Strong = 1, Weak = 2 };

    GraphNode(int x, int y, NODE_TYPE type_)
        : pos(x, y), type(type_), parent() {
        node_state = NOT_EXPAND_GVG;
    }

    // 使用WeakPtr来打破循环引用
    void addNeighbor(WeakPtr neighbor, const std::vector<IntPoint>& path,
                     bool sample_check = true) {
        if (this->type != Strong && !neighbor.lock() &&
            neighbor.lock()->type != Strong) {
            AERROR << "Error: Only strong nodes can have neighbors.";
            return;
        }
        if (sample_check) {
            for (const auto& nb : this->neighbors) {
                if (nb.lock() == neighbor.lock()) {
                    return;
                }
            }
        } else {
            for (size_t i = 0; i < this->neighbors.size(); ++i) {
                auto nb_ptr = this->neighbors[i].lock();
                auto neighbor_ptr = neighbor.lock();
                if (nb_ptr && neighbor_ptr &&
                    nb_ptr->pos == neighbor_ptr->pos) {
                    // 判断路径是否也完全相同
                    if (i < neighbor_paths.size() &&
                        neighbor_paths[i].path == path) {
                        return;
                    }
                }
            }
        }
        Path neighbor_path;
        neighbor_path.path = path;  // 复制路径
        neighbor_path.path_length_x = 0.0f;
        neighbor_path.path_length_y = 0.0f;

        // 计算累计增量
        for (size_t j = 1; j < path.size(); ++j) {
            neighbor_path.path_length_x += std::abs(path[j].x - path[j - 1].x);
            neighbor_path.path_length_y += std::abs(path[j].y - path[j - 1].y);
        }
        // 计算路径长度为欧式距离
        neighbor_path.path_length = std::sqrt(
            neighbor_path.path_length_x * neighbor_path.path_length_x +
            neighbor_path.path_length_y * neighbor_path.path_length_y);
        neighbor_paths.emplace_back(neighbor_path);
        this->neighbors.emplace_back(neighbor);
    }
    IntPoint pos;                      // 节点坐标
    NODE_TYPE type = None;             // 节点类型
    bool IsVisitedStage3 = false;      // 用于Stage3的访问标记
    bool RemovedByPVS = false;         // 用于标记是否被某个阶段删除
    std::vector<WeakPtr> neighbors;    // 邻居节点（使用WeakPtr）
    std::vector<Path> neighbor_paths;  // 到邻居的路径

    enum NODE_STATE {
        IN_CLOSE_SET_GVG = 0,
        IN_OPEN_SET_GVG = 1,
        NOT_EXPAND_GVG = 2
    };
    double g_score = 0.0, f_score = 0.0;  // 用于A*算法
    GraphNode::WeakPtr parent;
    char node_state;
    GraphNode() : parent(), node_state(NOT_EXPAND_GVG) {}
};

class NodeComparator0
{
public:
    bool operator()(GraphNode::Ptr node1, GraphNode::Ptr node2) {
        return node1->f_score > node2->f_score;
    }
};

class NodeHashTable0
{
public:
    /* data */
    std::unordered_map<IntPoint, GraphNode::Ptr, std::hash<IntPoint>> data_2d_;

public:
    NodeHashTable0(/* args */) {}
    ~NodeHashTable0() {}
    void insert(IntPoint idx, GraphNode::Ptr node) {
        // data_2d_.insert(make_pair(idx, node)); // 这种方法不会更新覆盖
        data_2d_[idx] = node;
    }
    GraphNode::Ptr find(const IntPoint& idx) const {
        auto iter = data_2d_.find(idx);
        return iter == data_2d_.end() ? nullptr : iter->second;
    }
    bool contains(const IntPoint& idx) const {
        return data_2d_.find(idx) != data_2d_.end();
    }
    void clear() {
        data_2d_.clear();
    }
};

// 定义扩展结果
struct ExpansionResult {
    Path path;           // 扩展生成的路径
    IntPoint end_point;  // 扩展结束的点
    ExpansionResult() : end_point(-1, -1) {}
};

// GVG 类
class GVG
{
public:
    // 创建图
    void createGraph(const DynamicVoronoi& voronoi);

    // 获取图
    std::vector<std::unordered_map<IntPoint, GraphNode::Ptr>>& getGraphs();

    int getGraphsSize();

    void set_use_EGVG(bool use_egvg);

    void getStrongNodes(std::vector<IntPoint>& strong_nodes) const;

    bool isVoronoi(int x, int y);
    bool isVoronoi(IntPoint pt);

    // 设置clearance_threshold_sq_
    void setClearanceThresholdSq(float threshold_low, float threshold_high);

    const std::vector<IntPoint>& getTruncatedPoints() const;
    const std::vector<IntPoint>& getCompleteConnectionPoints() const;

    GraphNode::Ptr findNodeInGraphs(const IntPoint& pt, int& graph_id);
    GraphNode::Ptr findNodeInGraphs(const IntPoint& pt);

    void insertNodeToGraph(const GraphNode::Ptr& node, int graph_id);

    void removeNodeFromGraph(const IntPoint& pos, int graph_id);
    void removeNodeFromGraph(const IntPoint& pos);

private:
    // 扩展函数。要通过result.path的size()来判断是不是有扩展
    // 根据result.end_point来判断，如果end_point是无效值，则说明扩展失败；然后判断end_point是强or弱grid
    ExpansionResult expandGrid(const IntPoint& start, Direction dir);

    // DFS 搜索
    // 每次从这个节点开始搜索、或者由别的节点搜索到该节点，都要从grid_adjs_中删除掉这个节点的邻接方向
    std::unordered_map<IntPoint, GraphNode::Ptr> DFSSearch(
        const IntPoint& start);

    void DFSSearch2(GraphNode::Ptr& center_node,
                    const std::vector<IntPoint>& cluster_points);

    int getNumVoronoiNeighbors(int x, int y, uint8_t& dir_code);

    void completeConnection(int x, int y, const DynamicVoronoi& voronoi);
    // 封装补全Voronoi点的函数
    void tryCompleteVoronoi(int x, int y, int idx1, int idx2,
                            const DynamicVoronoi& voronoi);

    bool deleteblock(int x, int y);

    bool voronoiAt(int x, int y) const;
    bool voronoiAt(const IntPoint& pt) const;

#ifdef USE_OPENCV
    void drawVoronoiByGraphs(cv::Mat& img) const;
    void drawVoronoiByGrid(cv::Mat& img) const;
#endif  // USE_OPENCV

    enum GRID_TYPE { None = 0, Strong = 1, Weak = 2, Edge = 3 };

    std::vector<std::unordered_map<IntPoint, GraphNode::Ptr>>
        graphs_;  // 图结构
    // std::vector<bool> grid_visited_;        // 访问标记 //
    // 现在不要这个了，全部用grid_adjs_中剩下的邻居方向来表示是否还可以扩展
    std::vector<GRID_TYPE>
        grid_types_;  // 网格类型,
                      // 0表示none，1表示强节点，2表示弱节点，3表示edge
    std::vector<uint8_t> grid_adjs_;          // 邻接方向
    std::vector<uint8_t> grid_adjs_origin_;   // 邻接方向，保留最初数据
    std::vector<IntPoint> truncated_points_;  // 被cle_thr_sq_high_截断的点
    std::vector<IntPoint> completeCoonction_points_;
    std::vector<bool> voronoi_new;  // 在gvg里面单独弄一个紧凑维诺图
    int sizeX_, sizeY_;             // 网格大小
    float cle_thr_sq_low_;  // 单位是网格数，平方，小于这个数的被维诺点不被考虑
    float cle_thr_sq_high_,
        cle_thr_sq_high2_;  // dis大于这个数的维诺点不被考虑，用来构建更加紧凑的维诺图结构
    bool use_EGVG_ = true;

    /*
     * 19 20 21 22 23
     * 17 5  6  7  18
     * 15 3  X  4  16
     * 13 0  1  2  14
     * 8  9  10 11 12
     */
    std::vector<IntPoint> ne_offset_ = {
        IntPoint(-1, -1),  // 0
        IntPoint(0, -1),   // 1
        IntPoint(1, -1),   // 2
        IntPoint(-1, 0),   // 3
        IntPoint(1, 0),    // 4
        IntPoint(-1, 1),   // 5
        IntPoint(0, 1),    // 6
        IntPoint(1, 1),    // 7
        IntPoint(-2, -2),  // 8
        IntPoint(-1, -2),  // 9
        IntPoint(0, -2),   // 10
        IntPoint(1, -2),   // 11
        IntPoint(2, -2),   // 12
        IntPoint(-2, -1),  // 13
        IntPoint(2, -1),   // 14
        IntPoint(-2, 0),   // 15
        IntPoint(2, 0),    // 16
        IntPoint(-2, 1),   // 17
        IntPoint(2, 1),    // 18
        IntPoint(-2, 2),   // 19
        IntPoint(-1, 2),   // 20
        IntPoint(0, 2),    // 21
        IntPoint(1, 2),    // 22
        IntPoint(2, 2)     // 23
    };
};

struct GridNode {
    IntPoint pos;
    double g, f;
    GridNode* parent;
    GridNode(const IntPoint& p, double g_, double f_, GridNode* par)
        : pos(p), g(g_), f(f_), parent(par) {}
};

struct GridNodeCmp {
    bool operator()(const GridNode* a, const GridNode* b) const {
        return a->f > b->f;
    }
};

}  // namespace egvg
}  // namespace planner
}  // namespace planning
}  // namespace autonomy