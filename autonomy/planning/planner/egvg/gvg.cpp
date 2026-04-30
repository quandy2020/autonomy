#include "autonomy/planning/planner/egvg/gvg.hpp"

#include "autolink/common/log.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace egvg {

void GVG::createGraph(const DynamicVoronoi& voronoi) {
    SimpleTimer timer;
    // Simple
    // DynaVoro::SimpleTimer timer;
    timer.reset();
    for (auto& graph : graphs_) {
        for (auto& node_pair : graph) {
            for (auto& neighbor : node_pair.second->neighbors) {
                neighbor.reset();  // 清空邻居指针
            }
            node_pair.second.reset();  // 清空节点指针
        }
    }
    graphs_.clear();
    if (grid_types_.size() != voronoi.getSizeX() * voronoi.getSizeY()) {
        grid_types_.resize(voronoi.getSizeX() * voronoi.getSizeY(),
                           GRID_TYPE::None);
        grid_adjs_.resize(voronoi.getSizeX() * voronoi.getSizeY(), (uint8_t)0);
        voronoi_new.resize(voronoi.getSizeX() * voronoi.getSizeY(), false);
        sizeX_ = voronoi.getSizeX();
        sizeY_ = voronoi.getSizeY();
        AINFO << "[GVG] Init!";
    }
    // std::fill(grid_types_.begin(), grid_types_.end(), GRID_TYPE::None);
    // std::fill(grid_adjs_.begin(), grid_adjs_.end(), (uint8_t)0);
    // std::fill(voronoi_new.begin(), voronoi_new.end(), false);
    truncated_points_.clear();
    completeCoonction_points_.clear();
#ifdef VERBOSE
    timer.print("Step 0: Init");
    timer.reset();
#endif
    // 整体的流程变化了
    // Step 1: 遍历所有点，从原始的voronoi里判断是否为Voronoi点；
    // Step 2:
    // 对于每个Voronoi点，判断其邻接关系，分类为强节点、弱节点、边，然后进行一次GVG化；
    // Step 3：单独处理所有弱节点和对应的强节点，然后删除"寄生"边；
    // Step 4：将剩下的图结构，画到voronoi_new上；
    // Step 5：将等高线加入到voronoi_new上，并补全连接；
    // Step 6：将voronoi_new上所有的Voronoi点，进行一次四方格删除和六方格删除；
    // Step 7：在新的voronoi_new上，再进行一次GVG化，得到最终的图结构。

    // Step 1: 遍历所有点，从原始的voronoi里判断是否为Voronoi点；
    std::vector<IntPoint> strong_grid_vec;
    for (int x = 0; x < sizeX_; ++x) {
        for (int y = 0; y < sizeY_; ++y) {
            if (voronoi.isVoronoiWithDisThr(x, y, cle_thr_sq_low_)) {
                voronoi_new[y * sizeX_ + x] = true;  // 标记为已voronoi点
            } else {
                voronoi_new[y * sizeX_ + x] = false;  // 标记为非voronoi点
            }
        }
    }
#ifdef VERBOSE
    timer.print("Step 1: Voronoi点判断");
    timer.reset();
#endif
    // Step 2:
    // 对于每个Voronoi点，判断其邻接关系，分类为强节点、弱节点、边，然后进行一次GVG化；
    for (int x = 0; x < sizeX_; ++x) {
        for (int y = 0; y < sizeY_; ++y) {
            if (!voronoi_new[y * sizeX_ + x]) {
                continue;  // 跳过非维诺图点
            }
            uint8_t dir_code = 0;
            int neighbors_num = getNumVoronoiNeighbors(x, y, dir_code);
            grid_adjs_[y * sizeX_ + x] = dir_code;
            if (neighbors_num == 1) {
                grid_types_[y * sizeX_ + x] = GRID_TYPE::Weak;  // 弱节点
            } else if (neighbors_num == 2) {
                grid_types_[y * sizeX_ + x] = GRID_TYPE::Edge;  // 边
            } else if (neighbors_num >= 3) {
                grid_types_[y * sizeX_ + x] = GRID_TYPE::Strong;  // 强节点
                strong_grid_vec.emplace_back(x, y);
            } else {  // 这是由于设置了阈值，导致某些地方的连接会断掉
                grid_types_[y * sizeX_ + x] = GRID_TYPE::None;  // none
                voronoi_new[y * sizeX_ + x] = false;  // 标记为非voronoi点
            }
        }
    }

#ifdef DEBUG_IMG
    cv::Mat voronoi_img0(sizeY_, sizeX_, CV_8UC3,
                         cv::Scalar(255, 255, 255));  // 创建白色背景的图像
    drawVoronoiByGrid(voronoi_img0);
#endif
    // // std::cout << "--------在修剪后的voronoi图上进行搜索---------" <<
    // std::endl; stage_ = 1;
    for (int i = 0; i < static_cast<int>(strong_grid_vec.size()); ++i) {
        IntPoint start = strong_grid_vec[i];
        if (parseDirection(grid_adjs_[start.y * sizeX_ + start.x]).empty()) {
            continue;  // 已访问，跳过
        }
        // DFS 搜索
        graphs_.emplace_back(DFSSearch(start));
    }
#ifdef VERBOSE
    timer.print("Step 2: DFS stage-1");
    timer.reset();
#endif
    // return;
    // Step 3：单独处理所有弱节点和对应的强节点，然后删除"寄生"边；
    // 这里测试将弱节点全部都删除，以删除寄生边，从而让图变得更简洁
    // 首先遍历所有的week_node，将其邻接的strong_node加入到队列里面去
    // 然后遍历到strong_node的时候，检查其邻接的状态：
    // **如果是4邻接，先不处理；
    // **如果是3邻接，检查其邻接的weak_node数量：
    // ***如果是1个，则不处理；
    // ***如果是2个，且当前strong_node的dis小于阈值，则将其标记被week_node，并将其邻接strong_node加入队列
    // ***如果是2个，且当前strong_node的dis大于阈值，则其肯定还有一个strong_node邻接
    // 判断当前节点和那个stong_node的角度关系，如果角度小于阈值，则将该节点标记为week_node，并将其邻接strong_node加入队列
    // ***如果是3个，则直接标记为weak_node
    int skip_border = 10;  // 在四个角上的弱节点不处理
    std::queue<GraphNode::Ptr> strong_node_queue;
    for (const auto& graph : graphs_) {
        for (const auto& node : graph) {
            if (node.second->type == GraphNode::Weak) {
                auto pos = node.second->pos;
                if ((pos.x < skip_border &&
                     (pos.y < skip_border ||
                      pos.y >= sizeY_ - skip_border)) ||  // 左上/左下
                    (pos.x >= sizeX_ - skip_border &&
                     (pos.y < skip_border ||
                      pos.y >= sizeY_ - skip_border))  // 右上/右下
                ) {
                    node.second->type = GraphNode::Strong;
                    continue;
                }
                // 将邻接的强节点加入队列
                for (const auto& neighbor : node.second->neighbors) {
                    if (auto locked_neighbor = neighbor.lock()) {
                        if (locked_neighbor->type == GraphNode::Strong) {
                            strong_node_queue.push(locked_neighbor);
                        }
                    }
                }
            }
        }
    }
    // 遍历强节点队列
    int neighbor_count = 0;
    int neighbor_week_count = 0;
    std::vector<int> neighbor_strong_idx;
    while (!strong_node_queue.empty()) {
        auto strong_node_pt = strong_node_queue.front();
        strong_node_queue.pop();
        if (strong_node_pt->type != GraphNode::Strong)
            continue;  // 只处理强节点
        neighbor_strong_idx.clear();
        neighbor_count = static_cast<int>(strong_node_pt->neighbors.size());
        neighbor_week_count = neighbor_count;
        for (int i = 0; i < neighbor_count; ++i) {
            auto neighbor_ptr = strong_node_pt->neighbors[i].lock();
            if (neighbor_ptr && neighbor_ptr->type == GraphNode::Strong) {
                neighbor_strong_idx.emplace_back(i);
                --neighbor_week_count;
            }
        }
        if (neighbor_count == 4) {
            if (neighbor_week_count == 4)
                strong_node_pt->type = GraphNode::Weak;
            else
                continue;
        } else if (neighbor_count == 3) {
            if (neighbor_week_count == 3)
                continue;
            if (neighbor_week_count == 1)
                continue;
            // if (strong_node_pt->pos.x == 1002 && strong_node_pt->pos.y ==
            // 760) {
            //     int debug = 0; // debug
            // }
            if (neighbor_week_count == 2) {
                if (voronoi.getDistance(strong_node_pt->pos.x,
                                        strong_node_pt->pos.y) <
                    2.0 * std::sqrt(cle_thr_sq_low_)) {
                    // 将其标记为弱节点
                    strong_node_pt->type = GraphNode::Weak;
                    // 将邻接的强节点加入队列
                    for (int i = 0;
                         i < static_cast<int>(neighbor_strong_idx.size());
                         ++i) {
                        auto neighbor_strong_lock =
                            strong_node_pt->neighbors[neighbor_strong_idx[i]]
                                .lock();
                        if (neighbor_strong_lock)
                            strong_node_queue.push(neighbor_strong_lock);
                    }
                } else {
                    // 使用theta_SMA方法来判断
                    for (int i = 0;
                         i < static_cast<int>(neighbor_strong_idx.size());
                         ++i) {
                        auto neighbor_strong_pt =
                            strong_node_pt->neighbors[neighbor_strong_idx[i]]
                                .lock();
                        IntPoint dir1 =
                            neighbor_strong_pt->pos - strong_node_pt->pos;
                        IntPoint neighbor_parent = IntPoint(
                            voronoi.getObstacleX(neighbor_strong_pt->pos.x,
                                                 neighbor_strong_pt->pos.y),
                            voronoi.getObstacleY(neighbor_strong_pt->pos.x,
                                                 neighbor_strong_pt->pos.y));
                        IntPoint dir2 = neighbor_parent - strong_node_pt->pos;
                        // 计算两个dir的夹角
                        float angle_cos =
                            (dir1.x * dir2.x + dir1.y * dir2.y) /
                            (std::sqrt(dir1.x * dir1.x + dir1.y * dir1.y) *
                             std::sqrt(dir2.x * dir2.x + dir2.y * dir2.y));
                        if (angle_cos < -0.5) {  // 夹角大于120度
                            // 将其标记为弱节点
                            strong_node_pt->type = GraphNode::Weak;
                            // 将邻接的强节点加入队列
                            for (int j = 0; j < static_cast<int>(
                                                    neighbor_strong_idx.size());
                                 ++j) {
                                strong_node_queue.push(
                                    strong_node_pt
                                        ->neighbors[neighbor_strong_idx[j]]
                                        .lock());
                            }
                        }
                    }
                }
            }
        }
    }
#ifdef VERBOSE
    timer.print("Step 3: 删除寄生边");
    timer.reset();
#endif
    // Step 4：将剩下的图结构，画到voronoi_new上；
    // std::fill(grid_types_.begin(), grid_types_.end(), GRID_TYPE::None);
    // std::fill(grid_adjs_.begin(), grid_adjs_.end(), (uint8_t)0);
    // TODO：这里是不是可以通过判断哪些node被遍历过了，从而跳过，来减少遍历的计算量。
    std::fill(voronoi_new.begin(), voronoi_new.end(), false);
    for (size_t graph_idx = 0; graph_idx < graphs_.size(); ++graph_idx) {
        const auto& graph = graphs_[graph_idx];
        for (const auto& node_pair : graph) {
            const auto& node_ptr = node_pair.second;
            if (node_ptr->type == GraphNode::Weak)
                continue;
            const IntPoint& node_pos = node_ptr->pos;
            std::vector<Path> paths = node_ptr->neighbor_paths;
            voronoi_new[node_ptr->pos.x + node_ptr->pos.y * sizeX_] = true;
            for (int i = 0; i < static_cast<int>(paths.size()); ++i) {
                const auto& path = paths[i];
                if (node_ptr->neighbors[i].lock()->type == GraphNode::Weak)
                    continue;
                for (size_t j = 0; j < path.path.size(); ++j) {
                    voronoi_new[path.path[j].y * sizeX_ + path.path[j].x] =
                        true;  // 标记为Voronoi起点
                }
            }
        }
    }
#ifdef VERBOSE
    timer.print("Step 4: 画出之前的voronoi_new");
    timer.reset();
#endif
    graphs_.clear();

#ifdef DEBUG_IMG
    cv::Mat voronoi_img1(sizeY_, sizeX_, CV_8UC3,
                         cv::Scalar(255, 255, 255));  // 创建白色背景的图像
    drawVoronoiByGrid(voronoi_img1);
#endif

    std::vector<IntPoint> strong_grid_vec2;
    // Step 5：将等高线加入到voronoi_new上，并补全连接；//
    // 把下面的注释，就是不添加等高线。
    if (use_EGVG_) {
        std::queue<IntPoint> voronoi_pt_queue;
        for (int x = 0; x < sizeX_; ++x) {
            for (int y = 0; y < sizeY_; ++y) {
                float dis_sq = voronoi.getDistanceSq(x, y);
                if (voronoi_new[y * sizeX_ + x] == true) {
                    if (x >= 2 && x < sizeX_ - 2 && y >= 2 && y < sizeY_ - 2) {
                        voronoi_pt_queue.emplace(x, y);  //
                    }
                }
                // 等高线点
                if (dis_sq >= cle_thr_sq_high_ - 1e-3 &&
                    dis_sq <= cle_thr_sq_high2_ - 1e-3) {
                    truncated_points_.emplace_back(x, y);  // 记录被截断的点
                    voronoi_new[y * sizeX_ + x] = true;    // 标记为已voronoi点
                    if (x >= 2 && x < sizeX_ - 2 && y >= 2 && y < sizeY_ - 2) {
                        voronoi_pt_queue.emplace(x, y);  //
                    }
                }
            }
        }
#ifdef DEBUG_IMG
        drawVoronoiByGrid(voronoi_img0);
#endif
        for (const auto& point : truncated_points_) {
            completeConnection(point.x, point.y, voronoi);
        }
        for (const auto& point : completeCoonction_points_) {
            voronoi_pt_queue.emplace(point);  // 将补全连接的点加入队列
        }
#ifdef VERBOSE
        timer.print("Step 5: 添加等高线");
        timer.reset();
#endif
        // 这里我要把voronoi_new里面被判断为维诺图点的点都记录下来，然后遍历一次来去除四方格
        while (!voronoi_pt_queue.empty()) {
            auto voronoi_pt = voronoi_pt_queue.front();
            voronoi_pt_queue.pop();
            bool retry = deleteblock(voronoi_pt.x, voronoi_pt.y);
            if (0) {
                voronoi_pt_queue.push(voronoi_pt);  // 如果删除失败，重新入队
            }
        }
    }
    // 添加等高线的注释到这里
#ifdef VERBOSE
    timer.print("Step 6: 删除四邻域和六邻域");
    timer.reset();
#endif

    for (int x = 0; x < sizeX_; ++x) {
        for (int y = 0; y < sizeY_; ++y) {
            if (!voronoi_new[y * sizeX_ + x]) {
                continue;  // 跳过非维诺图点
            }
            uint8_t dir_code = 0;
            int neighbors_num = getNumVoronoiNeighbors(x, y, dir_code);
            grid_adjs_[y * sizeX_ + x] = dir_code;
            if (neighbors_num == 1) {
                grid_types_[y * sizeX_ + x] = GRID_TYPE::Weak;  // 弱节点
            } else if (neighbors_num == 2) {
                grid_types_[y * sizeX_ + x] = GRID_TYPE::Edge;  // 边
            } else if (neighbors_num >= 3) {
                grid_types_[y * sizeX_ + x] = GRID_TYPE::Strong;  // 强节点
                strong_grid_vec2.emplace_back(x, y);
            } else {
                grid_types_[y * sizeX_ + x] = GRID_TYPE::None;  // none
                voronoi_new[y * sizeX_ + x] = false;  // 标记为非voronoi点
            }
        }
    }

#ifdef DEBUG_IMG
    cv::Mat voronoi_img2(sizeY_, sizeX_, CV_8UC3,
                         cv::Scalar(255, 255, 255));  // 创建白色背景的图像
    drawVoronoiByGrid(voronoi_img2);
#endif

    // std::cout << "--------在加入等高线后的voronoi图上进行搜索---------" <<
    // std::endl; stage_ = 2;
    for (int i = 0; i < static_cast<int>(strong_grid_vec2.size()); ++i) {
        IntPoint start = strong_grid_vec2[i];
        if (parseDirection(grid_adjs_[start.y * sizeX_ + start.x]).empty()) {
            continue;  // 已访问，跳过
        }
        // DFS 搜索
        graphs_.emplace_back(DFSSearch(start));
    }

#ifdef VERBOSE
    timer.print("Step 7: DFS Stage-2");
    timer.reset();
#endif

    // 使用 DBScan2D 对强节点进行聚类。
    // DBScan2D 期望输入为 geometry_msgs::Point，因此需要将 IntPoint 转换。
    std::vector<autonomy::commsgs::geometry_msgs::Point> cluster_input_points;
    cluster_input_points.reserve(strong_grid_vec2.size());
    for (const auto& ip : strong_grid_vec2) {
        autonomy::commsgs::geometry_msgs::Point p;
        p.x = static_cast<double>(ip.x);
        p.y = static_cast<double>(ip.y);
        p.z = 0.0;
        cluster_input_points.push_back(p);
    }

    DBScan2D dbscan(cluster_input_points, 4.3f, 2);  // 最大邻域判断点为(2, 1)
    std::vector<std::vector<int>> clusters;
    int num_clusters = dbscan.run(clusters);

#ifdef VERBOSE
    timer.print("Step 8.1: DBSCAN");
    timer.reset();
#endif

#ifdef DEBUG_IMG
    drawVoronoiByGrid(voronoi_img2);
#endif

    for (int i = 0; i < num_clusters; ++i) {
        std::vector<IntPoint> cluster_points;
        for (int j = 0; j < static_cast<int>(clusters[i].size()); ++j) {
            int idx = clusters[i][j];
            IntPoint pt = strong_grid_vec2[idx];
            cluster_points.emplace_back(pt);
        }
        // 这里进行一系列检查
        // cluster_points: 当前聚类所有点
        std::vector<IntPoint> pre_outside_strong_neighbors;
        for (const auto& pt : cluster_points) {
            auto node = findNodeInGraphs(pt);
            if (!node)
                continue;
            for (const auto& nb : node->neighbors) {
                auto nb_ptr = nb.lock();
                if (!nb_ptr)
                    continue;
                // 不在聚类内，且是强节点
                if (std::find(cluster_points.begin(), cluster_points.end(),
                              nb_ptr->pos) == cluster_points.end() &&
                    nb_ptr->type == GraphNode::Strong) {
                    pre_outside_strong_neighbors.push_back(nb_ptr->pos);
                }
            }
        }
        // 去重
        std::sort(pre_outside_strong_neighbors.begin(),
                  pre_outside_strong_neighbors.end());
        pre_outside_strong_neighbors.erase(
            std::unique(pre_outside_strong_neighbors.begin(),
                        pre_outside_strong_neighbors.end()),
            pre_outside_strong_neighbors.end());

        // 把第一个点作为中心点
        auto center_node = findNodeInGraphs(cluster_points[0]);

        if (!center_node)
            continue;

        DFSSearch2(center_node, cluster_points);
        // 简化后
        std::vector<IntPoint> center_neighbors;
        for (const auto& nb : center_node->neighbors) {
            auto nb_ptr = nb.lock();
            if (!nb_ptr)
                continue;
            if (nb_ptr->type != GraphNode::Strong)
                continue;
            if ((nb_ptr->pos == center_node->pos))
                continue;  // 这里可能碰到中心节点自成环的情况。
            center_neighbors.push_back(nb_ptr->pos);
            // 检查是否有聚类内邻居
            if (std::find(cluster_points.begin(), cluster_points.end(),
                          nb_ptr->pos) != cluster_points.end()
                // && !(nb_ptr->pos == center_node->pos)
            ) {
                // TODO: 这个地方还是有点错误。
                AERROR << "Error: Center node still has cluster-internal "
                          "neighbor: "
                       << nb_ptr->pos.x << "," << nb_ptr->pos.y;
            }
        }
        // 去重
        std::sort(center_neighbors.begin(), center_neighbors.end());
        center_neighbors.erase(
            std::unique(center_neighbors.begin(), center_neighbors.end()),
            center_neighbors.end());

        // 检查是否完全覆盖
        if (center_neighbors == pre_outside_strong_neighbors) {
            // AINFO << "Check passed: Center node neighbors match
            // pre-simplification outside strong neighbors.";
        } else {
            AERROR << "Check failed: Center node neighbors do not match!";
        }
        for (const auto& pt : cluster_points) {
            if (pt == center_node->pos)
                continue;
            auto node = findNodeInGraphs(pt);
            if (!node)
                continue;
            if (!node->neighbors.empty()) {
                AERROR << "Error: Non-center cluster node " << pt.x << ","
                       << pt.y << " still has neighbors!";
            }
        }
    }

    // 重新生成voronoi_new
    std::fill(voronoi_new.begin(), voronoi_new.end(), false);
    for (int i = 0; i < static_cast<int>(graphs_.size()); ++i) {
        auto graph = graphs_[i];
        for (auto& node : graph) {
            auto node_ptr = node.second;
            int x = node_ptr->pos.x, y = node_ptr->pos.y;
            if (node_ptr->type == GraphNode::Strong)
                voronoi_new[y * sizeX_ + x] = true;
            if (node_ptr->type == GraphNode::Weak &&
                node_ptr->neighbor_paths[0].path_length > 50) {
                // 如果是弱节点，但是路径长度大于200，则认为是强节点
                node_ptr->type = GraphNode::Strong;  // 将其标记为强节点
                voronoi_new[y * sizeX_ + x] = true;  // 标记为Voronoi起点
            } else if (node_ptr->type == GraphNode::Weak) {
                continue;  // 弱节点不处理
            }
            if (node_ptr->type == GraphNode::None) {
                this->removeNodeFromGraph(node_ptr->pos);
            }

            const std::vector<Path>& paths = node_ptr->neighbor_paths;
            for (const auto& path : paths) {
                if (path.path.empty())
                    continue;
                for (size_t j = 0; j < path.path.size(); ++j) {
                    voronoi_new[sizeX_ * path.path[j].y + path.path[j].x] =
                        true;
                }
            }
        }
    }

#ifdef DEBUG_IMG
    cv::Mat voronoi_img3(sizeY_, sizeX_, CV_8UC3, cv::Scalar(255, 255, 255));
    drawVoronoiByGraphs(voronoi_img3);
#endif

#ifdef VERBOSE
    timer.print("Step 8.2: 删除冗余强节点");
    timer.reset();
#endif

    int debug = 0;
    (void)debug;
}

std::vector<std::unordered_map<IntPoint, GraphNode::Ptr>>& GVG::getGraphs() {
    return graphs_;
}

int GVG::getGraphsSize() {
    return static_cast<int>(graphs_.size());
}

void GVG::set_use_EGVG(bool use_egvg) {
    use_EGVG_ = use_egvg;
}

void GVG::getStrongNodes(std::vector<IntPoint>& strong_nodes) const {
    strong_nodes.clear();
    for (const auto& graph : graphs_) {
        for (const auto& node_pair : graph) {
            const auto& node_ptr = node_pair.second;
            IntPoint node_pos = node_ptr->pos;
            if (node_ptr->type == GraphNode::Strong) {
                strong_nodes.emplace_back(node_pos);
            }
        }
    }
}

bool GVG::isVoronoi(int x, int y) {
    return voronoi_new[y * sizeX_ + x];
}

bool GVG::isVoronoi(IntPoint pt) {
    return voronoi_new[pt.y * sizeX_ + pt.x];
}

void GVG::setClearanceThresholdSq(float threshold_low, float threshold_high) {
    cle_thr_sq_low_ = threshold_low;
    cle_thr_sq_high_ = threshold_high;
    cle_thr_sq_high2_ =
        (std::sqrt(threshold_high) + 1.0f) * (std::sqrt(threshold_high) + 1.0f);
}

const std::vector<IntPoint>& GVG::getTruncatedPoints() const {
    return truncated_points_;
}

const std::vector<IntPoint>& GVG::getCompleteConnectionPoints() const {
    return completeCoonction_points_;
}

GraphNode::Ptr GVG::findNodeInGraphs(const IntPoint& pt, int& graph_id) {
    if (graph_id == -1) {
        // 在所有图中查找
        for (size_t i = 0; i < graphs_.size(); ++i) {
            auto& graph = graphs_[i];
            auto it = graph.find(pt);
            if (it != graph.end()) {
                graph_id = static_cast<int>(i);  // 传出实际命中的id
                return it->second;
            }
        }
        graph_id = -1;  // 未找到
    } else if (graph_id >= 0 && graph_id < static_cast<int>(graphs_.size())) {
        // 只在指定图查找
        auto& graph = graphs_[graph_id];
        auto it = graph.find(pt);
        if (it != graph.end()) {
            return it->second;
        }
        // graph_id = -1; // 未找到
    } else {
        graph_id = -1;  // 非法id
    }
    return nullptr;
}

GraphNode::Ptr GVG::findNodeInGraphs(const IntPoint& pt) {
    // 在所有图中查找
    for (auto& graph : graphs_) {
        auto it = graph.find(pt);
        if (it != graph.end()) {
            return it->second;
        }
    }
    return nullptr;
}

void GVG::insertNodeToGraph(const GraphNode::Ptr& node, int graph_id) {
    if (!node) {
        return;
    }
    if (graph_id < 0 || graph_id >= static_cast<int>(graphs_.size())) {
        AERROR << "insertNodeToGraph: invalid graph_id " << graph_id;
        return;
    }
    if (findNodeInGraphs(node->pos, graph_id)) {
        AERROR << "insertNodeToGraph: node already exists in graph "
               << graph_id;
        return;
    }
    graphs_[graph_id][node->pos] = node;
}

void GVG::removeNodeFromGraph(const IntPoint& pos, int graph_id) {
    if (graph_id < 0 || graph_id >= static_cast<int>(graphs_.size())) {
        AERROR << "removeNodeFromGraph: invalid graph_id " << graph_id;
        return;
    }
    graphs_[graph_id].erase(pos);
}

void GVG::removeNodeFromGraph(const IntPoint& pos) {
    for (size_t i = 0; i < graphs_.size(); ++i) {
        auto& graph = graphs_[i];
        auto it = graph.find(pos);
        if (it != graph.end()) {
            graph.erase(it);
            return;
        }
    }
}

// 私有函数实现

ExpansionResult GVG::expandGrid(const IntPoint& start, Direction dir) {
    ExpansionResult result;
    // result.path.path.emplace_back(start); //
    // start和end都不加入到path里面，这样是方便结果路径的生成
    result.path.path_length = 0.0f;
    result.path.path_length_x = 0.0f;
    result.path.path_length_y = 0.0f;

    IntPoint current = start;
    while (true) {
        // 计算下一个点
        IntPoint direction = getDirection(dir);
        IntPoint next(current.x + direction.x, current.y + direction.y);

        int next_idx = next.y * sizeX_ + next.x;
        int next_type = grid_types_[next_idx];
        uint8_t& next_adjs = grid_adjs_[next_idx];
        int next_visited = parseDirection(next_adjs).size();

        // 如果是空节点，跳过
        // 如果是成环的情况，这里会出现特殊情况，直接忽略就可以了
        if (next_type == 0) {
            // std::cout << "ERROR! 应该不会返回维诺图以外的节点. Stage: " <<
            // stage_
            // << std::endl;
            break;
        }
        if (next_visited == 0) {
            // std::cout << "ERROR! 应该不会出现next_visited = 0的情况. Stage: "
            // << stage_ << std::endl;
            break;
        }

        // 如果排除了上述的情况，那么下面一定是有一个有效的节点
        // 那就直接先减去现在的方向
        clearDirection(next_adjs, getOppositeDirection(dir));

        if (next_type == 1 || next_type == 2) {
            // 如果是强节点或弱节点，直接返回
            result.end_point = next;
            break;
        }

        if (next_type == 3) {
            result.path.path.emplace_back(next);
            result.path.path_length_x += std::fabs(direction.x);
            result.path.path_length_y += std::fabs(direction.y);
            current = next;
            // direction需要更新
            std::vector<Direction> next_dir = parseDirection(next_adjs);
            if (next_dir.size() == 1) {
                dir = next_dir[0];
            }
        }
    }
    result.path.path_length =
        std::sqrt(result.path.path_length_x * result.path.path_length_x +
                  result.path.path_length_y * result.path.path_length_y);
    return result;
}

std::unordered_map<IntPoint, GraphNode::Ptr> GVG::DFSSearch(
    const IntPoint& start) {
    std::unordered_map<IntPoint, GraphNode::Ptr> local_graph;  // 局部图结构
    std::stack<IntPoint> stack;                                // 使用栈代替队列
    stack.push(start);

    while (!stack.empty()) {
        IntPoint current = stack.top();  // 获取栈顶元素
        stack.pop();                     // 弹出栈顶元素
        // if (current.x == 250 && current.y == 217 && stage_ == 2) {
        //     int debug = 0;
        // }

        int current_idx = current.y * sizeX_ + current.x;
        int current_type = grid_types_[current_idx];

        if (current_type != 1 && current_type != 2) {
            AERROR << "ERROR! 只有强or弱 grid可以作为graph node.";
        }

        // 创建当前节点
        GraphNode::Ptr current_node;
        if (local_graph.find(current) != local_graph.end()) {
            current_node = local_graph[current];  // 如果已经存在，直接使用
        } else {
            // 创建新的节点
            current_node = std::make_shared<GraphNode>(
                current.x, current.y,
                current_type == 1 ? GraphNode::Strong : GraphNode::Weak);
            local_graph[current] = current_node;
        }

        // 遍历当前节点的邻接方向
        uint8_t& adj_dir = grid_adjs_[current_idx];
        std::vector<Direction> directions = parseDirection(adj_dir);

        for (Direction dir : directions) {
            clearDirection(adj_dir, dir);
            // 扩展到邻接节点
            ExpansionResult result = expandGrid(current, dir);

            // 如果扩展失败，跳过
            if (result.end_point.x == -1 && result.end_point.y == -1) {
                continue;
            }

            int next_idx = result.end_point.y * sizeX_ + result.end_point.x;
            int next_type = grid_types_[next_idx];
            if (next_type != 1 && next_type != 2) {
                AERROR << "ERROR! 只有强or弱 grid可以作为end_point.";
            }

            // 创建终点节点
            GraphNode::Ptr end_node;
            if (local_graph.find(result.end_point) != local_graph.end()) {
                end_node =
                    local_graph[result.end_point];  // 如果已经存在，直接使用
            } else {
                // 创建新的节点
                end_node = std::make_shared<GraphNode>(
                    result.end_point.x, result.end_point.y,
                    next_type == 1 ? GraphNode::Strong : GraphNode::Weak);
                local_graph[result.end_point] = end_node;
            }
            if (end_node->pos == current_node->pos) {
                // 如果终点和起点是同一个节点，跳过
                continue;
            }

            bool found = false;
            for (size_t i = 0; i < current_node->neighbors.size(); ++i) {
                auto nb_ptr = current_node->neighbors[i].lock();
                if (nb_ptr == end_node) {
                    found = true;
                    // 如果新路径更短，则更新
                    if (result.path.path_length <
                        current_node->neighbor_paths[i].path_length) {
                        current_node->neighbor_paths[i] = result.path;
                        // 反向也更新
                        for (size_t j = 0; j < end_node->neighbors.size();
                             ++j) {
                            auto back_ptr = end_node->neighbors[j].lock();
                            if (back_ptr == current_node) {
                                Path reverse_path = result.path;
                                std::reverse(reverse_path.path.begin(),
                                             reverse_path.path.end());
                                end_node->neighbor_paths[j] = reverse_path;
                                break;
                            }
                        }
                    }
                    break;
                }
            }

            if (!found) {
                // 添加边到 current_node 的邻居列表
                current_node->neighbors.emplace_back(end_node);
                current_node->neighbor_paths.emplace_back(result.path);
                // 添加边到 end_node 的邻居列表（双向边）
                end_node->neighbors.emplace_back(current_node);
                // 创建反向路径
                Path reverse_path = result.path;
                std::reverse(reverse_path.path.begin(),
                             reverse_path.path.end());  // 反转路径
                end_node->neighbor_paths.emplace_back(reverse_path);
            }

            // 如果终点是弱节点，不继续扩展
            if (next_type == GraphNode::Weak) {
                continue;
            } else {
                stack.push(result.end_point);
            }
        }
    }
    return local_graph;
}

void GVG::DFSSearch2(GraphNode::Ptr& center_node,
                     const std::vector<IntPoint>& cluster_points) {
    // 不用标记所有聚类内节点未访问，因为初始化就是false
    using PathPair = std::pair<GraphNode::Ptr, std::vector<IntPoint>>;
    std::stack<PathPair> stack;
    stack.push({center_node, {}});

    while (!stack.empty()) {
        auto [cur_node, path] = stack.top();
        stack.pop();
        cur_node->IsVisitedStage3 = true;

        for (size_t i = 0; i < cur_node->neighbors.size(); ++i) {
            auto nb_ptr = cur_node->neighbors[i].lock();
            if (!nb_ptr)
                continue;
            if (nb_ptr->type == GraphNode::Weak)
                continue;  // 跳过弱节点。这样可能会把弱节点的连接直接断开。应该没有什么关系吧

            std::vector<IntPoint> new_path = path;
            new_path.insert(new_path.end(),
                            cur_node->neighbor_paths[i].path.begin(),
                            cur_node->neighbor_paths[i].path.end());

            // 判断是否在聚类内
            if (std::find(cluster_points.begin(), cluster_points.end(),
                          nb_ptr->pos) != cluster_points.end()) {
                if (nb_ptr->IsVisitedStage3)
                    continue;
                new_path.push_back(nb_ptr->pos);
                stack.push({nb_ptr, new_path});
            } else {
                // 只用addNeighbor即可，内部已做重复判断
                center_node->addNeighbor(nb_ptr, new_path, false);
                // 反向也加
                nb_ptr->addNeighbor(
                    center_node,
                    std::vector<IntPoint>(new_path.rbegin(), new_path.rend()),
                    false);
            }
        }
    }
    // 清理聚类内非中心节点的邻接关系，并解除聚类外邻居对这些节点的连接
    for (const auto& pt : cluster_points) {
        if (pt == center_node->pos)
            continue;
        auto node = findNodeInGraphs(pt);
        if (!node)
            continue;

        // 1. 先清除所有聚类外邻居对该节点的连接
        for (auto& nb_weak : node->neighbors) {
            auto nb_ptr = nb_weak.lock();
            if (!nb_ptr)
                continue;
            // 如果邻居不在聚类内（即为聚类外邻居）
            if (std::find(cluster_points.begin(), cluster_points.end(),
                          nb_ptr->pos) == cluster_points.end()) {
                // 在邻居的neighbors中移除对node的连接
                for (size_t i = 0; i < nb_ptr->neighbors.size();) {
                    auto back_ptr = nb_ptr->neighbors[i].lock();
                    if (back_ptr && back_ptr->pos == pt) {
                        nb_ptr->neighbors.erase(nb_ptr->neighbors.begin() + i);
                        nb_ptr->neighbor_paths.erase(
                            nb_ptr->neighbor_paths.begin() + i);
                    } else {
                        ++i;
                    }
                }
            }
        }

        // 2. 清除该节点自己的邻接关系
        node->neighbors.clear();
        node->neighbor_paths.clear();
        node->type = GraphNode::None;
    }

    // 解除中心节点到其他聚类内节点的邻居关系
    std::vector<size_t> remove_indices;
    for (size_t i = 0; i < center_node->neighbors.size(); ++i) {
        auto nb_ptr = center_node->neighbors[i].lock();
        if (!nb_ptr)
            continue;
        if (nb_ptr->pos == center_node->pos)
            continue;  // 跳过中心节点自身.这是由于有时候成环造成的
        if (std::find(cluster_points.begin(), cluster_points.end(),
                      nb_ptr->pos) != cluster_points.end() &&
            !(nb_ptr->pos == center_node->pos)) {
            remove_indices.push_back(i);
        }
    }
    for (auto it = remove_indices.rbegin(); it != remove_indices.rend(); ++it) {
        center_node->neighbors.erase(center_node->neighbors.begin() + *it);
        center_node->neighbor_paths.erase(center_node->neighbor_paths.begin() +
                                          *it);
    }
}

int GVG::getNumVoronoiNeighbors(int x, int y, uint8_t& dir_code) {
    int count = 0;
    dir_code = 0;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            // 这是四邻域，八邻域的直接跳过
            if ((dx == 0 && dy == 0) || (dx != 0 && dy != 0))
                continue;

            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || nx >= sizeX_ || ny < 0 || ny >= sizeY_)
                continue;

            if (voronoi_new[ny * sizeX_ + nx])  // 如果是voronoi点
            {
                ++count;
                // 设置邻接方向
                if (dx == -1 && dy == 0)
                    setDirection(dir_code, LEFT);
                else if (dx == 1 && dy == 0)
                    setDirection(dir_code, RIGHT);
                else if (dx == 0 && dy == -1)
                    setDirection(dir_code, DOWN);  // 注意，Y轴朝上
                else if (dx == 0 && dy == 1)
                    setDirection(dir_code, UP);
            }
        }
    }
    return count;
}

void GVG::completeConnection(int x, int y, const DynamicVoronoi& voronoi) {
    if (x == 382 && y == 600 - 441) {
        int debug = 0;  // 调试用
        (void)debug;
    }
    // implementation of connectivity patterns
    bool f[8];

    int nx, ny;
    int dx, dy;

    int i = 0;
    int count = 0;
    //  int obstacleCount=0;
    int voroCount = 0;
    int voroCountFour = 0;

    // 只取前8个为8邻域
    for (int k = 0; k < 8; ++k) {
        int nx = x + ne_offset_[k].x;
        int ny = y + ne_offset_[k].y;
        bool isVoronoi = voronoi_new[ny * sizeX_ + nx];
        f[k] = isVoronoi;
        if (isVoronoi) {
            ++voroCount;
            // 四邻域：k==1,3,4,6
            if (k == 1 || k == 3 || k == 4 || k == 6)
                ++voroCountFour;
        }
        if (isVoronoi && (k == 1 || k == 3 || k == 4 || k == 6))
            ++count;
    }
    (void)nx;
    (void)ny;
    (void)dx;
    (void)dy;
    (void)i;
    (void)count;
    (void)voroCountFour;
    /*
     * 5 6 7
     * 3   4
     * 0 1 2
     */
    if (f[0] && (!f[1] && !f[3])) {
        tryCompleteVoronoi(x, y, 1, 3, voronoi);
    }
    if (f[2] && (!f[1] && !f[4])) {
        tryCompleteVoronoi(x, y, 1, 4, voronoi);
    }
    if (f[5] && (!f[3] && !f[6])) {
        tryCompleteVoronoi(x, y, 3, 6, voronoi);
    }
    if (f[7] && (!f[6] && !f[4])) {
        tryCompleteVoronoi(x, y, 6, 4, voronoi);
    }
}

void GVG::tryCompleteVoronoi(int x, int y, int idx1, int idx2,
                             const DynamicVoronoi& voronoi) {
    int x1 = x + ne_offset_[idx1].x;
    int y1 = y + ne_offset_[idx1].y;
    int x2 = x + ne_offset_[idx2].x;
    int y2 = y + ne_offset_[idx2].y;
    if (voronoi.getDistanceSq(x1, y1) > cle_thr_sq_high_) {
        voronoi_new[y1 * sizeX_ + x1] = true;
        completeCoonction_points_.emplace_back(x1, y1);
    } else if (voronoi.getDistanceSq(x2, y2) > cle_thr_sq_high_) {
        voronoi_new[y2 * sizeX_ + x2] = true;
        completeCoonction_points_.emplace_back(x2, y2);
    }
}

bool GVG::deleteblock(int x, int y) {
    if (x == 461 && y == 472) {
        int debug = 0;  // 调试用
        (void)debug;
    }
    bool f[8];

    int nx, ny;
    int dx, dy;
    IntPoint pt_curr(x, y);
    int i = 0;
    int count = 0;
    //  int obstacleCount=0;
    int voroCount = 0;
    int voroCountFour = 0;

    // 只取前8个为8邻域
    for (int k = 0; k < 8; ++k) {
        int nx = x + ne_offset_[k].x;
        int ny = y + ne_offset_[k].y;
        bool isVoronoi = voronoi_new[ny * sizeX_ + nx];
        f[k] = isVoronoi;
        if (isVoronoi) {
            ++voroCount;
            // 四邻域：k==1,3,4,6
            if (k == 1 || k == 3 || k == 4 || k == 6)
                ++voroCountFour;
        }
        // if (isVoronoi && (k == 1 || k == 3 || k == 4 || k == 6))
        //     ++count;
    }
    (void)nx;
    (void)ny;
    (void)dx;
    (void)dy;
    (void)i;
    (void)count;
    (void)voroCountFour;

    // 这里要先处理6方格的情况
    if (voroCount >= 5) {
        if ((f[0] && f[1] && f[2] && f[3] && f[4] && !f[6]) ||
            (f[3] && f[4] && f[5] && f[6] && f[7] && !f[1]) ||
            (f[0] && f[1] && f[3] && f[5] && f[6] && !f[4]) ||
            (f[1] && f[2] && f[4] && f[6] && f[7] && !f[3])) {
            // 6方格的情况，直接删除当前节点
            voronoi_new[(pt_curr.y) * sizeX_ + (pt_curr.x)] =
                false;     // 删除当前节点
            return false;  // 不需要重试
        }
    }

    /*
     * 19 20 21 22 23
     * 17 5  6  7  18
     * 15 3  X  4  16
     * 13 0  1  2  14
     * 8  9  10 11 12
     */
    bool retry = false;
    if (voroCount < 3) {
        return retry;  // 不需要重试
    }
    if (f[0] && f[1] && f[3]) {
        retry = true;
        // curr
        if (!(voronoiAt(pt_curr + ne_offset_[4]) ||
              voronoiAt(pt_curr + ne_offset_[6]))) {
            voronoi_new[(pt_curr.y) * sizeX_ + (pt_curr.x)] =
                false;  // 删除当前节点
            retry = false;
            return retry;
        }
    }
    if (f[1] && f[2] && f[4]) {
        if (!(voronoiAt(pt_curr + ne_offset_[3]) ||
              voronoiAt(pt_curr + ne_offset_[6]))) {
            voronoi_new[(pt_curr.y) * sizeX_ + (pt_curr.x)] =
                false;  // 删除当前节点
            retry = false;
            return retry;
        }
    }
    if (f[3] && f[5] && f[6]) {
        retry = true;
        // curr
        if (!(voronoiAt(pt_curr + ne_offset_[4]) ||
              voronoiAt(pt_curr + ne_offset_[1]))) {
            voronoi_new[(pt_curr.y) * sizeX_ + (pt_curr.x)] =
                false;  // 删除当前节点
            retry = false;
            return retry;
        }
    }
    if (f[4] && f[6] && f[7]) {
        retry = true;
        // curr
        if (!(voronoiAt(pt_curr + ne_offset_[3]) ||
              voronoiAt(pt_curr + ne_offset_[1]))) {
            voronoi_new[(pt_curr.y) * sizeX_ + (pt_curr.x)] =
                false;  // 删除当前节点
            retry = false;
            return retry;
        }
    }
    return retry;
}

bool GVG::voronoiAt(int x, int y) const {
    return voronoi_new[y * sizeX_ + x];
}

bool GVG::voronoiAt(const IntPoint& pt) const {
    return voronoi_new[pt.y * sizeX_ + pt.x];
}

#ifdef USE_OPENCV
void GVG::drawVoronoiByGraphs(cv::Mat& img) const {
    img.setTo(cv::Scalar(255, 255, 255));  // 清空图像
    for (size_t i = 0; i < graphs_.size(); ++i) {
        const auto& graph = graphs_[i];
        for (const auto& node : graph) {
            auto node_ptr = node.second;
            IntPoint node_pos = node_ptr->pos;
            if (node_ptr->type == GraphNode::Strong) {
                img.at<cv::Vec3b>(sizeY_ - node_pos.y - 1, node_pos.x) =
                    cv::Vec3b(0, 0, 255);  // 红色
            } else if (node_ptr->type == GraphNode::Weak) {
                img.at<cv::Vec3b>(sizeY_ - node_pos.y - 1, node_pos.x) =
                    cv::Vec3b(255, 0, 0);  // 蓝色
            }
            const std::vector<Path>& paths = node_ptr->neighbor_paths;
            for (const auto& path : paths) {
                if (path.path.empty())
                    continue;
                for (size_t j = 0; j < path.path.size(); ++j) {
                    int px = path.path[j].x;
                    int py = path.path[j].y;
                    img.at<cv::Vec3b>(sizeY_ - py - 1, px)[0] = 0;  // 蓝色通道
                    img.at<cv::Vec3b>(sizeY_ - py - 1, px)[1] =
                        255;                                        // 绿色通道
                    img.at<cv::Vec3b>(sizeY_ - py - 1, px)[2] = 0;  // 红色通道
                }
            }
        }
    }
}

void GVG::drawVoronoiByGrid(cv::Mat& img) const {
    img.setTo(cv::Scalar(255, 255, 255));  // 清空图像
    for (int y = 0; y < sizeY_; ++y) {
        for (int x = 0; x < sizeX_; ++x) {
            if (voronoiAt(x, y)) {
                img.at<cv::Vec3b>(sizeY_ - y - 1, x) =
                    cv::Vec3b(0, 0, 255);  // 红色
            }
        }
    }
    for (int y = 0; y < sizeY_; ++y) {
        for (int x = 0; x < sizeX_; ++x) {
            if (grid_types_[y * sizeX_ + x] == GRID_TYPE::Strong) {
                img.at<cv::Vec3b>(sizeY_ - y - 1, x) =
                    cv::Vec3b(255, 0, 255);  // 紫色
            } else if (grid_types_[y * sizeX_ + x] == GRID_TYPE::Weak) {
                img.at<cv::Vec3b>(sizeY_ - y - 1, x) =
                    cv::Vec3b(0, 255, 0);  // 绿色
            }
        }
    }
}
#endif  // USE_OPENCV

}  // namespace egvg
}  // namespace planner
}  // namespace planning
}  // namespace autonomy