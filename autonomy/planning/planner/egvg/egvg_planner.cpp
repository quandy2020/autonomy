#include "autonomy/planning/planner/egvg/egvg_planner.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace egvg {

Planner::Planner() = default;

Planner::~Planner() {
    for (int i = 0; i < allocate_num_; ++i) {
        path_node_pool_[i].reset();
    }
}

void Planner::init(std::shared_ptr<GVG> gvg) {
    gvg_ = std::move(gvg);
    path_node_pool_.resize(allocate_num_);
    for (int i = 0; i < allocate_num_; ++i) {
        path_node_pool_[i] = std::make_shared<GraphNode>();
    }

    use_node_num_ = 0;
    iter_num_ = 0;
}

void Planner::reset() {
    expanded_nodes_.clear();
    path_nodes_.clear();

    std::priority_queue<GraphNode::Ptr, std::vector<GraphNode::Ptr>, NodeComparator0> empty_queue;
    open_set_.swap(empty_queue);

    for (int i = 0; i < use_node_num_; ++i) {
        GraphNode::Ptr node = path_node_pool_[i];
        node->parent.reset();
        node->node_state = GraphNode::NOT_EXPAND_GVG;
        node->g_score = 0.0;
        node->f_score = 0.0;
    }
    use_node_num_ = 0;
    iter_num_ = 0;
}

bool Planner::serachPath(GraphNode::Ptr start_node, GraphNode::Ptr goal_node) {
    // 1. 初始化
    reset();

    // 2. 创建起点和终点节点
    GraphNode::Ptr current_node = start_node;
    current_node->parent.reset();
    current_node->g_score = 0.0;
    current_node->f_score = lambda_heu_ * getDiagHeu(current_node->pos, goal_node->pos);
    current_node->node_state = GraphNode::IN_OPEN_SET_GVG;

    open_set_.push(current_node);
    expanded_nodes_.insert(current_node->pos, current_node);
    GraphNode::Ptr terminate_node = nullptr;
    GraphNode::Ptr neighbor_node = nullptr;
    // 3. A*搜索
    while (!open_set_.empty()) {
        current_node = open_set_.top();
        if (current_node->pos == goal_node->pos) {
            terminate_node = current_node;
            retrievePath(terminate_node);
            return true;
        }
        open_set_.pop();
        current_node->node_state = GraphNode::IN_CLOSE_SET_GVG;
        iter_num_++;

        // 扩展邻居
        for (int i = 0; i < static_cast<int>(current_node->neighbors.size()); ++i) {
            neighbor_node = current_node->neighbors[i].lock();
            if (!neighbor_node) {
                continue;
            }

            IntPoint neighborPoint = neighbor_node->pos;
            if (neighbor_node->type != GraphNode::Strong || neighbor_node->RemovedByPVS) {
                continue;
            }
            auto expanded_node = expanded_nodes_.find(neighborPoint);
            if (expanded_node && expanded_node->node_state == GraphNode::IN_CLOSE_SET_GVG) {
                continue;
            }

            double tentative_g_score =
                current_node->g_score + current_node->neighbor_paths[i].path_length + 1.0;  // 假设网格的距离为 1
            double tentative_f_score = tentative_g_score + lambda_heu_ * getEuclHeu(neighborPoint, goal_node->pos);

            if (!expanded_node) {
                neighbor_node->g_score = tentative_g_score;
                neighbor_node->f_score = tentative_f_score;
                neighbor_node->parent = current_node;
                neighbor_node->node_state = GraphNode::IN_OPEN_SET_GVG;
                open_set_.push(neighbor_node);
                expanded_nodes_.insert(neighborPoint, neighbor_node);
                if (use_node_num_ == allocate_num_) {
                    std::cout << "A star on GVG run out of memory." << std::endl;
                    return false;
                }
            } else if (neighbor_node->node_state == GraphNode::IN_OPEN_SET_GVG) {
                if (tentative_g_score < neighbor_node->g_score) {
                    neighbor_node->f_score = tentative_f_score;
                    neighbor_node->g_score = tentative_g_score;
                    neighbor_node->parent = current_node;
                }
            } else {
                std::cout << "A star on GVG error type in searching: " << static_cast<int>(neighbor_node->node_state)
                          << std::endl;
            }
        }
    }
    return false;
}

void Planner::retrievePath(GraphNode::Ptr end_node) {
    GraphNode::Ptr cur_node = end_node;
    path_nodes_.emplace_back(cur_node);

    while (auto parent_ptr = cur_node->parent.lock()) {
        cur_node = parent_ptr;
        path_nodes_.emplace_back(cur_node);
    }

    std::reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<IntPoint> Planner::getPath() {
    std::vector<IntPoint> path;
    for (const auto& node : path_nodes_) {
        path.emplace_back(node->pos);
    }
    return path;
}

std::vector<IntPoint> Planner::getFullPath() {
    std::vector<IntPoint> path;
    if (path_nodes_.empty())
        return path;
    path.emplace_back(path_nodes_.front()->pos);
    for (int i = 1; i < static_cast<int>(path_nodes_.size()); ++i) {
        GraphNode::Ptr prev = path_nodes_[i - 1];
        GraphNode::Ptr curr = path_nodes_[i];
        int neighbor_idx = -1;
        for (size_t j = 0; j < prev->neighbors.size(); ++j) {
            if (prev->neighbors[j].lock() == curr) {
                neighbor_idx = static_cast<int>(j);
                break;
            }
        }
        if (neighbor_idx == -1) {
            // 没找到，说明图结构有问题
            continue;
        }
        const auto& sub_path = prev->neighbor_paths[neighbor_idx].path;
        path.insert(path.end(), sub_path.begin(), sub_path.end());
        path.emplace_back(path_nodes_[i]->pos);
    }
    return path;
}

std::vector<IntPoint> Planner::getFullPathNodes() {
    std::vector<IntPoint> path_nodes;
    if (path_nodes_.empty()) {
        return path_nodes;
    }
    path_nodes.emplace_back(path_nodes_.front()->pos);
    for (int i = 1; i < static_cast<int>(path_nodes_.size()); ++i) {
        GraphNode::Ptr prev = path_nodes_[i - 1];
        GraphNode::Ptr curr = path_nodes_[i];
        int neighbor_idx = -1;
        for (size_t j = 0; j < prev->neighbors.size(); ++j) {
            if (prev->neighbors[j].lock() == curr) {
                neighbor_idx = static_cast<int>(j);
                break;
            }
        }
        if (neighbor_idx == -1) {
            // 没找到，说明图结构有问题
            continue;
        }
        path_nodes.emplace_back(path_nodes_[i]->pos);
    }
    return path_nodes;
}

std::vector<IntPoint> Planner::getVisitedNodes() {
    std::vector<IntPoint> visited;
    for (int i = 0; i < use_node_num_ - 1; ++i) {
        visited.emplace_back(path_node_pool_[i]->pos);
    }
    return visited;
}

std::vector<IntPoint> Planner::AstarOnVoronoi(const IntPoint& pt1, const IntPoint& pt2, int sizeX, int sizeY,
                                              int graph_id) {
    (void)sizeX;
    (void)sizeY;
    // 1. 检查起点和终点是否在骨架上
    if (!gvg_->isVoronoi(pt1) || !gvg_->isVoronoi(pt2)) {
        std::cout << "Start or goal not on skeleton!" << std::endl;
        return {};
    }
    if (graph_id < 0) {
        std::cout << "It is not a valid graph id!" << std::endl;
        return {};
    }
    const auto graph_current = gvg_->getGraphs()[graph_id];
    // 2. 清空A*相关成员
    reset();

    // 3. 初始化起点
    GraphNode::Ptr start_node = path_node_pool_[use_node_num_++];
    start_node->pos = pt1;
    start_node->type = GraphNode::Strong;
    start_node->g_score = 0.0;
    start_node->f_score = getDiagHeu(pt1, pt2);
    start_node->parent.reset();
    start_node->node_state = GraphNode::IN_OPEN_SET_GVG;
    open_set_.push(start_node);
    expanded_nodes_.insert(pt1, start_node);

    // 四邻域
    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    while (!open_set_.empty()) {
        GraphNode::Ptr cur = open_set_.top();
        open_set_.pop();

        iter_num_++;

        auto it_curr = graph_current.find(cur->pos);
        if (cur->pos == pt2 || it_curr != graph_current.end()) {
            // 回溯路径
            path_nodes_.clear();
            for (auto node = cur; node; node = node->parent.lock())
                path_nodes_.push_back(node);
            std::reverse(path_nodes_.begin(), path_nodes_.end());
            std::vector<IntPoint> path;
            for (auto& n : path_nodes_)
                path.push_back(n->pos);
            return path;
        }

        cur->node_state = GraphNode::IN_CLOSE_SET_GVG;

        for (int i = 0; i < 4; ++i) {
            IntPoint nb(cur->pos.x + dx[i], cur->pos.y + dy[i]);
            if (!gvg_->isVoronoi(nb))
                continue;

            auto nb_node = expanded_nodes_.find(nb);
            double g_new = cur->g_score + 1.0;
            double f_new = g_new + getDiagHeu(nb, pt2);

            if (!nb_node) {
                GraphNode::Ptr new_node = path_node_pool_[use_node_num_++];
                new_node->pos = nb;
                new_node->type = GraphNode::Strong;
                new_node->g_score = g_new;
                new_node->f_score = f_new;
                new_node->parent = cur;
                new_node->node_state = GraphNode::IN_OPEN_SET_GVG;
                open_set_.push(new_node);
                expanded_nodes_.insert(nb, new_node);
                if (use_node_num_ == allocate_num_) {
                    std::cout << "A star on GVG run out of memory." << std::endl;
                    return {};
                }
            } else if (nb_node->node_state == GraphNode::IN_OPEN_SET_GVG && g_new < nb_node->g_score) {
                nb_node->g_score = g_new;
                nb_node->f_score = f_new;
                nb_node->parent = cur;
            }
        }
    }
    std::cout << "No path found on skeleton!" << std::endl;
    std::cout << "use node num: " << use_node_num_ << std::endl;
    std::cout << "iter num: " << iter_num_ << std::endl;
    return {};
}

std::vector<std::vector<IntPoint>> Planner::expand_voronoi_grid(const IntPoint& voronoi_grid,
                                                                const std::vector<IntPoint> strong_nodes,
                                                                const int& sizeX, const int& sizeY) {
    // 1. 检查起点是否在骨架上 isVoronoi, 并且不在 graph 上；
    if (!gvg_->isVoronoi(voronoi_grid)) {
        std::cout << "Start point is not on skeleton!" << std::endl;
        return {};
    }
    if (gvg_->findNodeInGraphs(voronoi_grid)) {
        std::cout << "Start point is already in graph!" << std::endl;
        return {};
    }
    // 四邻域
    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};
    std::vector<IntPoint> expand_dirs;
    for (int i = 0; i < 4; ++i) {
        IntPoint nb(voronoi_grid.x + dx[i], voronoi_grid.y + dy[i]);
        if (!gvg_->isVoronoi(nb))
            continue;
        expand_dirs.push_back(IntPoint(dx[i], dy[i]));
    }
    if (expand_dirs.size() != 2) {
        std::cout << "Start point is not a valid edge point!" << std::endl;
        return {};
    }
    std::vector<std::vector<IntPoint>> result_paths;
    IntPoint curr_dir;
    // 沿着两个方向开始扩展
    for (int i = 0; i < static_cast<int>(expand_dirs.size()); ++i) {
        std::vector<IntPoint> path;
        curr_dir = expand_dirs[i];
        std::queue<std::pair<IntPoint, std::vector<IntPoint>>> queue;
        queue.push({voronoi_grid + curr_dir, {voronoi_grid + curr_dir}});
        while (!queue.empty()) {
            auto current_pair = queue.front();
            IntPoint current_point = current_pair.first;
            path = current_pair.second;
            queue.pop();

            if (std::find(strong_nodes.begin(), strong_nodes.end(), current_point) != strong_nodes.end()) {
                result_paths.push_back(path);
                break;  // 找到强节点，结束扩展
            }
            if (path.size() >= 2)
                curr_dir = path.back() - path[path.size() - 2];  // 获取当前方向
            for (int j = 0; j < 4; ++j) {
                IntPoint expand_dir(dx[j], dy[j]);
                if (expand_dir.x == -curr_dir.x && expand_dir.y == -curr_dir.y)
                    continue;  // 跳过当前方向

                IntPoint candidate = current_point + expand_dir;
                if (candidate.x < 0 || candidate.x >= sizeX || candidate.y < 0 || candidate.y >= sizeY)
                    continue;
                if (!gvg_->isVoronoi(candidate))
                    continue;

                std::vector<IntPoint> new_path = path;
                new_path.push_back(candidate);
                queue.push({candidate, new_path});
            }
        }
    }
    // 检查两个路径是否有相同的强节点
    if (result_paths.size() == 2 && result_paths[0].back() == result_paths[1].back()) {
        // 两条路径的终点是同一个强节点，表示成环了
        std::cout << "Found a loop!" << std::endl;
        return {result_paths[0].size() < result_paths[1].size() ? result_paths[0]
                                                                : result_paths[1]};  // 返回只包含一条路径的二维vector
    }
    return result_paths;  // 返回两条路径
}

void Planner::DFSSearch(std::vector<GraphNode::Ptr>& vis, GraphNode::Ptr goal, int max_path_num) {
    GraphNode::Ptr cur = vis.back();

    for (int i = 0; i < static_cast<int>(cur->neighbors.size()); ++i) {
        // 到达终点
        if (cur->neighbors[i].lock() == goal) {
            raw_topo_paths_.push_back(vis);          // 直接存 GraphNode::Ptr 序列
            raw_topo_paths_.back().push_back(goal);  // 添加终点节点
            if (static_cast<int>(raw_topo_paths_.size()) >= max_path_num)
                return;
            break;
        }
    }
    IntPoint dir_to_goal = goal->pos - cur->pos;
    double dx1 = static_cast<double>(dir_to_goal.x);
    double dy1 = static_cast<double>(dir_to_goal.y);

    // 2. 收集可扩展邻居及其距离
    std::vector<std::pair<double, int>> neighbor_scores;
    for (int i = 0; i < static_cast<int>(cur->neighbors.size()); ++i) {
        GraphNode::Ptr neighbor = cur->neighbors[i].lock();
        if (!neighbor)
            continue;

        IntPoint dir_to_neighbor = neighbor->pos - cur->pos;
        IntPoint dir_neighbor_to_goal = goal->pos - neighbor->pos;
        (void)dir_to_neighbor;
        (void)dir_neighbor_to_goal;

        double dx3 = static_cast<double>(dir_neighbor_to_goal.x);
        double dy3 = static_cast<double>(dir_neighbor_to_goal.y);
        double dis = std::hypot(dx3, dy3);
        (void)dx1;
        (void)dy1;

        neighbor_scores.emplace_back(dis, i);
    }
    std::sort(neighbor_scores.begin(), neighbor_scores.end(),
              [](const std::pair<double, int>& a, const std::pair<double, int>& b) { return a.first < b.first; });

    for (const auto& item : neighbor_scores) {
        int idx = item.second;
        GraphNode::Ptr neighbor = cur->neighbors[idx].lock();
        if (!neighbor)
            continue;
        if (neighbor->type != GraphNode::Strong)
            continue;  // 只考虑强节点
        if (neighbor == goal)
            continue;
        // 跳过已访问节点，防止环路
        bool revisit = false;
        for (const auto& node : vis) {
            if (neighbor->pos == node->pos) {
                revisit = true;
                break;
            }
        }
        if (revisit)
            continue;

        vis.push_back(neighbor);
        DFSSearch(vis, goal, max_path_num);
        if (static_cast<int>(raw_topo_paths_.size()) >= max_path_num)
            return;
        vis.pop_back();
    }
}

std::vector<std::vector<IntPoint>> Planner::searchTopoPaths(GraphNode::Ptr start_node, GraphNode::Ptr goal_node,
                                                            int max_path_num) {
    raw_topo_paths_.clear();
    std::vector<GraphNode::Ptr> vis;
    vis.push_back(start_node);
    DFSSearch(vis, goal_node, max_path_num);

    std::vector<std::vector<IntPoint>> topo_paths;
    for (const auto& path_nodes : raw_topo_paths_) {
        std::vector<IntPoint> path;
        if (path_nodes.empty()) {
            topo_paths.push_back(path);
            continue;
        }
        // 起点
        path.emplace_back(path_nodes.front()->pos);
        for (int i = 1; i < static_cast<int>(path_nodes.size()); ++i) {
            GraphNode::Ptr prev = path_nodes[i - 1];
            GraphNode::Ptr curr = path_nodes[i];
            int neighbor_idx = -1;
            for (size_t j = 0; j < prev->neighbors.size(); ++j) {
                if (prev->neighbors[j].lock() == curr) {
                    neighbor_idx = static_cast<int>(j);
                    break;
                }
            }
            if (neighbor_idx == -1) {
                // 没找到，说明图结构有问题
                continue;
            }
            // 插入 prev 到 curr 的中间点（不含起点和终点）
            const auto& sub_path = prev->neighbor_paths[neighbor_idx].path;
            path.insert(path.end(), sub_path.begin(), sub_path.end());
            // 插入当前节点
            path.emplace_back(curr->pos);
        }
        topo_paths.push_back(path);
    }
    std::cout << "Found " << raw_topo_paths_.size() << " topological paths." << std::endl;
    for (int i = 0; i < static_cast<int>(raw_topo_paths_.size()); ++i) {
        std::cout << "Path " << i << ": " << raw_topo_paths_[i].size() << " nodes, " << topo_paths[i].size()
                  << " points, " << std::endl;
    }
    return topo_paths;
}

double Planner::getDiagHeu(const IntPoint& p1, const IntPoint& p2) {
    int dx = std::abs(p1.x - p2.x);
    int dy = std::abs(p1.y - p2.y);
    int diag = std::min(dx, dy);
    double h = std::sqrt(2.0) * diag + 1.0 * (std::max(dx, dy) - diag);
    return tie_breaker_ * h;
}

double Planner::getManhHeu(const IntPoint& p1, const IntPoint& p2) {
    int dx = std::abs(p1.x - p2.x);
    int dy = std::abs(p1.y - p2.y);
    return tie_breaker_ * (dx + dy);
}

double Planner::getEuclHeu(const IntPoint& p1, const IntPoint& p2) {
    return tie_breaker_ * std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

}  // namespace egvg
}  // namespace planner
}  // namespace planning
}  // namespace autonomy
