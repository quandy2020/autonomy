/*
 * Copyright 2026 The Openbot Authors
 */
#include "argus/system/voxel/global_mapping/pgo_edges.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

#include <Eigen/Geometry>

#include "glog/logging.h"

namespace argus {
namespace vxslam {

void PgoEdge::Push(int id1, int id2, const Eigen::Matrix3d& rot,
                   const Eigen::Vector3d& tra,
                   const Eigen::Matrix<double, 6, 1>& v6) {
    ids1.push_back(id1);
    ids2.push_back(id2);
    rots.push_back(rot);
    tras.push_back(tra);
    covs.push_back(v6);
}

bool PgoEdge::IsAdapt(const std::vector<int>& maps,
                      std::vector<int>* step) const {
    if (step == nullptr || step->size() < 2) {
        return false;
    }
    bool f1 = false;
    bool f2 = false;
    for (int i = 0; i < static_cast<int>(maps.size()); ++i) {
        if (m1 == maps[static_cast<size_t>(i)]) {
            f1 = true;
            (*step)[0] = i;
        }
        if (m2 == maps[static_cast<size_t>(i)]) {
            f2 = true;
            (*step)[1] = i;
        }
    }
    return f1 && f2;
}

void PgoEdges::Push(int m1, int m2, int id1, int id2,
                    const Eigen::Matrix3d& rot, const Eigen::Vector3d& tra,
                    const Eigen::Matrix<double, 6, 1>& v6) {
    for (PgoEdge& e : edges) {
        if (e.m1 == m1 && e.m2 == m2) {
            e.Push(id1, id2, rot, tra, v6);
            return;
        }
    }
    PgoEdge e;
    e.m1 = m1;
    e.m2 = m2;
    e.Push(id1, id2, rot, tra, v6);
    edges.push_back(e);
    while (static_cast<int>(mates.size()) <= std::max(m1, m2)) {
        mates.emplace_back();
    }
    mates[static_cast<size_t>(m1)].push_back(m2);
    mates[static_cast<size_t>(m2)].push_back(m1);
}

void PgoEdges::Connect(int root, std::vector<int>* ids) const {
    if (ids == nullptr) {
        return;
    }
    ids->clear();
    ids->push_back(root);
    std::vector<int> stack = {root};
    while (!stack.empty()) {
        const int ord = stack.back();
        stack.pop_back();
        if (ord < 0 || ord >= static_cast<int>(mates.size())) {
            continue;
        }
        for (int id : mates[static_cast<size_t>(ord)]) {
            if (std::find(ids->begin(), ids->end(), id) != ids->end()) {
                continue;
            }
            ids->push_back(id);
            stack.push_back(id);
        }
    }
    std::sort(ids->begin(), ids->end());
}

bool PgoEdgesIo(PgoEdges* edges, const std::vector<std::string>& fnames,
                int io, const std::string& save_path,
                const std::string& bagname) {
    if (edges == nullptr) {
        return false;
    }
    static std::vector<std::string> seq_absent;
    Eigen::Matrix<double, 6, 1> v6_init;
    v6_init << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    if (io == 0) {
        std::ifstream infile(save_path + "edge.txt");
        if (!infile) {
            return false;
        }
        std::string line;
        while (std::getline(infile, line)) {
            std::stringstream ss(line);
            std::vector<std::string> sts;
            std::string str;
            while (ss >> str) {
                sts.push_back(str);
            }
            if (sts.size() < 11) {
                continue;
            }
            int mp[2] = {-1, -1};
            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < static_cast<int>(fnames.size()); ++j) {
                    if (sts[static_cast<size_t>(i)] ==
                        fnames[static_cast<size_t>(j)]) {
                        mp[i] = j;
                        break;
                    }
                }
            }
            if (mp[0] != -1 && mp[1] != -1) {
                const int id1 = std::stoi(sts[2]);
                const int id2 = std::stoi(sts[3]);
                Eigen::Vector3d v3(std::stod(sts[4]), std::stod(sts[5]),
                                   std::stod(sts[6]));
                Eigen::Quaterniond qq(std::stod(sts[10]), std::stod(sts[7]),
                                      std::stod(sts[8]), std::stod(sts[9]));
                Eigen::Matrix3d rot = qq.normalized().toRotationMatrix();
                if (mp[0] <= mp[1]) {
                    edges->Push(mp[0], mp[1], id1, id2, rot, v3, v6_init);
                } else {
                    v3 = -rot.transpose() * v3;
                    rot = rot.transpose();
                    edges->Push(mp[1], mp[0], id2, id1, rot, v3, v6_init);
                }
            } else if (sts[0] != bagname && sts[1] != bagname) {
                seq_absent.push_back(line);
            }
        }
        return true;
    }
    std::ofstream outfile(save_path + "edge.txt");
    if (!outfile) {
        return false;
    }
    for (const std::string& s : seq_absent) {
        outfile << s << "\n";
    }
    outfile.setf(std::ios::fixed);
    outfile.precision(7);
    for (const PgoEdge& edge : edges->edges) {
        for (size_t i = 0; i < edge.rots.size(); ++i) {
            outfile << fnames[static_cast<size_t>(edge.m1)] << " "
                    << fnames[static_cast<size_t>(edge.m2)] << " "
                    << edge.ids1[i] << " " << edge.ids2[i] << " "
                    << edge.tras[i][0] << " " << edge.tras[i][1] << " "
                    << edge.tras[i][2] << " ";
            const Eigen::Quaterniond qq(edge.rots[i]);
            outfile << qq.x() << " " << qq.y() << " " << qq.z() << " "
                    << qq.w() << "\n";
        }
    }
    return true;
}

void BuildPoseGraph(backend::PoseGraph* graph,
                    const std::vector<ImuState>& poses,
                    const std::vector<backend::LoopFactor>& loops,
                    double odom_info) {
    if (graph == nullptr || poses.empty()) {
        return;
    }
    graph->Clear();
    for (int i = 0; i < static_cast<int>(poses.size()); ++i) {
        const ImuState& x = poses[static_cast<size_t>(i)];
        graph->AddNode(
            i, transform::Rigid3d(x.p, Eigen::Quaterniond(x.R)));
        if (i > 0) {
            const ImuState& a = poses[static_cast<size_t>(i - 1)];
            const ImuState& b = poses[static_cast<size_t>(i)];
            const Eigen::Matrix3d R = a.R.transpose() * b.R;
            const Eigen::Vector3d t = a.R.transpose() * (b.p - a.p);
            graph->AddOdometryEdge(
                i - 1, i, transform::Rigid3d(t, Eigen::Quaterniond(R)),
                odom_info);
        }
    }
    for (const backend::LoopFactor& f : loops) {
        graph->AddLoopFactor(f);
    }
}

}  // namespace vxslam
}  // namespace argus
