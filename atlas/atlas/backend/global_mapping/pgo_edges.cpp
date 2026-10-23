/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/pgo_edges.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

#include "glog/logging.h"

namespace atlas {
namespace vxslam {
namespace {

std::filesystem::path TemporaryPathFor(const std::filesystem::path& path) {
  static std::atomic<std::uint64_t> sequence{0};
  const auto stamp =
      std::chrono::steady_clock::now().time_since_epoch().count();
  return path.string() + ".tmp." + std::to_string(stamp) + "." +
         std::to_string(sequence.fetch_add(1, std::memory_order_relaxed));
}

}  // namespace

bool ResolvePoseGraphNodeIndex(int session_index, int local_node_index,
                               const std::vector<int>& session_offsets,
                               int* node_index) {
  if (node_index == nullptr || session_index < 0 || local_node_index < 0 ||
      session_offsets.size() < 2 ||
      static_cast<std::size_t>(session_index) >= session_offsets.size() - 1) {
    return false;
  }
  const std::size_t offset_index = static_cast<std::size_t>(session_index);
  const int first_node_index = session_offsets[offset_index];
  const int past_last_node_index = session_offsets[offset_index + 1];
  if (first_node_index < 0 || past_last_node_index < first_node_index ||
      local_node_index >= past_last_node_index - first_node_index) {
    return false;
  }
  *node_index = first_node_index + local_node_index;
  return true;
}

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
  if (m1 < 0 || m2 < 0 || id1 < 0 || id2 < 0 || !rot.allFinite() ||
      !tra.allFinite() || !v6.allFinite() || v6.minCoeff() < 0.0) {
    return;
  }
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

bool PgoEdgesIo(PgoEdges* edges, const std::vector<std::string>& fnames, int io,
                const std::string& save_path, const std::string& bagname) {
  if (edges == nullptr || (io != 0 && io != 1)) {
    return false;
  }
  const std::filesystem::path edge_path =
      std::filesystem::path(save_path) / "edge.txt";
  if (io == 1 && !bagname.empty() &&
      std::find(fnames.begin(), fnames.end(), bagname) == fnames.end()) {
    return false;
  }
  Eigen::Matrix<double, 6, 1> v6_init;
  v6_init << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
  if (io == 0) {
    PgoEdges parsed_edges;
    std::ifstream infile(edge_path);
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
      if (sts.empty()) {
        continue;
      }
      if (sts.size() != 11 && sts.size() != 17) {
        return false;
      }
      int mp[2] = {-1, -1};
      for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < static_cast<int>(fnames.size()); ++j) {
          if (sts[static_cast<size_t>(i)] == fnames[static_cast<size_t>(j)]) {
            mp[i] = j;
            break;
          }
        }
      }
      if (mp[0] != -1 && mp[1] != -1) {
        int id1 = -1;
        int id2 = -1;
        Eigen::Vector3d v3;
        Eigen::Quaterniond qq;
        Eigen::Matrix<double, 6, 1> covariance = v6_init;
        try {
          size_t used = 0;
          id1 = std::stoi(sts[2], &used);
          if (used != sts[2].size()) {
            return false;
          }
          id2 = std::stoi(sts[3], &used);
          if (used != sts[3].size()) {
            return false;
          }
          const auto parse_double = [&sts](size_t index, double* value) {
            size_t parsed = 0;
            *value = std::stod(sts[index], &parsed);
            return parsed == sts[index].size() && std::isfinite(*value);
          };
          if (!parse_double(4, &v3.x()) || !parse_double(5, &v3.y()) ||
              !parse_double(6, &v3.z())) {
            return false;
          }
          double qx = 0.0;
          double qy = 0.0;
          double qz = 0.0;
          double qw = 0.0;
          if (!parse_double(7, &qx) || !parse_double(8, &qy) ||
              !parse_double(9, &qz) || !parse_double(10, &qw)) {
            return false;
          }
          qq = Eigen::Quaterniond(qw, qx, qy, qz);
          if (sts.size() == 17) {
            for (size_t i = 0; i < 6; ++i) {
              if (!parse_double(11 + i, &covariance[static_cast<int>(i)])) {
                return false;
              }
            }
          }
        } catch (const std::exception&) {
          return false;
        }
        if (id1 < 0 || id2 < 0 || qq.norm() < 1e-12 ||
            covariance.minCoeff() < 0.0) {
          return false;
        }
        const Eigen::Matrix3d rot = qq.normalized().toRotationMatrix();
        if (mp[0] <= mp[1]) {
          parsed_edges.Push(mp[0], mp[1], id1, id2, rot, v3, covariance);
        } else {
          v3 = -rot.transpose() * v3;
          const Eigen::Matrix3d reverse_rotation = rot.transpose();
          parsed_edges.Push(mp[1], mp[0], id2, id1, reverse_rotation, v3,
                            covariance);
        }
      }
    }
    *edges = std::move(parsed_edges);
    return true;
  }

  // Preserve edges whose endpoint sessions are not part of this write. The
  // old implementation kept these lines in a process-global static vector,
  // which leaked state across Odometry instances.
  std::vector<std::string> preserved_edges;
  {
    std::ifstream infile(edge_path);
    std::string line;
    while (infile && std::getline(infile, line)) {
      std::stringstream ss(line);
      std::vector<std::string> fields;
      std::string field;
      while (ss >> field) {
        fields.push_back(field);
      }
      if (fields.empty()) {
        continue;
      }
      if (fields.size() != 11 && fields.size() != 17) {
        return false;
      }
      const bool from_selected =
          std::find(fnames.begin(), fnames.end(), fields[0]) != fnames.end();
      const bool to_selected =
          std::find(fnames.begin(), fnames.end(), fields[1]) != fnames.end();
      if (!from_selected || !to_selected) {
        preserved_edges.push_back(line);
      }
    }
    if (infile.bad()) {
      return false;
    }
  }
  for (const PgoEdge& edge : edges->edges) {
    if (edge.m1 < 0 || edge.m2 < 0 ||
        edge.m1 >= static_cast<int>(fnames.size()) ||
        edge.m2 >= static_cast<int>(fnames.size()) ||
        edge.ids1.size() != edge.rots.size() ||
        edge.ids2.size() != edge.rots.size() ||
        edge.tras.size() != edge.rots.size() ||
        edge.covs.size() != edge.rots.size()) {
      return false;
    }
    for (size_t i = 0; i < edge.rots.size(); ++i) {
      if (edge.ids1[i] < 0 || edge.ids2[i] < 0 || !edge.rots[i].allFinite() ||
          !edge.tras[i].allFinite() || !edge.covs[i].allFinite() ||
          edge.covs[i].minCoeff() < 0.0) {
        return false;
      }
    }
  }

  const std::filesystem::path temporary_path = TemporaryPathFor(edge_path);
  std::ofstream outfile(temporary_path, std::ios::trunc);
  if (!outfile) {
    return false;
  }
  for (const std::string& s : preserved_edges) {
    outfile << s << "\n";
  }
  outfile.setf(std::ios::fixed);
  outfile.precision(7);
  for (const PgoEdge& edge : edges->edges) {
    for (size_t i = 0; i < edge.rots.size(); ++i) {
      outfile << fnames[static_cast<size_t>(edge.m1)] << " "
              << fnames[static_cast<size_t>(edge.m2)] << " " << edge.ids1[i]
              << " " << edge.ids2[i] << " " << edge.tras[i][0] << " "
              << edge.tras[i][1] << " " << edge.tras[i][2] << " ";
      const Eigen::Quaterniond qq(edge.rots[i]);
      outfile << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w()
              << std::scientific << std::setprecision(12);
      for (int axis = 0; axis < 6; ++axis) {
        outfile << " " << edge.covs[i][axis];
      }
      outfile << std::fixed << std::setprecision(7) << "\n";
    }
  }
  outfile.flush();
  const bool write_ok = static_cast<bool>(outfile);
  outfile.close();
  if (!write_ok) {
    std::error_code ignored;
    std::filesystem::remove(temporary_path, ignored);
    return false;
  }
  std::error_code rename_error;
  std::filesystem::rename(temporary_path, edge_path, rename_error);
  if (rename_error) {
    std::error_code ignored;
    std::filesystem::remove(temporary_path, ignored);
    return false;
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
    graph->AddNode(i, transform::Rigid3d(x.p, Eigen::Quaterniond(x.R)));
    if (i > 0) {
      const ImuState& a = poses[static_cast<size_t>(i - 1)];
      const ImuState& b = poses[static_cast<size_t>(i)];
      const Eigen::Matrix3d R = a.R.transpose() * b.R;
      const Eigen::Vector3d t = a.R.transpose() * (b.p - a.p);
      graph->AddOdometryEdge(
          i - 1, i, transform::Rigid3d(t, Eigen::Quaterniond(R)), odom_info);
    }
  }
  for (const backend::LoopFactor& f : loops) {
    graph->AddLoopFactor(f);
  }
}

}  // namespace vxslam
}  // namespace atlas
