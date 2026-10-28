/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/voxel/serialization/voxel_map_serializer.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>

#include "glog/logging.h"

namespace argus {
namespace vxslam {

std::vector<PreviousMapSpec> ParsePreviousMapList(const std::string& text) {
  std::vector<PreviousMapSpec> out;
  std::string cleaned;
  for (char c : text) {
    if (c == '#' || c == '\n') {
      continue;
    }
    cleaned.push_back(c);
  }
  std::stringstream ss(cleaned);
  std::string token;
  while (std::getline(ss, token, ',')) {
    const auto colon = token.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    PreviousMapSpec s;
    s.name = token.substr(0, colon);
    while (!s.name.empty() && s.name.front() == ' ') {
      s.name.erase(s.name.begin());
    }
    while (!s.name.empty() && s.name.back() == ' ') {
      s.name.pop_back();
    }
    try {
      s.jud = std::stod(token.substr(colon + 1));
    } catch (...) {
      s.jud = 0.5;
    }
    if (!s.name.empty()) {
      out.push_back(s);
    }
  }
  return out;
}

bool SaveTrajectoryTum(const std::string& path,
                       const std::vector<ImuState>& poses) {
  std::ofstream ofs(path);
  if (!ofs) {
    LOG(ERROR) << "SaveTrajectoryTum failed: " << path;
    return false;
  }
  ofs.setf(std::ios::fixed);
  ofs.precision(6);
  for (const auto& x : poses) {
    const Eigen::Quaterniond q(x.R);
    ofs << x.t << " " << x.p.x() << " " << x.p.y() << " " << x.p.z() << " "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
  }
  return static_cast<bool>(ofs);
}

bool SaveMapXyz(const std::string& path, const sensor::PointCloudXYZI& cloud) {
  std::ofstream ofs(path);
  if (!ofs) {
    return false;
  }
  ofs.setf(std::ios::fixed);
  ofs.precision(4);
  for (const auto& p : cloud.points) {
    ofs << p.x << " " << p.y << " " << p.z << "\n";
  }
  return static_cast<bool>(ofs);
}

bool LoadTrajectoryTum(const std::string& path, std::vector<ImuState>* poses) {
  if (poses == nullptr) {
    return false;
  }
  std::ifstream ifs(path);
  if (!ifs) {
    return false;
  }
  poses->clear();
  double t, x, y, z, qx, qy, qz, qw;
  while (ifs >> t >> x >> y >> z >> qx >> qy >> qz >> qw) {
    ImuState s;
    s.t = t;
    s.p = Eigen::Vector3d(x, y, z);
    s.R = Eigen::Quaterniond(qw, qx, qy, qz).normalized().toRotationMatrix();
    poses->push_back(s);
  }
  return !poses->empty();
}

bool SaveSessionPoses(const std::string& path,
                      const std::vector<SessionPoseEntry>& poses) {
  if (poses.size() < 1) {
    return false;
  }
  std::ofstream ofs(path);
  if (!ofs) {
    return false;
  }
  ofs.setf(std::ios::fixed);
  for (const SessionPoseEntry& e : poses) {
    const ImuState& xx = e.x;
    const Eigen::Quaterniond qq(xx.R);
    ofs.precision(6);
    ofs << xx.t << " ";
    ofs.precision(7);
    ofs << xx.p[0] << " " << xx.p[1] << " " << xx.p[2] << " ";
    ofs << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w();
    ofs << " " << xx.v[0] << " " << xx.v[1] << " " << xx.v[2];
    ofs << " " << xx.bg[0] << " " << xx.bg[1] << " " << xx.bg[2];
    ofs << " " << xx.ba[0] << " " << xx.ba[1] << " " << xx.ba[2];
    ofs << " " << xx.g[0] << " " << xx.g[1] << " " << xx.g[2];
    for (int j = 0; j < 6; ++j) {
      ofs << " " << e.v6[j];
    }
    ofs << "\n";
  }
  return static_cast<bool>(ofs);
}

bool LoadSessionPoses(const std::string& path,
                      std::vector<SessionPoseEntry>* poses) {
  if (poses == nullptr) {
    return false;
  }
  std::ifstream ifs(path);
  if (!ifs) {
    return false;
  }
  poses->clear();
  std::string line;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    SessionPoseEntry e;
    double qx, qy, qz, qw;
    if (!(ss >> e.x.t >> e.x.p[0] >> e.x.p[1] >> e.x.p[2] >> qx >> qy >> qz >>
          qw >> e.x.v[0] >> e.x.v[1] >> e.x.v[2] >> e.x.bg[0] >> e.x.bg[1] >>
          e.x.bg[2] >> e.x.ba[0] >> e.x.ba[1] >> e.x.ba[2] >> e.x.g[0] >>
          e.x.g[1] >> e.x.g[2])) {
      poses->clear();
      return false;
    }
    for (int j = 0; j < 6; ++j) {
      if (!(ss >> e.v6[j])) {
        poses->clear();
        return false;
      }
    }
    std::string extra;
    const Eigen::Quaterniond q(qw, qx, qy, qz);
    if (ss >> extra || !std::isfinite(e.x.t) || !e.x.p.allFinite() ||
        !e.x.v.allFinite() || !e.x.bg.allFinite() || !e.x.ba.allFinite() ||
        !e.x.g.allFinite() || !e.v6.allFinite() || !q.coeffs().allFinite() ||
        q.norm() < 1e-12) {
      poses->clear();
      return false;
    }
    e.x.R = q.normalized().toRotationMatrix();
    poses->push_back(e);
  }
  return !poses->empty();
}

bool SaveCloudBin(const std::string& path, const PointVarList& pvec) {
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) {
    return false;
  }
  const uint64_t n = pvec.size();
  ofs.write(reinterpret_cast<const char*>(&n), sizeof(n));
  for (const PointVar& pv : pvec) {
    const float xyz[4] = {static_cast<float>(pv.pnt[0]),
                          static_cast<float>(pv.pnt[1]),
                          static_cast<float>(pv.pnt[2]), 0.f};
    ofs.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
  }
  return static_cast<bool>(ofs);
}

bool LoadCloudBin(const std::string& path, PointVarList* pvec) {
  if (pvec == nullptr) {
    return false;
  }
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    return false;
  }
  ifs.seekg(0, std::ios::end);
  const std::streamoff file_size = ifs.tellg();
  if (file_size < static_cast<std::streamoff>(sizeof(uint64_t))) {
    return false;
  }
  ifs.seekg(0, std::ios::beg);
  uint64_t n = 0;
  ifs.read(reinterpret_cast<char*>(&n), sizeof(n));
  constexpr std::streamoff kPointBytes = 4 * sizeof(float);
  const std::streamoff payload_size = file_size - sizeof(n);
  if (!ifs || n > pvec->max_size() || payload_size % kPointBytes != 0 ||
      n != static_cast<uint64_t>(payload_size / kPointBytes)) {
    return false;
  }
  pvec->clear();
  pvec->reserve(static_cast<size_t>(n));
  for (uint64_t i = 0; i < n; ++i) {
    float xyz[4];
    ifs.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (!ifs) {
      pvec->clear();
      return false;
    }
    PointVar pv;
    pv.pnt << xyz[0], xyz[1], xyz[2];
    pvec->push_back(pv);
  }
  return !pvec->empty();
}

bool SaveCloudBin(const std::string& path,
                  const sensor::PointCloudXYZI& cloud) {
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) {
    return false;
  }
  const uint64_t n = cloud.size();
  ofs.write(reinterpret_cast<const char*>(&n), sizeof(n));
  for (const auto& p : cloud.points) {
    const float xyz[4] = {p.x, p.y, p.z, p.intensity};
    ofs.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
  }
  return static_cast<bool>(ofs);
}

bool LoadCloudBin(const std::string& path, sensor::PointCloudXYZI* cloud) {
  if (cloud == nullptr) {
    return false;
  }
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    return false;
  }
  ifs.seekg(0, std::ios::end);
  const std::streamoff file_size = ifs.tellg();
  if (file_size < static_cast<std::streamoff>(sizeof(uint64_t))) {
    return false;
  }
  ifs.seekg(0, std::ios::beg);
  uint64_t n = 0;
  ifs.read(reinterpret_cast<char*>(&n), sizeof(n));
  constexpr std::streamoff kPointBytes = 4 * sizeof(float);
  const std::streamoff payload_size = file_size - sizeof(n);
  if (!ifs || n > cloud->points.max_size() || payload_size % kPointBytes != 0 ||
      n != static_cast<uint64_t>(payload_size / kPointBytes)) {
    return false;
  }
  cloud->clear();
  cloud->reserve(static_cast<size_t>(n));
  for (uint64_t i = 0; i < n; ++i) {
    float xyz[4];
    ifs.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (!ifs) {
      cloud->clear();
      return false;
    }
    sensor::PointType p;
    p.x = xyz[0];
    p.y = xyz[1];
    p.z = xyz[2];
    p.intensity = xyz[3];
    cloud->push_back(p);
  }
  return !cloud->empty();
}

}  // namespace vxslam
}  // namespace argus
