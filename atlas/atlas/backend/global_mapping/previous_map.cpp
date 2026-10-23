/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/previous_map.hpp"

#include <deque>
#include <filesystem>
#include <iostream>

#include "frontend/lidar/odometry/downsample.hpp"

namespace atlas {
namespace vxslam {

namespace {

void ClearSession(PreviousMapSession* session) {
  if (session == nullptr) {
    return;
  }
  for (Keyframe* keyframe : session->keyframes) {
    delete keyframe;
  }
  session->keyframes.clear();
}

void ClearSessionKeyframes(std::vector<PreviousMapSession>* sessions) {
  if (sessions == nullptr) {
    return;
  }
  for (PreviousMapSession& session : *sessions) {
    ClearSession(&session);
  }
  sessions->clear();
}

bool LoadScanPoses(const std::string& path, std::vector<ScanPosePtr>* out) {
  if (out == nullptr) {
    return false;
  }
  std::vector<SessionPoseEntry> entries;
  if (!LoadSessionPoses(path, &entries)) {
    return false;
  }
  out->clear();
  out->reserve(entries.size());
  for (const SessionPoseEntry& e : entries) {
    auto sp = std::make_shared<ScanPose>(e.x, nullptr);
    sp->v6 = e.v6;
    out->push_back(std::move(sp));
  }
  return !out->empty();
}

}  // namespace

bool PreviousMapRead(const PreviousMapOptions& opt,
                     std::vector<PreviousMapSession>* sessions,
                     PgoEdges* edges) {
  if (sessions == nullptr) {
    return false;
  }
  ClearSessionKeyframes(sessions);
  PgoEdges loaded_edges;
  const auto specs = ParsePreviousMapList(opt.previous_map);
  if (specs.empty()) {
    if (edges != nullptr) {
      *edges = PgoEdges{};
    }
    return true;
  }
  std::vector<std::string> fnames;
  for (const PreviousMapSpec& s : specs) {
    fnames.push_back(s.name);
  }
  if (edges != nullptr) {
    PgoEdgesIo(&loaded_edges, fnames, /*io=*/0, opt.save_path,
               /*bagname=*/"");
  }

  std::vector<PreviousMapSession> loaded_sessions;

  const int win = std::max(1, opt.win_size);
  const int acsize = std::max(1, opt.acsize);
  const int mgsize = std::max(1, opt.mgsize);

  for (const PreviousMapSpec& spec : specs) {
    const std::filesystem::path session_name(spec.name);
    if (session_name.is_absolute() || session_name.has_parent_path() ||
        spec.name == "." || spec.name == "..") {
      ClearSessionKeyframes(&loaded_sessions);
      return false;
    }
    PreviousMapSession sess;
    sess.name = spec.name;
    sess.jud = spec.jud;
    const std::string root =
        (std::filesystem::path(opt.save_path) / spec.name).string();
    if (!LoadScanPoses(root + "/alidarState.txt", &sess.scan_poses)) {
      std::cerr << "PreviousMapRead: missing poses " << root << "\n";
      ClearSessionKeyframes(&loaded_sessions);
      return false;
    }
    std::cerr << "PreviousMapRead " << root << ": " << sess.scan_poses.size()
              << " scans\n";

    std::deque<sensor::PointCloudXYZI> plbuf;
    std::deque<ImuState> xxbuf;
    for (int i = 0; i < static_cast<int>(sess.scan_poses.size()); ++i) {
      sensor::PointCloudXYZI cloud;
      const std::string bin =
          (std::filesystem::path(root) / (std::to_string(i) + ".bin")).string();
      if (!LoadCloudBin(bin, &cloud)) {
        std::cerr << "PreviousMapRead: missing cloud " << bin << "\n";
        ClearSession(&sess);
        ClearSessionKeyframes(&loaded_sessions);
        return false;
      }
      xxbuf.push_back(sess.scan_poses[static_cast<size_t>(i)]->x);
      plbuf.push_back(std::move(cloud));
      if (static_cast<int>(xxbuf.size()) < win) {
        continue;
      }
      ImuState xc = xxbuf.back();
      auto* smp = new Keyframe(xc);
      smp->id = i;
      smp->mp = static_cast<int>(loaded_sessions.size());
      smp->exist = 1;
      sensor::PointCloudXYZI pl_lc;
      for (int j = 0; j < win; ++j) {
        const Eigen::Vector3d delta_p =
            xc.R.transpose() * (xxbuf[static_cast<size_t>(j)].p - xc.p);
        const Eigen::Matrix3d delta_R =
            xc.R.transpose() * xxbuf[static_cast<size_t>(j)].R;
        for (const auto& pp : plbuf[static_cast<size_t>(j)].points) {
          Eigen::Vector3d v3(pp.x, pp.y, pp.z);
          v3 = delta_R * v3 + delta_p;
          sensor::PointType pt;
          pt.x = static_cast<float>(v3[0]);
          pt.y = static_cast<float>(v3[1]);
          pt.z = static_cast<float>(v3[2]);
          pl_lc.push_back(pt);
        }
      }
      DownsampleVoxel(&pl_lc, opt.voxel_size / 10.0);
      smp->plptr->reserve(pl_lc.size());
      for (const auto& pp : pl_lc.points) {
        smp->plptr->push_back(pp);
      }
      sess.keyframes.push_back(smp);
      for (int j = 0; j < win; ++j) {
        plbuf.pop_front();
        xxbuf.pop_front();
      }
    }

    sess.triangle = std::make_unique<loop_closure::triangle::TriangleDetector>(
        opt.triangle);
    const int subsize = static_cast<int>(sess.keyframes.size());
    for (int i = 0; i + acsize < subsize; i += mgsize) {
      const int up = i + acsize;
      sensor::PointCloudXYZI pl_btc;
      const ImuState& xc = sess.keyframes[static_cast<size_t>(up - 1)]->x0;
      for (int j = i; j < up; ++j) {
        const ImuState& xj = sess.keyframes[static_cast<size_t>(j)]->x0;
        const Eigen::Vector3d delta_p = xc.R.transpose() * (xj.p - xc.p);
        const Eigen::Matrix3d delta_R = xc.R.transpose() * xj.R;
        for (const auto& ap :
             sess.keyframes[static_cast<size_t>(j)]->plptr->points) {
          Eigen::Vector3d v3(ap.x, ap.y, ap.z);
          v3 = delta_R * v3 + delta_p;
          sensor::PointType pp;
          pp.x = static_cast<float>(v3[0]);
          pp.y = static_cast<float>(v3[1]);
          pp.z = static_cast<float>(v3[2]);
          pl_btc.push_back(pp);
        }
      }
      std::vector<loop_closure::triangle::TriangleDescriptor> stds;
      sensor::PointCloudXYZI plane;
      sess.triangle->GenerateDescriptors(
          pl_btc, sess.keyframes[static_cast<size_t>(up - 1)]->id, &stds,
          &plane);
      sess.triangle->AddDescriptors(stds, plane);
    }
    sess.triangle->DisableNearSkip();
    loaded_sessions.push_back(std::move(sess));
    std::cerr << "PreviousMapRead done: " << root
              << " keyframes=" << loaded_sessions.back().keyframes.size()
              << " btc=" << loaded_sessions.back().triangle->database_size()
              << "\n";
  }
  if (loaded_sessions.empty()) {
    return false;
  }
  *sessions = std::move(loaded_sessions);
  if (edges != nullptr) {
    *edges = std::move(loaded_edges);
  }
  return true;
}

}  // namespace vxslam
}  // namespace atlas
