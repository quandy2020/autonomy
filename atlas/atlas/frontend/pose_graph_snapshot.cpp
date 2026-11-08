/*
 * Copyright 2026 The Openbot Authors
 */
#include "frontend/pose_graph_snapshot.hpp"

namespace atlas {
namespace frontend {

bool PoseGraphSnapshot::empty() const {
  return scan_poses.empty() && keyframes.empty();
}

void PoseGraphSnapshot::BuildViews() {
  scan_views.clear();
  keyframe_views.clear();
  keyframe_view_pointers.clear();
  scan_views.reserve(scan_poses.size());
  keyframe_views.reserve(keyframes.size());
  keyframe_view_pointers.reserve(keyframes.size());
  for (auto& session_scans : scan_poses) {
    scan_views.push_back(&session_scans);
  }
  for (const auto& session_keyframes : keyframes) {
    keyframe_views.emplace_back();
    auto& keyframe_view = keyframe_views.back();
    keyframe_view.reserve(session_keyframes.size());
    for (const auto& keyframe : session_keyframes) {
      keyframe_view.push_back(keyframe.get());
    }
  }
  for (auto& keyframe_view : keyframe_views) {
    keyframe_view_pointers.push_back(&keyframe_view);
  }
}

}  // namespace frontend
}  // namespace atlas
