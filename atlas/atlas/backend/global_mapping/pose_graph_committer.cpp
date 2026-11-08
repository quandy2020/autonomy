/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/global_mapping/pose_graph_committer.hpp"

#include <cmath>
#include <cstddef>

namespace atlas {
namespace backend {
namespace global_mapping {
namespace {

using ::atlas::frontend::inertial::InertialState;
using ::atlas::frontend::odometry::ScanPosePtr;
using ::atlas::map::Keyframe;

bool HasFiniteState(const InertialState& state) {
  return std::isfinite(state.t) && state.R.allFinite() && state.p.allFinite() &&
         state.v.allFinite() && state.bg.allFinite() && state.ba.allFinite() &&
         state.g.allFinite() && state.cov.allFinite();
}

bool SessionMatches(const std::vector<ScanPosePtr>& optimized_scans,
                    const std::vector<Keyframe*>& optimized_keyframes,
                    const std::vector<ScanPosePtr>& current_scans,
                    const std::vector<Keyframe*>& current_keyframes) {
  if (optimized_scans.size() != current_scans.size() ||
      optimized_keyframes.size() != current_keyframes.size()) {
    return false;
  }
  for (std::size_t index = 0; index < optimized_scans.size(); ++index) {
    if (static_cast<bool>(optimized_scans[index]) !=
            static_cast<bool>(current_scans[index]) ||
        (optimized_scans[index] &&
         !HasFiniteState(optimized_scans[index]->x))) {
      return false;
    }
  }
  for (std::size_t index = 0; index < optimized_keyframes.size(); ++index) {
    const Keyframe* optimized_keyframe = optimized_keyframes[index];
    const Keyframe* current_keyframe = current_keyframes[index];
    if (static_cast<bool>(optimized_keyframe) !=
        static_cast<bool>(current_keyframe)) {
      return false;
    }
    if (optimized_keyframe != nullptr &&
        (optimized_keyframe->keyframe_id != current_keyframe->keyframe_id ||
         optimized_keyframe->session_id != current_keyframe->session_id ||
         !HasFiniteState(optimized_keyframe->inertial_state))) {
      return false;
    }
  }
  return true;
}

void ApplySessionPoses(const std::vector<ScanPosePtr>& optimized_scans,
                       const std::vector<Keyframe*>& optimized_keyframes,
                       const std::vector<ScanPosePtr>& current_scans,
                       const std::vector<Keyframe*>& current_keyframes) {
  for (std::size_t index = 0; index < optimized_scans.size(); ++index) {
    if (optimized_scans[index]) {
      current_scans[index]->SetState(optimized_scans[index]->x.R,
                                     optimized_scans[index]->x.p);
    }
  }
  for (std::size_t index = 0; index < optimized_keyframes.size(); ++index) {
    if (optimized_keyframes[index]) {
      current_keyframes[index]->inertial_state =
          optimized_keyframes[index]->inertial_state;
    }
  }
}

}  // namespace

bool PoseGraphCommitter::Commit(
    const std::vector<std::vector<ScanPosePtr>*>& optimized_scans,
    const std::vector<std::vector<Keyframe*>*>& optimized_keyframes,
    const std::vector<std::vector<ScanPosePtr>*>& current_scans,
    const std::vector<std::vector<Keyframe*>*>& current_keyframes) const {
  if (optimized_scans.empty() ||
      optimized_scans.size() != optimized_keyframes.size() ||
      optimized_scans.size() != current_scans.size() ||
      optimized_scans.size() != current_keyframes.size()) {
    return false;
  }
  for (std::size_t session_index = 0; session_index < optimized_scans.size();
       ++session_index) {
    if (optimized_scans[session_index] == nullptr ||
        optimized_keyframes[session_index] == nullptr ||
        current_scans[session_index] == nullptr ||
        current_keyframes[session_index] == nullptr ||
        !SessionMatches(*optimized_scans[session_index],
                        *optimized_keyframes[session_index],
                        *current_scans[session_index],
                        *current_keyframes[session_index])) {
      return false;
    }
  }
  for (std::size_t session_index = 0; session_index < optimized_scans.size();
       ++session_index) {
    ApplySessionPoses(
        *optimized_scans[session_index], *optimized_keyframes[session_index],
        *current_scans[session_index], *current_keyframes[session_index]);
  }
  return true;
}

}  // namespace global_mapping
}  // namespace backend
}  // namespace atlas
