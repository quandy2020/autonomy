/*
 * Copyright 2026 The Openbot Authors
 */
#include "backend/loop_closure/lidar/loop_update.hpp"

#include <algorithm>
#include <memory>

#include "frontend/lidar/odometry/point_cov.hpp"
#include "glog/logging.h"

namespace atlas {
namespace vxslam {

void LoopUpdateMap(std::unordered_map<VoxelKey, SlidingOctree*>* feat_map,
                   const std::vector<ImuState>& x_buf,
                   const std::vector<PointVarListPtr>& pvec_buf, int win_count,
                   int win_size, std::vector<FrameWindow*>* sw_pool) {
  if (feat_map == nullptr || sw_pool == nullptr) {
    return;
  }
  LOG(INFO) << "LoopUpdateMap win_count=" << win_count
            << " pool=" << sw_pool->size();

  std::vector<SlidingOctree*> octos;
  for (auto& kv : *feat_map) {
    kv.second->tras_ptr(octos);
    kv.second->clear_slwd(*sw_pool);
    delete kv.second;
  }
  for (SlidingOctree* o : octos) {
    delete o;
  }
  feat_map->clear();
  octos.clear();

  const int n = std::min(win_count, static_cast<int>(pvec_buf.size()));
  for (int i = 0; i < n; ++i) {
    if (!pvec_buf[static_cast<size_t>(i)]) {
      continue;
    }
    VectorList<3> pwld;
    PointVarList tmp = *pvec_buf[static_cast<size_t>(i)];
    UpdateWorldVars(x_buf[static_cast<size_t>(i)], &tmp, &pwld);
    auto ptr = std::make_shared<PointVarList>(std::move(tmp));
    std::unordered_map<VoxelKey, SlidingOctree*> feat_tem;
    CutVoxel(*feat_map, ptr, i, feat_tem, win_size, pwld, *sw_pool);
    for (auto& kv : feat_tem) {
      if (kv.second) {
        // non-const buffer for Recut
        std::vector<ImuState> xs = x_buf;
        kv.second->Recut(win_count, xs, *sw_pool);
      }
    }
  }
}

}  // namespace vxslam
}  // namespace atlas
