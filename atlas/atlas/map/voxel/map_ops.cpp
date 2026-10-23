/*
 * Copyright 2026 The Openbot Authors
 */
#include "map/voxel/map_ops.hpp"

#include <thread>

namespace atlas {
namespace vxslam {

void MultiMarginalize(std::unordered_map<VoxelKey, SlidingOctree*>* feat_map,
                      double jour, int win_count, std::vector<ImuState>* xs,
                      LidarFactor* voxopt, std::vector<FrameWindow*>* sw,
                      int thread_num) {
  if (feat_map == nullptr || xs == nullptr || voxopt == nullptr ||
      sw == nullptr) {
    return;
  }
  int thd_num = thread_num > 0 ? thread_num : 1;
  const int g_size = static_cast<int>(feat_map->size());
  if (g_size == 0) {
    return;
  }
  if (g_size < thd_num) {
    thd_num = 1;
  }
  std::vector<std::vector<SlidingOctree*>> octs(static_cast<size_t>(thd_num));
  const double part = 1.0 * g_size / thd_num;
  int cnt = 0;
  for (auto& kv : *feat_map) {
    kv.second->jour = static_cast<float>(jour);
    octs[static_cast<size_t>(cnt)].push_back(kv.second);
    if (static_cast<double>(octs[static_cast<size_t>(cnt)].size()) >= part &&
        cnt < thd_num - 1) {
      ++cnt;
    }
  }

  auto margi_func = [&](std::vector<SlidingOctree*>* oct) {
    for (SlidingOctree* oc : *oct) {
      oc->Marginalize(win_count, 1, *xs, *voxopt);
    }
  };

  std::vector<std::thread> threads;
  threads.reserve(static_cast<size_t>(thd_num - 1));
  for (int i = 1; i < thd_num; ++i) {
    threads.emplace_back(margi_func, &octs[static_cast<size_t>(i)]);
  }
  margi_func(&octs[0]);
  for (auto& th : threads) {
    th.join();
  }

  for (auto iter = feat_map->begin(); iter != feat_map->end();) {
    if (iter->second->isexist) {
      ++iter;
    } else {
      iter->second->clear_slwd(*sw);
      iter = feat_map->erase(iter);
    }
  }
}

void MultiRecut(std::unordered_map<VoxelKey, SlidingOctree*>* feat_map,
                int win_count, std::vector<ImuState>* xs, LidarFactor* voxopt,
                std::vector<std::vector<FrameWindow*>>* sws, int thread_num) {
  if (feat_map == nullptr || xs == nullptr || voxopt == nullptr ||
      sws == nullptr || sws->empty()) {
    return;
  }
  int thd_num = thread_num > 0 ? thread_num : 1;
  if (static_cast<int>(sws->size()) < thd_num) {
    sws->resize(static_cast<size_t>(thd_num));
  }
  const int g_size = static_cast<int>(feat_map->size());
  if (g_size < thd_num) {
    thd_num = 1;
  }
  std::vector<std::vector<SlidingOctree*>> octss(static_cast<size_t>(thd_num));
  const double part = 1.0 * g_size / thd_num;
  int cnt = 0;
  for (auto& kv : *feat_map) {
    octss[static_cast<size_t>(cnt)].push_back(kv.second);
    if (static_cast<double>(octss[static_cast<size_t>(cnt)].size()) >= part &&
        cnt < thd_num - 1) {
      ++cnt;
    }
  }

  auto recut_func = [&](int idx) {
    for (SlidingOctree* oc : octss[static_cast<size_t>(idx)]) {
      oc->Recut(win_count, *xs, (*sws)[static_cast<size_t>(idx)]);
    }
  };

  std::vector<std::thread> threads;
  for (int i = 1; i < thd_num; ++i) {
    threads.emplace_back(recut_func, i);
  }
  recut_func(0);
  for (auto& th : threads) {
    th.join();
  }

  for (size_t i = 1; i < sws->size(); ++i) {
    (*sws)[0].insert((*sws)[0].end(), (*sws)[i].begin(), (*sws)[i].end());
    (*sws)[i].clear();
  }
  for (auto& kv : *feat_map) {
    kv.second->CollectFactors(*voxopt);
  }
}

}  // namespace vxslam
}  // namespace atlas
