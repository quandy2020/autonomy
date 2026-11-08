/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ATLAS_BACKEND_LOOP_CLOSURE_LOOP_CLOSURE_OPTIONS_HPP_
#define ATLAS_BACKEND_LOOP_CLOSURE_LOOP_CLOSURE_OPTIONS_HPP_

namespace atlas {
namespace backend {
namespace loop_closure {

struct LoopClosureOptions {
  int useful_corner_num = 30;
  float plane_merge_normal_thre = 0.1f;
  float plane_merge_dis_thre = 0.3f;
  float plane_detection_thre = 0.01f;
  float voxel_size = 1.0f;
  int voxel_init_num = 10;
  int proj_plane_num = 3;
  float proj_image_resolution = 0.5f;
  float proj_image_high_inc = 0.5f;
  float proj_dis_min = 0.f;
  float proj_dis_max = 5.f;
  float summary_min_thre = 10.f;
  int line_filter_enable = 0;
  float descriptor_near_num = 10.f;
  float descriptor_min_len = 1.f;
  float descriptor_max_len = 10.f;
  float non_max_suppression_radius = 3.0f;
  float std_side_resolution = 0.2f;
  int skip_near_num = 20;
  int candidate_num = 50;
  float rough_dis_threshold = 0.03f;
  float similarity_threshold = 0.7f;
  float icp_threshold = 0.5f;
  float normal_threshold = 0.1f;
  float dis_threshold = 0.3f;
  bool touch_filter_enable = false;
};

}  // namespace loop_closure
}  // namespace backend
}  // namespace atlas

#endif  // ATLAS_BACKEND_LOOP_CLOSURE_LOOP_CLOSURE_OPTIONS_HPP_
