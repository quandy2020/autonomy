/*
 * Copyright 2025 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/common/network/options.hpp"

namespace autonomy {
namespace common {
namespace network {

void InferenceOptions::Merge(const InferenceOptions& o) {
    if (!o.backend_id.empty()) {
        backend_id = o.backend_id;
    }
    if (!o.model_path.empty()) {
        model_path = o.model_path;
    }
    if (o.onnx.intra_op_num_threads > 0) {
        onnx.intra_op_num_threads = o.onnx.intra_op_num_threads;
    }
    if (o.onnx.inter_op_num_threads > 0) {
        onnx.inter_op_num_threads = o.onnx.inter_op_num_threads;
    }
    if (o.onnx.graph_optimization_level >= 0) {
        onnx.graph_optimization_level = o.onnx.graph_optimization_level;
    }
    if (o.tensorrt.device_id.has_value() ||
        o.tensorrt.max_workspace_size_mb != 0 ||
        !o.tensorrt.trt_cache_path.empty()) {
        if (o.tensorrt.device_id.has_value()) {
            tensorrt.device_id = o.tensorrt.device_id;
        }
        if (o.tensorrt.max_workspace_size_mb != 0) {
            tensorrt.max_workspace_size_mb = o.tensorrt.max_workspace_size_mb;
        }
        if (!o.tensorrt.trt_cache_path.empty()) {
            tensorrt.trt_cache_path = o.tensorrt.trt_cache_path;
        }
    }
    if (o.rknn.device_id.has_value()) {
        rknn.device_id = o.rknn.device_id;
    }
    if (o.rknn.core_mask.has_value()) {
        rknn.core_mask = o.rknn.core_mask;
    }
}

}  // namespace network
}  // namespace common
}  // namespace autonomy
