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

/**
 * @file pcd_io.hpp
 * @brief Minimal PCD reader (ASCII / binary) for apps/datasets (PCL-free).
 */

#ifndef ARGUS_APPS_DATASETS_PCD_IO_HPP_
#define ARGUS_APPS_DATASETS_PCD_IO_HPP_

#include <string>

#include "argus/sensor/point_types.hpp"

namespace argus {
namespace datasets {

/**
 * @brief Load XYZ (+ intensity if present) from a PCD v0.7 file.
 * @param path Filesystem path to the PCD file.
 * @return Shared cloud on success; null pointer if the file cannot be read.
 */
sensor::PointCloudXYZI::Ptr LoadPcdXYZI(const std::string& path);

}  // namespace datasets
}  // namespace argus

#endif  // ARGUS_APPS_DATASETS_PCD_IO_HPP_
