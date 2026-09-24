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
 * @file tiled_cloud.hpp
 * @brief Split a world cloud into XY tiles on disk and read them back.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MAP_DENSE_TILED_CLOUD_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MAP_DENSE_TILED_CLOUD_HPP_

#include <string>

#include "autonomy/localization/atlas/common/types.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @brief Write `directory/tiles/*.tile`. Each tile is one XY chunk.
 * @param directory Map directory. Created when missing.
 * @param cloud World-frame points.
 * @param chunk_size Tile edge length in meters.
 * @return false when the cloud is empty or the directory cannot be written.
 */
bool SaveTiledCloud(const std::string& directory, const PointCloud& cloud,
                    double chunk_size);

/**
 * @brief Read every tile under `directory/tiles`.
 * @param directory Map directory.
 * @param[out] cloud Replaced with the loaded points.
 * @return false when no tile could be read.
 */
bool LoadTiledCloud(const std::string& directory, PointCloud* cloud);

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MAP_DENSE_TILED_CLOUD_HPP_
