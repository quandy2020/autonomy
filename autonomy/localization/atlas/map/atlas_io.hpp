/*
 * Copyright 2026 The Openbot Authors
 *
 * Lightweight MultiMap JSON archive (ORB-SLAM3 File.saveAtlasTo / loadAtlasFrom
 * role). v5: KF features, dual stereo, covisibility, BoW, cameras, IMU
 * bias/calib/velocity + Preintegrator + prev/next links (near boost subset).
 */

/**
 * @file atlas_io.hpp
 * @brief MultiMap JSON archive: save / load Atlas (lightweight alternative to
 *        ORB boost serialization).
 *
 * Role aligned with ORB-SLAM3 `File::saveAtlasTo` / `loadAtlasFrom`. Current v5
 * includes keyframe features, stereo, covisibility, BoW, cameras, IMU
 * bias/calib/velocity, preintegration, and temporal links.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_MAP_ATLAS_IO_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_MAP_ATLAS_IO_HPP_

#include <string>

#include "autonomy/localization/atlas/map/multi_map.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

/**
 * @brief Serialize a MultiMap to a JSON file.
 * @param multi_map Source multi-map.
 * @param path Output path.
 * @return `true` on success.
 *
 * Writes keyframes (\(T_{cw}\) + features) and map points (xyz + observations).
 *
 * @note
 * @code{.cpp}
 * if (!SaveAtlasJson(atlas, "/tmp/atlas.json")) {
 *   // handle I/O error
 * }
 * @endcode
 */
bool SaveAtlasJson(const MultiMap& multi_map, const std::string& path);

/**
 * @brief Load JSON into a MultiMap (clears current contents).
 * @param[in,out] multi_map Destination; cleared then filled.
 * @param path Input path.
 * @return `true` on success.
 *
 * Restores descriptors / observations / dual cameras / covisibility / BoW /
 * cameras / IMU (v2–v5); pose-only v1 remains supported.
 */
bool LoadAtlasJson(MultiMap* multi_map, const std::string& path);

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_MAP_ATLAS_IO_HPP_
