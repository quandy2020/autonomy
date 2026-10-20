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
 * @file map_package.hpp
 * @brief On-disk Argus map package layout (meta + geometry + reloc prior).
 *
 * Directory layout (version 3):
 *   map_meta.txt      — package header + content fingerprint
 *   session.txt       — MapSession registry / trajectories
 *   map.xyz           — world LiDAR points
 *   prior.xyz         — occupancy prior cloud for BnB
 *   visual.xyz        — sparse visual landmarks
 *   trajectory.tum    — TUM poses
 *   keyframes/        — optional body (or world) clouds for SC + prior LTA
 *     index.txt       — `local_id stem.bin npts cloud_in_world`
 *     000000.bin …    — float32 xyz i; LoadBuiltMap → ExportLoopKeyFrames
 *
 * Writes are atomic (temp file + rename). Load verifies fingerprint when
 * present.
 */

#ifndef ARGUS_MAPPING_MAP_PACKAGE_HPP_
#define ARGUS_MAPPING_MAP_PACKAGE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace argus {
namespace mapping {

/** @brief Bump on breaking package layout changes. */
constexpr int kMapPackageVersion = 3;

struct MapPackageMeta {
    int version = kMapPackageVersion;
    std::string session_id = "map";
    double prior_resolution = 0.5;
    double ikd_downsample_resolution = 0.2;
    std::size_t map_points = 0;
    std::size_t visual_landmarks = 0;
    std::size_t keyframes = 0;
    /** @brief FNV-1a 64-bit of map.xyz bytes; 0 = unset / skip verify. */
    std::uint64_t map_xyz_fnv64 = 0;
};

/**
 * @brief Write `map_meta.txt` header.
 * @param path Output file path.
 * @param meta Package metadata to serialize.
 * @return false on I/O error.
 */
bool SaveMapPackageMeta(const std::string& path, const MapPackageMeta& meta);
/**
 * @brief Parse `map_meta.txt`.
 * @param path Input file path.
 * @param[out] meta Filled metadata on success.
 * @return false on missing file or parse error.
 */
bool LoadMapPackageMeta(const std::string& path, MapPackageMeta* meta);

/**
 * @brief True if directory looks like a saved Argus map package.
 * @param directory Candidate package root.
 * @return true when required markers are present.
 */
bool IsMapPackageDirectory(const std::string& directory);

/**
 * @brief FNV-1a 64-bit fingerprint of a file (0 on read failure).
 * @param path File to hash.
 * @return 64-bit FNV-1a digest, or 0 if unreadable.
 */
std::uint64_t FileFnv64(const std::string& path);

/**
 * @brief Write `data` to `path` via `path.tmp` + rename (crash-safe).
 * @param path Destination file path.
 * @param data Bytes to write atomically.
 * @return false on I/O error.
 */
bool AtomicWriteFile(const std::string& path, const std::string& data);

/**
 * @brief Verify map.xyz fingerprint when meta.map_xyz_fnv64 != 0.
 * @param directory Package root containing `map.xyz`.
 * @param meta Expected meta including fingerprint.
 * @param[out] error Human-readable failure reason when returning false.
 * @return true when verification passes or fingerprint unset.
 */
bool VerifyMapPackage(const std::string& directory, const MapPackageMeta& meta,
                      std::string* error);

}  // namespace mapping
}  // namespace argus

#endif  // ARGUS_MAPPING_MAP_PACKAGE_HPP_
