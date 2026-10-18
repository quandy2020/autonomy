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
 * @file dataset_common.hpp
 * @brief Shared helpers for apps/datasets readers (header-only).
 */

#ifndef ARGUS_APPS_DATASETS_DATASET_COMMON_HPP_
#define ARGUS_APPS_DATASETS_DATASET_COMMON_HPP_

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace argus {
namespace datasets {
namespace detail {

namespace fs = std::filesystem;

/**
 * @brief Compare a path extension to a literal (e.g. ".png").
 * @param p File path.
 * @param ext Expected extension including the dot.
 * @return True if `p.extension()` equals `ext`.
 */
inline bool HasExt(const fs::path& p, const char* ext) {
  return p.extension() == ext;
}

/**
 * @brief List sorted filename stems for regular files with a given extension.
 * @param dir Directory to scan.
 * @param ext File extension including the dot.
 * @return Sorted stems; empty if `dir` is not a directory.
 */
inline std::vector<std::string> ListStems(const fs::path& dir,
                                          const char* ext) {
  std::vector<std::string> stems;
  if (!fs::is_directory(dir)) {
    return stems;
  }
  for (const auto& e : fs::directory_iterator(dir)) {
    if (!e.is_regular_file()) {
      continue;
    }
    if (HasExt(e.path(), ext)) {
      stems.push_back(e.path().stem().string());
    }
  }
  std::sort(stems.begin(), stems.end());
  return stems;
}

/**
 * @brief Resolve EuRoC-style sensor directory.
 * @param base Sensor root (e.g. cam0 or ouster/lidar).
 * @return `base/data` when that subdirectory exists, otherwise `base`.
 */
inline fs::path ResolveDataDir(const fs::path& base) {
  const fs::path nested = base / "data";
  if (fs::is_directory(nested)) {
    return nested;
  }
  return base;
}

/**
 * @brief Parse a filename stem as a timestamp in seconds.
 *
 * Integer ns (≥1e15) → /1e9; integer us (≥1e12) → /1e6; otherwise treated as seconds.
 *
 * @param stem Filename stem (no extension).
 * @return Time in seconds, or 0.0 if parsing fails.
 */
inline double StemToSeconds(const std::string& stem) {
  if (stem.empty()) {
    return 0.0;
  }
  char* end = nullptr;
  const double v = std::strtod(stem.c_str(), &end);
  if (end == stem.c_str()) {
    return 0.0;
  }
  // Pure integer path via magnitude.
  if (std::strchr(stem.c_str(), '.') == nullptr) {
    if (v >= 1e15) {
      return v * 1e-9;
    }
    if (v >= 1e12) {
      return v * 1e-6;
    }
  }
  return v;
}

/**
 * @brief Convert a list of stems to timestamps via StemToSeconds.
 * @param stems Sorted or unsorted filename stems.
 * @return Parallel vector of times in seconds.
 */
inline std::vector<double> StemsToTimes(const std::vector<std::string>& stems) {
  std::vector<double> times;
  times.reserve(stems.size());
  for (const auto& s : stems) {
    times.push_back(StemToSeconds(s));
  }
  return times;
}

/**
 * @brief Find the nearest index in a sorted time vector.
 * @param times Sorted timestamps (seconds).
 * @param t Query time (seconds).
 * @param max_dt Maximum allowed |times[i] - t| for a match.
 * @return Index of the best match, or -1 if none within `max_dt`.
 */
inline int NearestIndex(const std::vector<double>& times, double t,
                        double max_dt) {
  if (times.empty()) {
    return -1;
  }
  const auto it = std::lower_bound(times.begin(), times.end(), t);
  int best = -1;
  double best_dt = max_dt;
  auto consider = [&](int i) {
    if (i < 0 || i >= static_cast<int>(times.size())) {
      return;
    }
    const double dt = std::abs(times[static_cast<std::size_t>(i)] - t);
    if (dt <= best_dt) {
      best_dt = dt;
      best = i;
    }
  };
  if (it == times.end()) {
    consider(static_cast<int>(times.size()) - 1);
  } else {
    const int i = static_cast<int>(it - times.begin());
    consider(i);
    consider(i - 1);
  }
  return best;
}

/**
 * @brief Locate an image file for a given stem under `dir`.
 * @param dir Directory containing images.
 * @param stem Filename stem (timestamp or index).
 * @return First existing path among .png/.jpg/.jpeg/.bmp, or empty if none.
 */
inline fs::path FindImageFile(const fs::path& dir, const std::string& stem) {
  for (const char* ext : {".png", ".jpg", ".jpeg", ".bmp"}) {
    const fs::path p = dir / (stem + ext);
    if (fs::exists(p)) {
      return p;
    }
  }
  return {};
}

/**
 * @brief Locate a point-cloud file for a given stem under `dir`.
 * @param dir Directory containing clouds.
 * @param stem Filename stem.
 * @return First existing path among .pcd/.bin, or empty if none.
 */
inline fs::path FindCloudFile(const fs::path& dir, const std::string& stem) {
  for (const char* ext : {".pcd", ".bin"}) {
    const fs::path p = dir / (stem + ext);
    if (fs::exists(p)) {
      return p;
    }
  }
  return {};
}

/**
 * @brief Count regular files in a directory with a given extension.
 * @param dir Directory to scan.
 * @param ext File extension including the dot.
 * @return File count, or 0 if `dir` is not a directory.
 */
inline std::size_t CountExt(const fs::path& dir, const char* ext) {
  if (!fs::is_directory(dir)) {
    return 0;
  }
  std::size_t n = 0;
  for (const auto& e : fs::directory_iterator(dir)) {
    if (e.is_regular_file() && HasExt(e.path(), ext)) {
      ++n;
    }
  }
  return n;
}

}  // namespace detail
}  // namespace datasets
}  // namespace argus

#endif  // ARGUS_APPS_DATASETS_DATASET_COMMON_HPP_
