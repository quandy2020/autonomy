/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#pragma once

/**
 * @file upload.hpp
 * @brief Copy episodes whose manifest.upload is true. P0/P1 always; P2/P3 sampled.
 */

#include <string>

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Counts from one UploadEpisodes run.
 */
struct UploadReport {
    int copied{0};       ///< Successfully copied episode dirs.
    int skipped{0};      ///< Manifest said do not upload.
    int failed{0};       ///< I/O failures.
    std::string reason;  ///< First failure reason, if any.
};

/**
 * @brief Copy admitted episode directories from src_dir to dest_dir.
 * @param [in] src_dir: Local episode root.
 * @param [in] dest_dir: Destination root.
 * @param [out] report: Optional counters.
 * @return True if dest_dir is usable and no hard failure stopped the walk.
 */
bool UploadEpisodes(const std::string& src_dir, const std::string& dest_dir,
                    UploadReport* report = nullptr);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
