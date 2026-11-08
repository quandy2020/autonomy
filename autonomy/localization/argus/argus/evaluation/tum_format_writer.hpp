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
 * @file tum_format_writer.hpp
 * @brief Load and write TUM RGB-D trajectory text files.
 */

#ifndef ARGUS_EVALUATION_TUM_FORMAT_WRITER_HPP_
#define ARGUS_EVALUATION_TUM_FORMAT_WRITER_HPP_

#include <string>
#include <vector>

#include "argus/common/rigid_transform.hpp"

namespace argus {
namespace evaluation {

/**
 * @brief Timestamped body pose used as one TUM trajectory element.
 * @var stamp_sec Time stamp in seconds (same clock as the trajectory file).
 * @var T_w_b     Rigid transform from body to world / map frame.
 */
struct TimedPose {
    double stamp_sec = 0.0;
    transform::Rigid3d T_w_b = transform::Rigid3d::Identity();
};

/**
 * @brief Load a TUM trajectory file (`t tx ty tz qx qy qz qw` per line).
 * Lines that fail to parse are skipped. Quaternion is normalized when its
 * norm is non-zero; otherwise identity rotation is used.
 * @param[in]  path Filesystem path to the TUM text file.
 * @param[out] out  Cleared then filled with parsed poses. Must be non-null.
 * @return false if @p out is null, the file cannot be opened, or no poses are
 *         parsed (empty result); true when at least one pose is loaded.
 */
bool LoadTumTrajectory(const std::string& path, std::vector<TimedPose>* out);

/**
 * @brief Write poses in TUM format (`t tx ty tz qx qy qz qw`).
 * @param[in] path  Destination filesystem path (created or overwritten).
 * @param[in] poses Poses to write in order; may be empty (creates an empty
 *                  file on success).
 * @return false if the file cannot be opened for writing; true on success.
 */
bool WriteTumTrajectory(const std::string& path,
                        const std::vector<TimedPose>& poses);

}  // namespace evaluation

/** @brief Compatibility aliases for apps / matching docs. */
namespace relocalization {
using TimedPose = ::argus::evaluation::TimedPose;
using ::argus::evaluation::LoadTumTrajectory;
using ::argus::evaluation::WriteTumTrajectory;
}  // namespace relocalization

}  // namespace argus

#endif  // ARGUS_EVALUATION_TUM_FORMAT_WRITER_HPP_
