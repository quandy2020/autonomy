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
 * @file newer_college_options.hpp
 * @brief Reader options for Newer College Dataset dumps.
 */

#ifndef ARGUS_APPS_DATASETS_NEWER_COLLEGE_OPTIONS_HPP_
#define ARGUS_APPS_DATASETS_NEWER_COLLEGE_OPTIONS_HPP_

#include <string>

namespace argus {
namespace datasets {

/** @brief Reader options (camera, sync threshold, optional calibration dir). */
struct NewerCollegeOptions {
    NewerCollegeOptions();

    /**
     * @brief Preferred camera folder. Empty → auto (cam0, else infra1).
     */
    std::string camera;
    double max_image_dt_sec = 0.05;
    /** @brief Optional Kalibr / camchain directory. */
    std::string calibration_dir;
};

}  // namespace datasets
}  // namespace argus

#endif  // ARGUS_APPS_DATASETS_NEWER_COLLEGE_OPTIONS_HPP_
