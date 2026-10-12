/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file penalties.hpp
 * @brief penalties (bolza).
 */


// soft equalities
#include <bolza/core/penalties/penalties/quadratic_penalty.hpp>
#include <bolza/core/penalties/penalties/smooth_absolute_penalty.hpp>

// soft inequalities
#include <bolza/core/penalties/penalties/double_sided_penalty.hpp>
#include <bolza/core/penalties/penalties/relaxed_barrier_penalty.hpp>
#include <bolza/core/penalties/penalties/squared_hinge_penalty.hpp>

// hard equalities
#include <bolza/core/penalties/augmented/quadratic_penalty.hpp>
#include <bolza/core/penalties/augmented/smooth_absolute_penalty.hpp>

// hard inequalities
#include <bolza/core/penalties/augmented/modified_relaxed_barrier_penalty.hpp>
#include <bolza/core/penalties/augmented/slackness_squared_hinge_penalty.hpp>

// load functions
namespace bolza {
namespace loadData {

template <class ConfigType>
/**
 * @brief Load Penalty Config.
 * @param [in] fileName: File Name.
 * @param [in] fieldName: Field Name.
 * @param [in,out] config: Config.
 * @param [in] verbose: Verbose.
 */
void LoadPenaltyConfig(const std::string& fileName, const std::string& fieldName, ConfigType& config, bool verbose = true);

}  // namespace loadData
}  // namespace bolza
