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
 * @file loopshaping_property_tree.hpp
 * @brief loopshaping property tree (bolza).
 */


#include <bolza/core/misc/property_tree.hpp>
#include <bolza/core/types.hpp>

#include "loopshaping_definition.hpp"
#include "loopshaping_filter.hpp"

namespace bolza {
namespace loopshaping_property_tree {

/**
 * @brief Read SISOFilter.
 * @param [in] pt: Pt.
 * @param [in] filterName: Filter Name.
 * @param [in] invert: Invert.
 * @return Result value.
 */
Filter ReadSISOFilter(const bolza::property_tree::ptree& pt, std::string filterName, bool invert = false);

/**
 * @brief Read MIMOFilter.
 * @param [in] pt: Pt.
 * @param [in] filterName: Filter Name.
 * @param [in] invert: Invert.
 * @return Result value.
 */
Filter ReadMIMOFilter(const bolza::property_tree::ptree& pt, std::string filterName, bool invert = false);

/**
 * @brief Load.
 * @param [in] settingsFile: Settings File.
 * @return Owning pointer.
 */
std::shared_ptr<LoopshapingDefinition> Load(const std::string& settingsFile);

}  // namespace loopshaping_property_tree
}  // namespace bolza
