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
 * @file parser.hpp
 * @brief Fields2Cover parser.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <string>
#include <algorithm>
#include <vector>
#include "autonomy/planning/planner/fields2cover/types.hpp"
#include "autonomy/planning/planner/fields2cover/utils/transformation.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {


/**
 * @class autonomy::planning::planner::fields2cover::Parser
 * @brief Class to parse files of the OGC Standards to our custom format.
 */
class Parser {
 public:
    /**
     * @brief Import GML fields into @p fields.
     * @param[in] file Path to the GML file.
     * @param[out] fields Parsed fields appended from @p file.
     */
    static void ImportGml(const std::string& file, F2CFields& fields);

    /**
     * @brief Import a single field from a GML file.
     * @param[in] file Path to the GML file.
     * @param[in] fail_silently When true, return an empty field instead of throwing.
     * @return Parsed field.
     */
    static F2CField ImportFieldGml(
        const std::string& file, bool fail_silently = false);

    /**
     * @brief Import JSON fields into @p fields.
     * @param[in] file Path to the JSON file.
     * @param[out] fields Parsed fields written from @p file.
     * @return 0 on success; a negative value on parse failure.
     */
    static int ImportJson(const std::string& file, F2CFields& fields);

    /**
     * @brief Import a cell from JSON.
     * @param[in] file Path to the JSON file.
     * @return Parsed cell.
     */
    static F2CCell ImportCellJson(const std::string& file);

    /**
     * @brief Import swaths from JSON.
     * @param[in] file Path to the JSON file.
     * @return Parsed swaths.
     */
    static F2CSwaths ImportSwathsJson(const std::string& file);

    /**
     * @brief Import strips from JSON.
     * @param[in] file Path to the JSON file.
     * @return Parsed strips.
     */
    static F2CStrips ImportStripsJson(const std::string& file);
};

}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy

