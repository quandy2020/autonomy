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
 * @file types.hpp
 * @brief Fields2Cover types.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <ogr_geometry.h>
#include <vector>


#include "autonomy/planning/planner/fields2cover/types/point.hpp"
#include "autonomy/planning/planner/fields2cover/types/multi_point.hpp"
#include "autonomy/planning/planner/fields2cover/types/linear_ring.hpp"
#include "autonomy/planning/planner/fields2cover/types/line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/multi_line_string.hpp"
#include "autonomy/planning/planner/fields2cover/types/cell.hpp"
#include "autonomy/planning/planner/fields2cover/types/cells.hpp"

#include "autonomy/planning/planner/fields2cover/types/strip.hpp"
#include "autonomy/planning/planner/fields2cover/types/field.hpp"

#include "autonomy/planning/planner/fields2cover/types/swath.hpp"
#include "autonomy/planning/planner/fields2cover/types/swaths.hpp"
#include "autonomy/planning/planner/fields2cover/types/swaths_by_cells.hpp"
#include "autonomy/planning/planner/fields2cover/types/route.hpp"
#include "autonomy/planning/planner/fields2cover/types/path.hpp"
#include "autonomy/planning/planner/fields2cover/types/robot.hpp"

#include "autonomy/planning/planner/fields2cover/types/graph.hpp"
#include "autonomy/planning/planner/fields2cover/types/graph2d.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {

using F2CRobot = types::Robot;


using F2CPoint = types::Point;
using F2CMultiPoint = types::MultiPoint;
using F2CLineString = types::LineString;
using F2CLinearRing = types::LinearRing;
using F2CMultiLineString = types::MultiLineString;
using F2CCell = types::Cell;
using F2CCells = types::Cells;
using F2CStrip = types::Strip;
using F2CField = types::Field;
using F2CPath = types::Path;
using F2CPathState = types::PathState;
using F2CRoute = types::Route;
using F2CSwath = types::Swath;
using F2CSwaths = types::Swaths;
using F2CSwathsByCells = types::SwathsByCells;
using F2CGraph = types::Graph;
using F2CGraph2D = types::Graph2D;

using F2CRobots = std::vector<F2CRobot>;
using F2CStrips = std::vector<F2CStrip>;
using F2CFields = std::vector<F2CField>;

}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy
