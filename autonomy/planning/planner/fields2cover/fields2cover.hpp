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
 * @file fields2cover.hpp
 * @brief Fields2Cover fields2cover.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include "autonomy/planning/planner/fields2cover/types.hpp"


#include "autonomy/planning/planner/fields2cover/utils/random.hpp"
#include "autonomy/planning/planner/fields2cover/utils/spline.hpp"
#include "autonomy/planning/planner/fields2cover/utils/transformation.hpp"
#include "autonomy/planning/planner/fields2cover/utils/parser.hpp"


#include "autonomy/planning/planner/fields2cover/objectives/base_objective.hpp"

#include "autonomy/planning/planner/fields2cover/objectives/hg_obj/hg_objective.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/hg_obj/rem_area.hpp"

#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/sg_objective.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/n_swath.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/n_swath_modified.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/field_coverage.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/overlaps.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/sg_obj/swath_length.hpp"

#include "autonomy/planning/planner/fields2cover/objectives/rp_obj/rp_objective.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/rp_obj/direct_dist_path_obj.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/rp_obj/complete_turn_path_obj.hpp"

#include "autonomy/planning/planner/fields2cover/objectives/pp_obj/pp_objective.hpp"
#include "autonomy/planning/planner/fields2cover/objectives/pp_obj/path_length.hpp"

#include "autonomy/planning/planner/fields2cover/objectives/decomp_obj/decomp_objective.hpp"


#include "autonomy/planning/planner/fields2cover/decomposition/decomposition_base.hpp"
#include "autonomy/planning/planner/fields2cover/decomposition/trapezoidal_decomp.hpp"
#include "autonomy/planning/planner/fields2cover/decomposition/boustrophedon_decomp.hpp"

#include "autonomy/planning/planner/fields2cover/headland_generator/headland_generator_base.hpp"
#include "autonomy/planning/planner/fields2cover/headland_generator/constant_headland.hpp"
#include "autonomy/planning/planner/fields2cover/headland_generator/corridor_headland.hpp"
#include "autonomy/planning/planner/fields2cover/headland_generator/required_headland.hpp"

#include "autonomy/planning/planner/fields2cover/swath_generator/swath_generator_base.hpp"
#include "autonomy/planning/planner/fields2cover/swath_generator/brute_force.hpp"


#include "autonomy/planning/planner/fields2cover/route_planning/single_cell_swaths_order_base.hpp"
#include "autonomy/planning/planner/fields2cover/route_planning/boustrophedon_order.hpp"
#include "autonomy/planning/planner/fields2cover/route_planning/snake_order.hpp"
#include "autonomy/planning/planner/fields2cover/route_planning/spiral_order.hpp"
#include "autonomy/planning/planner/fields2cover/route_planning/custom_order.hpp"

#include "autonomy/planning/planner/fields2cover/route_planning/route_planner_base.hpp"

#include "autonomy/planning/planner/fields2cover/path_planning/turning_base.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/dubins_curves.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/dubins_curves_cc.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/reeds_shepp_curves.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/reeds_shepp_curves_hc.hpp"
#include "autonomy/planning/planner/fields2cover/path_planning/path_planning.hpp"


/**
 * @brief Main namespace of the fields2cover library.
 */
namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {



/**
 * @brief Types used by fields2cover library.
 */
namespace types {}

/**
 * @brief Objective algorithms' namespace.
 */
namespace obj {}

/**
 * @brief Swath Generator algorithms' namespace.
 */
namespace sg {}

enum class SGAlg {
  BRUTE_FORCE = 0,  //Default
  GIVEN_ANGLE = 1,
};

enum class SGObjFunc {
  N_SWATH_MOD = 0,  // Default
  N_SWATH = 1,
  SWATH_LENGTH = 2,
  FIELD_COV = 3,
  OVERLAPS = 4,
};

/**
 * @brief Route planning algorithms' namespace.
 */
namespace rp {}

enum class RPAlg {
  SHORTEST_ROUTE = 0,  // Default
  //BOUSTROPHEDON = 1,
  //SNAKE = 2,
  //SPIRAL = 3,
  //CUSTOM = 4,
};

/**
 * @brief Path planning algorithms' namespace.
 */
namespace pp {}

enum class PPAlg {
  DUBINS = 0,  // Default
  DUBINS_CC = 1,
  REEDS_SHEPP = 2,
  REEDS_SHEPP_HC = 3,
};


/**
 * @brief Headland Generator algorithms' namespace.
 */
namespace hg {}

enum class HGAlg {
  NONE = 0,
  CONST = 1,  // Default
};


/**
 * @brief Decomposition algorithms' namespace.
 */
namespace decomp {}

enum class DecompAlg {
  NONE = 0,  // Default
  // TRAPEZOIDAL = 1,
  // BOUSTROPHEDON = 2,
};

struct Options {
  DecompAlg decomp_alg {DecompAlg::NONE};
  double decomp_angle {0.0};

  HGAlg hg_alg {HGAlg::CONST};
  int hg_swaths {3};

  SGAlg sg_alg {SGAlg::BRUTE_FORCE};
  SGObjFunc sg_obj {SGObjFunc::N_SWATH_MOD};
  double sg_angle {0.0};

  RPAlg rp_alg {RPAlg::SHORTEST_ROUTE};

  PPAlg pp_alg {PPAlg::DUBINS};
};

F2CRoute PlanCovRoute(const F2CRobot& robot, const F2CCell& cell,
    const Options& opt = Options());
F2CRoute PlanCovRoute(const F2CRobot& robot, const F2CCells& cells,
    const Options& opt = Options());
F2CRoute PlanCovRoute(const F2CRobot& robot, const F2CField& field,
    bool local_crs);
F2CRoute PlanCovRoute(const F2CRobot& robot, const F2CField& field,
    const Options& opt = Options(), bool local_crs = true);
F2CPath PlanCovPath(const F2CRobot& robot, const F2CCell& cell,
    const Options& opt = Options());
F2CPath PlanCovPath(const F2CRobot& robot, const F2CCells& cells,
    const Options& opt = Options());
F2CPath PlanCovPath(const F2CRobot& robot, const F2CField& field,
    bool local_crs);
F2CPath PlanCovPath(const F2CRobot& robot, const F2CField& field,
    const Options& opt = Options(), bool local_crs = true);

}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy

