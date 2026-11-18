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

/**
 * @file fields2cover.cpp
 * @brief Fields2Cover fields2cover implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <utility>
#include "autonomy/planning/planner/fields2cover/fields2cover.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {


/*
std::unique_ptr<autonomy::planning::planner::fields2cover::decomp::DecompositionBase> GetDecompPtr(const Options& opt) {
  std::unique_ptr<autonomy::planning::planner::fields2cover::decomp::DecompositionBase> ptr;
  switch (opt.decomp_alg) {
    case DecompAlg::TRAPEZOIDAL: {
      decomp::TrapezoidalDecomp trap_decomp;
      trap_decomp.SetSplitAngle(opt.decomp_angle);
      ptr = std::make_unique<decomp::TrapezoidalDecomp>(trap_decomp);
      break;
    }
    case DecompAlg::BOUSTROPHEDON: {
      decomp::BoustrophedonDecomp boustrophedon_decomp;
      boustrophedon_decomp.SetSplitAngle(opt.decomp_angle);
      ptr = std::make_unique<decomp::BoustrophedonDecomp>(
          boustrophedon_decomp);
      break;
    }
    default:
      break;
  }
  return ptr;
}
*/

std::unique_ptr<obj::SGObjective> GetSGObjPtr(const Options& opt) {
  std::unique_ptr<obj::SGObjective> ptr;
  switch (opt.sg_obj) {
    case SGObjFunc::N_SWATH_MOD:
      ptr = std::make_unique<obj::NSwathModified>();
      break;
    case SGObjFunc::N_SWATH:
      ptr = std::make_unique<obj::NSwath>();
      break;
    case SGObjFunc::SWATH_LENGTH:
      ptr = std::make_unique<obj::SwathLength>();
      break;
    case SGObjFunc::FIELD_COV:
      ptr = std::make_unique<obj::FieldCoverage>();
      break;
    case SGObjFunc::OVERLAPS:
      ptr = std::make_unique<obj::Overlaps>();
      break;
  }
  return ptr;
}

std::unique_ptr<autonomy::planning::planner::fields2cover::pp::TurningBase> GetPPTurningBasePtr(const Options& opt) {
  std::unique_ptr<autonomy::planning::planner::fields2cover::pp::TurningBase> turn_planner;
  switch (opt.pp_alg) {
    case PPAlg::DUBINS:
      turn_planner = std::make_unique<autonomy::planning::planner::fields2cover::pp::DubinsCurves>();
      break;
    case PPAlg::DUBINS_CC:
      turn_planner = std::make_unique<autonomy::planning::planner::fields2cover::pp::DubinsCurvesCC>();
      break;
    case PPAlg::REEDS_SHEPP:
      turn_planner = std::make_unique<autonomy::planning::planner::fields2cover::pp::ReedsSheppCurves>();
      break;
    case PPAlg::REEDS_SHEPP_HC:
      turn_planner = std::make_unique<autonomy::planning::planner::fields2cover::pp::ReedsSheppCurvesHC>();
      break;
  }
  return turn_planner;
}





F2CRoute PlanCovRoute(const F2CRobot& robot, const F2CCell& cell,
    const Options& opt) {
  return PlanCovRoute(robot, F2CCells(cell), opt);
}

F2CRoute PlanCovRoute(const F2CRobot& robot, const F2CCells& cells,
    const Options& opt) {
  F2CCells mainland;
  F2CCells fast_hl_rings;
  switch (opt.hg_alg) {
    case HGAlg::NONE:
      mainland = cells;
      fast_hl_rings = cells;
      break;
    case HGAlg::CONST: {
      autonomy::planning::planner::fields2cover::hg::ConstHL const_hl;
      mainland = const_hl.GenerateHeadlandArea(
          cells, robot.GetWidth(), opt.hg_swaths);
      auto hl_swaths = const_hl.GenerateHeadlandSwaths(
          cells, robot.GetWidth(), opt.hg_swaths);
      fast_hl_rings = hl_swaths[floor(0.5*opt.hg_swaths)];
      break;
    }
  }

  autonomy::planning::planner::fields2cover::sg::BruteForce bf;
  F2CSwathsByCells swaths;
  switch (opt.sg_alg) {
    case SGAlg::BRUTE_FORCE:
      swaths = bf.GenerateBestSwaths(
          *GetSGObjPtr(opt), robot.GetCovWidth(), mainland);
      break;
    case SGAlg::GIVEN_ANGLE:
      swaths = bf.GenerateSwaths(
          opt.sg_angle, robot.GetCovWidth(), mainland);
  }
  autonomy::planning::planner::fields2cover::rp::RoutePlannerBase rp;
  return rp.GenRoute(fast_hl_rings, swaths);
}

F2CRoute PlanCovRoute(const F2CRobot& robot, const F2CField& field,
    bool local_crs) {
  return PlanCovRoute(robot, field, Options(), local_crs);
}

F2CRoute PlanCovRoute(const F2CRobot& robot, const F2CField& field,
    const Options& opt, bool local_crs) {
  F2CField field_c = field.Clone();
  if (!local_crs) {
    autonomy::planning::planner::fields2cover::Transform::TransformToUTM(field_c);
  }
  F2CRoute route = PlanCovRoute(robot, field_c.GetField(), opt);
  return local_crs ? route : autonomy::planning::planner::fields2cover::Transform::TransformToPrevCRS(route, field_c);
}

F2CPath PlanCovPath(const F2CRobot& robot, const F2CCell& cell,
    const Options& opt) {
  return autonomy::planning::planner::fields2cover::pp::PathPlanning::PlanPath(
      robot, PlanCovRoute(robot, cell, opt), *GetPPTurningBasePtr(opt));
}

F2CPath PlanCovPath(const F2CRobot& robot, const F2CCells& cells,
    const Options& opt) {
  return autonomy::planning::planner::fields2cover::pp::PathPlanning::PlanPath(
      robot, PlanCovRoute(robot, cells, opt), *GetPPTurningBasePtr(opt));
}

F2CPath PlanCovPath(const F2CRobot& robot, const F2CField& field,
    bool local_crs) {
  return PlanCovPath(robot, field, Options(), local_crs);
}


F2CPath PlanCovPath(const F2CRobot& robot, const F2CField& field,
    const Options& opt, bool local_crs) {
  F2CField field_c = field.Clone();
  if (!local_crs) {
    autonomy::planning::planner::fields2cover::Transform::TransformToUTM(field_c);
  }
  F2CPath path = autonomy::planning::planner::fields2cover::pp::PathPlanning::PlanPath(
      robot, PlanCovRoute(robot, field_c.GetField(), opt),
      *GetPPTurningBasePtr(opt));
  return local_crs ? path : autonomy::planning::planner::fields2cover::Transform::TransformToPrevCRS(path, field_c);
}



}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy

