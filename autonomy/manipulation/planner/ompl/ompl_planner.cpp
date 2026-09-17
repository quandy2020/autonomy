/*
 * Copyright 2026 The Openbot Authors
 *
 * OMPL joint-space planner (RRTConnect / RRT / PRM / RRTstar / KPIECE) +
 * path constraints, goal tolerance, dense validity.
 */

#include "autonomy/manipulation/planner/ompl/ompl_planner.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/multiplan/ParallelPlan.h>

#include <Eigen/Core>

#include "autolink/plugin_manager/plugin_manager.hpp"
#include "autonomy/common/conf_loader.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/manipulation/planner/constraint_samplers/constraint_sampler_manager.hpp"
#include "autonomy/manipulation/planner/constraint_samplers/constraint_samplers.hpp"
#include "autonomy/manipulation/planner/ompl/ompl_planning_config.hpp"
#include "autonomy/manipulation/common/joint_state_util.hpp"
#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {
namespace {

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ot = ompl::tools;

double GetJointLowerBound(const MotionPlanRequest& request, const std::string& name) {
  if (request.model) {
    if (const auto* lim = request.model->GetJointLimits(name)) {
      if (!lim->has_position_limits) {
        return -1e6;  // continuous
      }
      return lim->min_position;
    }
    for (const auto& jm : request.model->Joints()) {
      if (jm.name == name &&
          (jm.type == "continuous" || !jm.limits.has_position_limits)) {
        return -1e6;
      }
    }
  }
  return -3.141592653589793;
}

double GetJointUpperBound(const MotionPlanRequest& request, const std::string& name) {
  if (request.model) {
    if (const auto* lim = request.model->GetJointLimits(name)) {
      if (!lim->has_position_limits) {
        return 1e6;
      }
      return lim->max_position;
    }
    for (const auto& jm : request.model->Joints()) {
      if (jm.name == name &&
          (jm.type == "continuous" || !jm.limits.has_position_limits)) {
        return 1e6;
      }
    }
  }
  return 3.141592653589793;
}

std::string NormalizePlannerType(const std::string& id) {
  std::string s = id;
  for (char& c : s) {
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
  }
  if (s.find("prm") != std::string::npos) {
    return "PRM";
  }
  if (s.find("rrtstar") != std::string::npos || s.find("rrt*") != std::string::npos) {
    return "RRTstar";
  }
  if (s.find("bitrrt") != std::string::npos) {
    return "BiTRRT";
  }
  if (s.find("kpiece") != std::string::npos) {
    return "KPIECE";
  }
  if (s.find("est") != std::string::npos && s.find("rrt") == std::string::npos) {
    return "EST";
  }
  if (s.find("rrtconnect") != std::string::npos ||
      s.find("rrt_connect") != std::string::npos || s == "ompl") {
    return "RRTConnect";
  }
  if (s.find("rrt") != std::string::npos) {
    return "RRT";
  }
  return "RRTConnect";
}

core::JointState StateToJoint(const ob::State* state,
                              const std::vector<std::string>& names,
                              std::size_t dof, bool constrained = false) {
  const double* vals = nullptr;
  if (constrained) {
    const auto* cst = state->as<ob::ConstrainedStateSpace::StateType>();
    vals = cst->getState()->as<ob::RealVectorStateSpace::StateType>()->values;
  } else {
    vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
  }
  std::vector<double> positions(dof);
  for (std::size_t i = 0; i < dof; ++i) {
    positions[i] = vals[i];
  }
  core::JointState js;
  SetJointState(&js, names, positions);
  return js;
}

bool StateValid(const MotionPlanRequest& request, const core::JointState& js) {
  if (!constraint_samplers::SatisfiesJointConstraints(request, js)) {
    return false;
  }
  if (request.kinematics &&
      (!request.position_constraints.empty() ||
       !request.orientation_constraints.empty())) {
    kinematics::Pose tip;
    if (request.kinematics->GetPositionFK(js, &tip)) {
      for (const auto& c : request.position_constraints) {
        if (!constraint_samplers::SatisfiesPositionConstraint(c, tip)) {
          return false;
        }
      }
      for (const auto& c : request.orientation_constraints) {
        if (!constraint_samplers::SatisfiesOrientationConstraint(c, tip)) {
          return false;
        }
      }
    } else if (!request.position_constraints.empty() ||
               !request.orientation_constraints.empty()) {
      return false;
    }
  }
  if (request.scene) {
    return request.scene->IsStateValid(js);
  }
  return true;
}

bool HasCartesianConstraints(const MotionPlanRequest& request) {
  return request.kinematics &&
         (!request.position_constraints.empty() ||
          !request.orientation_constraints.empty());
}

unsigned ConstraintManifoldDim(const MotionPlanRequest& request) {
  unsigned m = 0;
  m += 3u * static_cast<unsigned>(request.position_constraints.size());
  // Orientation residual as 3-vector (scaled angle-axis lite).
  m += 3u * static_cast<unsigned>(request.orientation_constraints.size());
  return std::max(1u, m);
}

/**
 * OMPL Constraint: tip pose residual (MoveIt ProjectedStateSpace manifold lite).
 */
class TipPoseConstraint : public ob::Constraint {
 public:
  TipPoseConstraint(const MotionPlanRequest* request,
                    const std::vector<std::string>* names, unsigned dof)
      : ob::Constraint(dof, ConstraintManifoldDim(*request)),
        request_(request),
        names_(names),
        dof_(dof) {}

  void function(const Eigen::Ref<const Eigen::VectorXd>& x,
                Eigen::Ref<Eigen::VectorXd> out) const override {
    out.setZero();
    if (!request_ || !request_->kinematics || !names_) {
      return;
    }
    std::vector<double> positions(dof_);
    for (unsigned i = 0; i < dof_; ++i) {
      positions[i] = x[static_cast<Eigen::Index>(i)];
    }
    core::JointState js;
    SetJointState(&js, *names_, positions);
    kinematics::Pose tip;
    if (!request_->kinematics->GetPositionFK(js, &tip)) {
      out.setConstant(1.0);
      return;
    }
    Eigen::Index row = 0;
    for (const auto& c : request_->position_constraints) {
      const double tol = std::max(1e-6, c.tolerance);
      out[row++] = (tip.position().x() - c.target().pose().position().x()) / tol;
      out[row++] = (tip.position().y() - c.target().pose().position().y()) / tol;
      out[row++] = (tip.position().z() - c.target().pose().position().z()) / tol;
    }
    for (const auto& c : request_->orientation_constraints) {
      const double tol = std::max(1e-6, c.tolerance);
      const double dot =
          std::abs(tip.orientation().w() * c.target().pose().orientation().w() +
                   tip.orientation().x() * c.target().pose().orientation().x() +
                   tip.orientation().y() * c.target().pose().orientation().y() +
                   tip.orientation().z() * c.target().pose().orientation().z());
      const double angle =
          2.0 * std::acos(std::min(1.0, std::max(0.0, dot)));
      // Spread scalar angle into 3 equal residuals for manifold dim.
      const double r = angle / tol / std::sqrt(3.0);
      out[row++] = r;
      out[row++] = r;
      out[row++] = r;
    }
  }

 private:
  const MotionPlanRequest* request_;
  const std::vector<std::string>* names_;
  unsigned dof_;
};

void CopyJointsIntoState(ob::State* state, const core::JointState& js,
                         std::size_t dof, bool constrained) {
  double* vals = nullptr;
  if (constrained) {
    vals = state->as<ob::ConstrainedStateSpace::StateType>()
               ->getState()
               ->as<ob::RealVectorStateSpace::StateType>()
               ->values;
  } else {
    vals = state->as<ob::RealVectorStateSpace::StateType>()->values;
  }
  for (std::size_t i = 0; i < dof; ++i) {
    vals[i] = static_cast<int>(i) < js.position_size() ? js.position(static_cast<int>(i)) : 0.0;
  }
}

/**
 * OMPL sampler that draws from joint / IK constraint regions when present
 * (MoveIt JointConstraintSampler + IKConstraintSampler lite), else uniform
 * + Cartesian projection.
 */
class CartesianProjectingSampler : public ob::StateSampler {
 public:
  CartesianProjectingSampler(const ob::StateSpace* space,
                             const MotionPlanRequest* request,
                             const std::vector<std::string>* names,
                             std::size_t dof, bool constrained)
      : ob::StateSampler(space),
        request_(request),
        names_(names),
        dof_(dof),
        constrained_(constrained),
        default_(space->allocDefaultStateSampler()),
        rng_(42) {}

  void sampleUniform(ob::State* state) override {
    if (TryConstrainedSample(state)) {
      return;
    }
    default_->sampleUniform(state);
    Project(state);
  }

  void sampleUniformNear(ob::State* state, const ob::State* near,
                         double distance) override {
    default_->sampleUniformNear(state, near, distance);
    Project(state);
  }

  void sampleGaussian(ob::State* state, const ob::State* mean,
                      double stdDev) override {
    default_->sampleGaussian(state, mean, stdDev);
    Project(state);
  }

 private:
  bool TryConstrainedSample(ob::State* state) {
    const bool has_cart = HasCartesianConstraints(*request_);
    const bool has_joint = !request_->joint_constraints.empty();
    if (!has_cart && !has_joint) {
      return false;
    }
    core::JointState seed;
    SetJointState(&seed, *names_, std::vector<double>(dof_, 0.0));
    if (request_->start_state.position_size() > 0 &&
        static_cast<std::size_t>(request_->start_state.position_size()) ==
            dof_) {
      seed = request_->start_state;
      seed.clear_name();
      for (const auto& n : *names_) {
        seed.add_name(n);
      }
    }
    core::JointState js;
    if (has_cart) {
      if (!constraint_samplers::SampleIkConstrainedState(*request_, seed, &js,
                                                         &rng_, 8)) {
        return false;
      }
    } else {
      js = seed;
      if (!constraint_samplers::SampleJointConstrainedState(*request_, &js,
                                                            &rng_)) {
        return false;
      }
    }
    auto* rv_unused = state;  // silence — write via helper
    (void)rv_unused;
    CopyJointsIntoState(state, js, dof_, constrained_);
    return true;
  }

  void Project(ob::State* state) {
    if (!HasCartesianConstraints(*request_)) {
      return;
    }
    core::JointState js = StateToJoint(state, *names_, dof_, constrained_);
    if (!constraint_samplers::ProjectOntoCartesianConstraints(*request_,
                                                              &js)) {
      return;
    }
    CopyJointsIntoState(state, js, dof_, constrained_);
  }

  const MotionPlanRequest* request_;
  const std::vector<std::string>* names_;
  std::size_t dof_;
  bool constrained_ = false;
  ob::StateSamplerPtr default_;
  std::mt19937 rng_;
};

/**
 * Goal region that samples constrained goals (MoveIt goal sampling lite).
 * Used when joint/Cartesian path constraints exist so planners can aim at
 * any valid goal in the constrained set, not only the nominal goal_state.
 */
class ConstrainedGoalRegion : public ob::GoalSampleableRegion {
 public:
  ConstrainedGoalRegion(const ob::SpaceInformationPtr& si,
                        const MotionPlanRequest* request,
                        const std::vector<std::string>* names, std::size_t dof,
                        constraint_samplers::ConstraintSamplerManager* mgr,
                        bool constrained)
      : ob::GoalSampleableRegion(si),
        request_(request),
        names_(names),
        dof_(dof),
        mgr_(mgr),
        constrained_(constrained) {
    setThreshold(1e-3);
  }

  double distanceGoal(const ob::State* st) const override {
    if (!request_) {
      return 0.0;
    }
    const core::JointState js =
        StateToJoint(st, *names_, dof_, constrained_);
    if (StateValid(*request_, js)) {
      return 0.0;
    }
    double s = 0.0;
    for (std::size_t i = 0;
         i < dof_ && static_cast<int>(i) < request_->goal_state.position_size();
         ++i) {
      const double d = js.position(static_cast<int>(i)) -
                       request_->goal_state.position(static_cast<int>(i));
      s += d * d;
    }
    return std::sqrt(s);
  }

  bool isSatisfied(const ob::State* st, double* distance) const override {
    const double d = distanceGoal(st);
    if (distance) {
      *distance = d;
    }
    return d <= getThreshold();
  }

  void sampleGoal(ob::State* st) const override {
    if (!request_ || !mgr_) {
      CopyJointsIntoState(st, request_->goal_state, dof_, constrained_);
      return;
    }
    core::JointState sampled;
    if (mgr_->Sample(*request_, request_->goal_state, &sampled, 16) &&
        static_cast<std::size_t>(sampled.position_size()) == dof_) {
      CopyJointsIntoState(st, sampled, dof_, constrained_);
      return;
    }
    CopyJointsIntoState(st, request_->goal_state, dof_, constrained_);
  }

  unsigned int maxSampleCount() const override { return 64; }

 private:
  const MotionPlanRequest* request_;
  const std::vector<std::string>* names_;
  std::size_t dof_;
  constraint_samplers::ConstraintSamplerManager* mgr_;
  bool constrained_ = false;
};

}  // namespace

bool OmplPlanner::Init(const std::string& planner_id) {
  planner_id_ = planner_id.empty() ? "ompl" : planner_id;
  configs_.clear();
  by_name_.clear();
  std::string conf_path;
  if (common::ResolveModuleConfPath("manipulation", "ompl_planning.conf",
                                    &conf_path) &&
      LoadOmplPlannerConfigsFile(conf_path, &configs_)) {
    for (const auto& c : configs_) {
      by_name_[c.name] = c;
    }
    AINFO << "OmplPlanner loaded " << configs_.size() << " configs from "
          << conf_path;
  } else {
    configs_ = DefaultOmplPlannerConfigs();
    for (const auto& c : configs_) {
      by_name_[c.name] = c;
    }
  }
  AINFO << "OmplPlanner init id=" << planner_id_
        << " type=" << NormalizePlannerType(planner_id_);
  sampler_manager_.LoadExternalPluginDescriptions("");
  return true;
}

MotionPlanResponse OmplPlanner::Plan(const MotionPlanRequest& request_in) {
  MotionPlanRequest request = request_in;
  double goal_tol = request.goal_joint_tolerance > 0.0
                        ? request.goal_joint_tolerance
                        : 1e-3;
  double lvs_frac = 0.01;
  if (!request.planner_id.empty() && by_name_.count(request.planner_id)) {
    const auto& c = by_name_.at(request.planner_id);
    if (!c.planner_id.empty()) {
      planner_id_ = c.planner_id;
      request.planner_id = c.planner_id;
    }
    if (c.planning_time > 0) {
      request.planning_time = c.planning_time;
    }
    if (c.max_attempts > 0) {
      request.max_attempts = c.max_attempts;
    }
    if (c.goal_joint_tolerance > 0.0) {
      goal_tol = c.goal_joint_tolerance;
      request.goal_joint_tolerance = c.goal_joint_tolerance;
    }
    if (c.longest_valid_segment_fraction > 0.0) {
      lvs_frac = c.longest_valid_segment_fraction;
    }
  }
  if ((!request.position_constraints.empty() ||
       !request.orientation_constraints.empty() ||
       !request.joint_constraints.empty()) &&
      request.start_state.position_size() > 0) {
    core::JointState s = request.start_state;
    if (sampler_manager_.Project(request, &s)) {
      request.start_state = std::move(s);
    }
  }

  MotionPlanResponse response;
  if (request.start_state.position_size() == 0 ||
      request.goal_state.position_size() == 0 ||
      request.start_state.position_size() !=
          request.goal_state.position_size()) {
    response.error_code = ErrorCode::kInvalidRobotState;
    response.error = "OMPL: invalid start/goal";
    return response;
  }

  {
    std::string cerr;
    const ErrorCode ccode =
        constraint_samplers::EvaluateRequestConstraints(request, &cerr);
    if (ccode != ErrorCode::kSuccess) {
      response.error_code = ccode;
      response.error = cerr.empty() ? "OMPL constraint pre-check failed" : cerr;
      return response;
    }
  }

  if (!StateValid(request, request.start_state)) {
    response.error_code = ErrorCode::kStartStateInCollision;
    response.error = "OMPL: start invalid (collision/constraints)";
    return response;
  }
  if (!StateValid(request, request.goal_state)) {
    response.error_code = ErrorCode::kGoalInCollision;
    response.error = "OMPL: goal invalid (collision/constraints)";
    return response;
  }

  std::vector<std::string> names;
  if (request.goal_state.name_size() > 0) {
    names.assign(request.goal_state.name().begin(),
                 request.goal_state.name().end());
  } else {
    names.assign(request.start_state.name().begin(),
                 request.start_state.name().end());
  }
  const std::size_t dof =
      static_cast<std::size_t>(request.start_state.position_size());
  const bool use_css = HasCartesianConstraints(request);

  auto ambient =
      std::make_shared<ob::RealVectorStateSpace>(static_cast<unsigned>(dof));
  ob::RealVectorBounds bounds(static_cast<unsigned>(dof));
  for (std::size_t i = 0; i < dof; ++i) {
    const std::string n = i < names.size() ? names[i] : "";
    bounds.setLow(static_cast<unsigned>(i), GetJointLowerBound(request, n));
    bounds.setHigh(static_cast<unsigned>(i), GetJointUpperBound(request, n));
  }
  ambient->setBounds(bounds);

  ob::StateSpacePtr space = ambient;
  std::shared_ptr<TipPoseConstraint> tip_constraint;
  if (use_css) {
    tip_constraint = std::make_shared<TipPoseConstraint>(
        &request, &names, static_cast<unsigned>(dof));
    auto css =
        std::make_shared<ob::ProjectedStateSpace>(ambient, tip_constraint);
    css->setup();
    space = css;
    AINFO << "OmplPlanner using ProjectedStateSpace manifold_dim="
          << tip_constraint->getCoDimension();
  }

  if (HasCartesianConstraints(request) || !request.joint_constraints.empty()) {
    space->setStateSamplerAllocator(
        [&](const ob::StateSpace* ss) -> ob::StateSamplerPtr {
          return std::make_shared<CartesianProjectingSampler>(
              ss, &request, &names, dof, use_css);
        });
  }

  ob::SpaceInformationPtr si;
  if (use_css) {
    si = std::make_shared<ob::ConstrainedSpaceInformation>(space);
  } else {
    si = std::make_shared<ob::SpaceInformation>(space);
  }
  og::SimpleSetup setup(si);
  setup.setStateValidityChecker([&](const ob::State* state) {
    return StateValid(request, StateToJoint(state, names, dof, use_css));
  });
  setup.getSpaceInformation()->setStateValidityCheckingResolution(lvs_frac);

  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  CopyJointsIntoState(start.get(), request.start_state, dof, use_css);
  CopyJointsIntoState(goal.get(), request.goal_state, dof, use_css);
  if (use_css && tip_constraint) {
    Eigen::VectorXd xs(static_cast<Eigen::Index>(dof));
    Eigen::VectorXd xg(static_cast<Eigen::Index>(dof));
    for (std::size_t i = 0; i < dof; ++i) {
      xs[static_cast<Eigen::Index>(i)] = request.start_state.position(static_cast<int>(i));
      xg[static_cast<Eigen::Index>(i)] = request.goal_state.position(static_cast<int>(i));
    }
    tip_constraint->project(xs);
    tip_constraint->project(xg);
    CopyJointsIntoState(start.get(), [&] {
      core::JointState js = request.start_state;
      ResizeJointState(&js, static_cast<int>(dof));
      for (std::size_t i = 0; i < dof; ++i) {
        js.set_position(static_cast<int>(i), xs[static_cast<Eigen::Index>(i)]);
      }
      return js;
    }(), dof, use_css);
    CopyJointsIntoState(goal.get(), [&] {
      core::JointState js = request.goal_state;
      ResizeJointState(&js, static_cast<int>(dof));
      for (std::size_t i = 0; i < dof; ++i) {
        js.set_position(static_cast<int>(i), xg[static_cast<Eigen::Index>(i)]);
      }
      return js;
    }(), dof, use_css);
  }
  setup.setStartAndGoalStates(start, goal);

  const bool use_goal_region =
      HasCartesianConstraints(request) || !request.joint_constraints.empty();
  ob::GoalPtr goal_ptr;
  if (use_goal_region) {
    auto region = std::make_shared<ConstrainedGoalRegion>(
        setup.getSpaceInformation(), &request, &names, dof, &sampler_manager_,
        use_css);
    region->setThreshold(goal_tol);
    goal_ptr = region;
  } else {
    auto goal_state =
        std::make_shared<ob::GoalState>(setup.getSpaceInformation());
    goal_state->setState(goal);
    goal_state->setThreshold(goal_tol);
    goal_ptr = goal_state;
  }
  setup.setGoal(goal_ptr);

  const std::string type = NormalizePlannerType(planner_id_);
  auto si_plan = setup.getSpaceInformation();
  auto make_planner = [&](const std::string& t) -> ob::PlannerPtr {
    if (t == "PRM") {
      return std::make_shared<og::PRM>(si_plan);
    }
    if (t == "RRT") {
      return std::make_shared<og::RRT>(si_plan);
    }
    if (t == "RRTstar") {
      auto p = std::make_shared<og::RRTstar>(si_plan);
      auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(si_plan);
      setup.setOptimizationObjective(obj);
      return p;
    }
    if (t == "KPIECE") {
      return std::make_shared<og::KPIECE1>(si_plan);
    }
    if (t == "BiTRRT") {
      return std::make_shared<og::BiTRRT>(si_plan);
    }
    if (t == "EST") {
      return std::make_shared<og::EST>(si_plan);
    }
    return std::make_shared<og::RRTConnect>(si_plan);
  };

  const double timeout =
      request.planning_time > 0.0 ? request.planning_time : 1.0;
  const int attempts = std::max(1, request.max_attempts);
  // ParallelPlan + hybridize when multiple attempts (MoveIt ompl_interface lite).
  const bool use_parallel = attempts > 1 || type == "RRTConnect";
  ob::PlannerStatus status = ob::PlannerStatus::TIMEOUT;
  if (use_parallel) {
    ot::ParallelPlan pp(setup.getProblemDefinition());
    pp.addPlanner(make_planner(type));
    if (type != "RRTConnect") {
      pp.addPlanner(std::make_shared<og::RRTConnect>(si_plan));
    }
    if (type != "RRT") {
      pp.addPlanner(std::make_shared<og::RRT>(si_plan));
    }
    if (type != "KPIECE") {
      pp.addPlanner(std::make_shared<og::KPIECE1>(si_plan));
    }
    const std::size_t max_sol =
        static_cast<std::size_t>(std::min(4, std::max(2, attempts)));
    status = pp.solve(timeout, /*minSolCount=*/1, max_sol, /*hybridize=*/true);
    if (!status) {
      // Fallback: sequential retries with primary planner.
      auto planner = make_planner(type);
      setup.setPlanner(planner);
      for (int a = 0; a < attempts && !status; ++a) {
        setup.clear();
        setup.setStartState(start);
        setup.setGoal(goal_ptr);
        setup.setPlanner(planner);
        status = setup.solve(timeout);
      }
    }
  } else {
    auto planner = make_planner(type);
    setup.setPlanner(planner);
    for (int a = 0; a < attempts; ++a) {
      setup.clear();
      setup.setStartState(start);
      setup.setGoal(goal_ptr);
      setup.setPlanner(planner);
      status = setup.solve(timeout);
      if (status) {
        break;
      }
    }
  }
  if (!status) {
    response.error_code = ErrorCode::kPlanningFailed;
    response.error = "OMPL " + type + " failed after " +
                     std::to_string(attempts) + " attempts";
    return response;
  }

  setup.simplifySolution(timeout * 0.25);
  og::PathGeometric path = setup.getSolutionPath();
  const unsigned int n_interp = std::max(
      2u, static_cast<unsigned int>(std::max(path.getStateCount(), 10u) * 2u));
  path.interpolate(n_interp);

  response.trajectory.Clear();
  for (std::size_t i = 0; i < path.getStateCount(); ++i) {
    core::JointState wp =
        StateToJoint(path.getState(i), names, dof, use_css);
    if (HasCartesianConstraints(request)) {
      constraint_samplers::ProjectOntoCartesianConstraints(request, &wp);
    }
    AddTrajectoryPoint(&response.trajectory, wp, 0.05 * static_cast<double>(i));
  }

  if (!constraint_samplers::SatisfiesPathConstraints(request,
                                                     response.trajectory)) {
    response.success = false;
    response.error_code = ErrorCode::kGoalViolatesPathConstraints;
    response.error = "OMPL path violates constraints";
    response.trajectory = {};
    return response;
  }
  if (request.scene &&
      !request.scene->IsPathValidDense(response.trajectory, 4)) {
    response.success = false;
    response.error_code = ErrorCode::kInvalidMotionPlan;
    response.error = "OMPL path fails dense collision check";
    response.trajectory = {};
    return response;
  }

  response.success = true;
  response.error_code = ErrorCode::kSuccess;
  return response;
}

std::shared_ptr<PlannerBase> CreateOmplPlanner() {
  return std::make_shared<OmplPlanner>();
}

AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN(OmplPlanner, PlannerBase);

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
