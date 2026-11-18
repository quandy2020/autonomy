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
 * @file robot.cpp
 * @brief Fields2Cover Robot implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <algorithm>
#include "autonomy/common/math/steering/utilities.hpp"
#include "autonomy/planning/planner/fields2cover/types/robot.hpp"

namespace autonomy::planning::planner::fields2cover::types {

Robot::Robot(double width, double cov_width,
    double max_curv, double max_diff_curv) :
      width_(width), cov_width_(cov_width),
      max_icc_(max_curv), linear_curv_change_(max_diff_curv) {
  if (width <= 0.0 || cov_width < 0.0) {
    throw std::out_of_range("Robot widths have to be greater than 0.");
  }
  if (cov_width == 0.0) {
    this->cov_width_ = this->width_;
  }
}

Robot::Robot() = default;
Robot::~Robot() = default;
Robot::Robot(const Robot&) = default;
Robot::Robot(Robot &&) = default;
Robot& Robot::operator=(const Robot&) = default;
Robot& Robot::operator=(Robot&&) = default;



std::string Robot::GetName() const {
  return this->name_;
}

void Robot::SetName(const std::string& str) {
  this->name_ = str;
}

double Robot::GetWidth() const {
  return this->width_;
}

void Robot::SetWidth(double w) {
  this->width_ = w;
}

double Robot::GetCovWidth() const {
  return this->cov_width_;
}

void Robot::SetCovWidth(double w) {
  this->cov_width_ = w;
}

double Robot::GetCruiseVel() const {
  return this->cruise_speed_;
}

void Robot::SetCruiseVel(double v) {
  this->cruise_speed_ = v;
}

double Robot::GetTurnVel() const {
  return this->turn_vel_ ? *this->turn_vel_ : this->cruise_speed_;
}

void Robot::SetTurnVel(double v) {
  this->turn_vel_ = v;
}

double Robot::GetMinTurningRadius() const {
  return 1. / (this->max_icc_ + 1e-7);
}

void Robot::SetMinTurningRadius(double rad) {
  this->max_icc_ = 1.0 / (fabs(rad) + 1e-7);
}

double Robot::GetMaxCurv() const {
  return this->max_icc_;
}

void Robot::SetMaxCurv(double c) {
  this->max_icc_ = fabs(c);
}

double Robot::GetMaxDiffCurv() const {
  return this->linear_curv_change_;
}

void Robot::SetMaxDiffCurv(double dc) {
  this->linear_curv_change_ = fabs(dc);
}

double Robot::GetSmoothTurningRadius() const {
  double x, y, ang, k;
  autonomy::common::math::steering::EndOfClothoid(
      0.0, 0.0, 0.0, 0.0, this->GetMaxDiffCurv(), 1.0,
      this->GetMaxCurv() / this->GetMaxDiffCurv(),
      &x, &y, &ang, &k);
  double xi = x - sin(ang) / this->GetMaxCurv();
  double yi = y + cos(ang) / this->GetMaxCurv();
  return sqrt(xi * xi + yi * yi);
}

double Robot::GetTurnRadius(double sweep, bool continuous) const {
  const double min_radius = this->GetMinTurningRadius();
  const double rate = this->GetMaxDiffCurv();
  if (!continuous || rate <= 0.0) {
    return min_radius;
  }
  double radius = std::max(min_radius, this->GetSmoothTurningRadius());
  if (sweep > 0.0) {
    radius = std::max(radius, 1.0 / sqrt(rate * sweep));
  }
  return radius;
}

double Robot::GetMaxCornerCut() const {
  if (this->max_corner_cut_) {
    return *this->max_corner_cut_;
  }
  return std::max(
      std::min(this->GetMinTurningRadius(), this->cov_width_),
      0.5 * this->cov_width_);
}

void Robot::SetMaxCornerCut(double d) {
  this->max_corner_cut_ = fabs(d);
}

double Robot::GetDirectHopMaxDev() const {
  return this->direct_hop_max_dev_ ?
      *this->direct_hop_max_dev_ : 0.25 * this->cov_width_;
}

void Robot::SetDirectHopMaxDev(double d) {
  this->direct_hop_max_dev_ = fabs(d);
}

double Robot::GetUturnMaxHop() const {
  return this->uturn_max_hop_ ?
      *this->uturn_max_hop_ : 3.0 * this->cov_width_;
}

void Robot::SetUturnMaxHop(double d) {
  this->uturn_max_hop_ = fabs(d);
}

double Robot::GetTrackSimplifyTol() const {
  return this->track_simplify_tol_ ?
      *this->track_simplify_tol_ : 0.1 * this->cov_width_;
}

void Robot::SetTrackSimplifyTol(double d) {
  this->track_simplify_tol_ = fabs(d);
}

double Robot::GetReversalSweep() const {
  return this->reversal_sweep_;
}

void Robot::SetReversalSweep(double a) {
  this->reversal_sweep_ = fabs(a);
}

double Robot::GetTurnSlack() const {
  return this->turn_slack_;
}

void Robot::SetTurnSlack(double a) {
  this->turn_slack_ = fabs(a);
}

double Robot::GetMinSweep() const {
  return this->min_sweep_;
}

void Robot::SetMinSweep(double a) {
  this->min_sweep_ = fabs(a);
}

double Robot::GetLegShare() const {
  return this->leg_share_;
}

void Robot::SetLegShare(double s) {
  this->leg_share_ = fabs(s);
}

double Robot::GetRadiusMargin() const {
  return this->radius_margin_;
}

void Robot::SetRadiusMargin(double m) {
  this->radius_margin_ = fabs(m);
}

double Robot::GetMinBackoffRadii() const {
  return this->min_backoff_radii_;
}

void Robot::SetMinBackoffRadii(double r) {
  this->min_backoff_radii_ = fabs(r);
}

double Robot::GetUturnReachRadii() const {
  return this->uturn_reach_radii_;
}

void Robot::SetUturnReachRadii(double r) {
  this->uturn_reach_radii_ = fabs(r);
}

size_t Robot::GetMaxCornerSpan() const {
  return this->max_corner_span_;
}

void Robot::SetMaxCornerSpan(size_t n) {
  this->max_corner_span_ = n;
}

}  // namespace autonomy::planning::planner::fields2cover::types

