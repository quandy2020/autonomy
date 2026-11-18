/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file dubins_zero_max.cpp
 * @brief Continuous-curvature Dubins from zero to max curvature (implementation).
 */

#include <cmath>
#include <limits>

#include "autonomy/common/math/steering/clothoid/dubins_zero_max.hpp"
#include "autonomy/common/math/steering/clothoid/configuration.hpp"
#include "autonomy/common/math/steering/utilities.hpp"
#include <algorithm>

using namespace std;

namespace autonomy {
namespace common {
namespace math {
namespace steering {


class Cc0pmDubinsStateSpace::Impl
{
private:
  Cc0pmDubinsStateSpace *parent_;

public:
  explicit Impl(Cc0pmDubinsStateSpace *parent)
  {
    parent_ = parent;
  }

  double distance = 0.0;
  double angle = 0.0;

  // ##### TT ###################################################################
  bool TTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return fabs(distance - 2 * c1.radius) < GetEpsilon();
  }

  void TT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q) const
  {
    double x = (c1.xc + c2.xc) / 2;
    double y = (c1.yc + c2.yc) / 2;
    double angle = atan2(c2.yc - c1.yc, c2.xc - c1.xc);
    double theta;
    if (c1.left)
    {
      if (c1.forward)
      {
        theta = angle + M_PI_2 - c1.mu;
      }
      else
      {
        theta = angle + M_PI_2 + c1.mu;
      }
    }
    else
    {
      if (c1.forward)
      {
        theta = angle - M_PI_2 + c1.mu;
      }
      else
      {
        theta = angle - M_PI_2 - c1.mu;
      }
    }
    *q = new Configuration(x, y, theta, 0);
  }

  double TTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                 Configuration **q1, Configuration **q2) const
  {
    TT_tangent_circles(c1, c2, q1);
    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(**q1, c2.left, !c2.forward, true, parent_->hc_cc_circle_param_);
    *q2 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);
    return (*cstart)->CcTurnLength(**q1) + (*cend)->HcTurnLength(**q2);
  }

  // ##### Dubins families: #####################################################

  // ##### TST ##################################################################
  bool TiSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return (distance >= 2 * c1.radius);
  }

  bool TeSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return (distance >= 2 * c1.radius * c1.sin_mu);
  }

  bool TSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TiSTExists(c1, c2) || TeSTExists(c1, c2);
  }

  void TiST_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1,
                            Configuration **q2) const
  {
    double distance = CenterDistance(c1, c2);
    double angle = atan2(c2.yc - c1.yc, c2.xc - c1.xc);
    double alpha = asin(2 * c1.radius * c1.cos_mu / distance);
    double delta_x = c1.radius * c1.sin_mu;
    double delta_y = c1.radius * c1.cos_mu;
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
  }

  void TeST_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1,
                            Configuration **q2) const
  {
    double delta_x = c1.radius * c1.sin_mu;
    double delta_y = c1.radius * c1.cos_mu;
    double theta = atan2(c2.yc - c1.yc, c2.xc - c1.xc);
    double x, y;
    if (c1.left && c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (c1.left && !c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (!c1.left && c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && !c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
  }

  double TiSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                   Configuration **q1, Configuration **q2, Configuration **q3) const
  {
    TiST_tangent_circles(c1, c2, q1, q2);
    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(**q2, c2.left, !c2.forward, true, parent_->hc_cc_circle_param_);
    *q3 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);
    return (*cstart)->CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->HcTurnLength(**q3);
  }

  double TeSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                   Configuration **q1, Configuration **q2, Configuration **q3) const
  {
    TeST_tangent_circles(c1, c2, q1, q2);
    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(**q2, c2.left, !c2.forward, true, parent_->hc_cc_circle_param_);
    *q3 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);
    return (*cstart)->CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->HcTurnLength(**q3);
  }

  double TSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                  Configuration **q1, Configuration **q2, Configuration **q3) const
  {
    if (TiSTExists(c1, c2))
    {
      return TiSTPath(c1, c2, cstart, cend, q1, q2, q3);
    }
    if (TeSTExists(c1, c2))
    {
      return TeSTPath(c1, c2, cstart, cend, q1, q2, q3);
    }
    return numeric_limits<double>::max();
  }

  // ##### TTT ##################################################################
  bool TTTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return distance <= 4 * c1.radius;
  }

  void TTT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                           Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r = 2 * c1.radius;
    double delta_x = 0.5 * distance;
    double delta_y = sqrt(pow(r, 2) - pow(delta_x, 2));
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    HcCcCircle tgt2(x, y, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    TT_tangent_circles(c1, tgt1, q1);
    TT_tangent_circles(tgt1, c2, q2);
    TT_tangent_circles(c1, tgt2, q3);
    TT_tangent_circles(tgt2, c2, q4);
  }

  double TTTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                  Configuration **q1, Configuration **q2, Configuration **q3, HcCcCircle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TTT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    HcCcCircle *end1, *end2, *middle1, *middle2;
    middle1 = new HcCcCircle(*qa, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    end1 = new HcCcCircle(*qb, c2.left, !c2.forward, true, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qc, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    end2 = new HcCcCircle(*qd, c2.left, !c2.forward, true, parent_->hc_cc_circle_param_);

    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    *q3 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);

    // select shortest connection
    double length1 = (*cstart)->CcTurnLength(*qa) + middle1->CcTurnLength(*qb) + end1->HcTurnLength(**q3);
    double length2 = (*cstart)->CcTurnLength(*qc) + middle2->CcTurnLength(*qd) + end2->HcTurnLength(**q3);
    if (length1 < length2)
    {
      *cend = end1;
      *q1 = qa;
      *q2 = qb;
      *ci = middle1;
      delete qc;
      delete qd;
      delete middle2;
      delete end2;
      return length1;
    }
    else
    {
      *cend = end2;
      *q1 = qc;
      *q2 = qd;
      *ci = middle2;
      delete qa;
      delete qb;
      delete middle1;
      delete end1;
      return length2;
    }
    return numeric_limits<double>::max();
  }
};

// ############################################################################

Cc0pmDubinsStateSpace::Cc0pmDubinsStateSpace(double kappa, double sigma, double discretization, bool forwards)
  : HcCcStateSpace(kappa, sigma, discretization)
  , forwards_(forwards)
  , cc0pm_dubins_{ unique_ptr<Impl>(new Impl(this)) }
{
  rs_circle_param_.SetParam(kappa_, numeric_limits<double>::max(), 1 / kappa_, 0.0, 0.0, 1.0, 0.0);
  radius_ = hc_cc_circle_param_.radius;
  mu_ = hc_cc_circle_param_.mu;
}

Cc0pmDubinsStateSpace::~Cc0pmDubinsStateSpace() = default;

CcDubinsPath *Cc0pmDubinsStateSpace::Cc0pmCirclesDubinsPath(const HcCcCircle &c1,
                                                                    const HcCcCircle &c2) const
{
  // table containing the lengths of the paths, the intermediate configurations and circles
  double length[kNbCcDubinsPaths];
  std::fill(length, (length) + (kNbCcDubinsPaths), numeric_limits<double>::max());
  Configuration *qi1[kNbCcDubinsPaths];
  std::fill(qi1, (qi1) + (kNbCcDubinsPaths), nullptr);
  Configuration *qi2[kNbCcDubinsPaths];
  std::fill(qi2, (qi2) + (kNbCcDubinsPaths), nullptr);
  Configuration *qi3[kNbCcDubinsPaths];
  std::fill(qi3, (qi3) + (kNbCcDubinsPaths), nullptr);
  HcCcCircle *cstart[kNbCcDubinsPaths];
  std::fill(cstart, (cstart) + (kNbCcDubinsPaths), nullptr);
  HcCcCircle *ci1[kNbCcDubinsPaths];
  std::fill(ci1, (ci1) + (kNbCcDubinsPaths), nullptr);
  HcCcCircle *cend[kNbCcDubinsPaths];
  std::fill(cend, (cend) + (kNbCcDubinsPaths), nullptr);

  // precomputations
  cc0pm_dubins_->distance = CenterDistance(c1, c2);
  cc0pm_dubins_->angle = atan2(c2.yc - c1.yc, c2.xc - c1.xc);

  // case E
  if (ConfigurationEqual(c1.start, c2.start))
  {
    length[cc_dubins::E] = 0;
    goto label_end;
  }
  // case T
  if (cc0pm_dubins_->distance < GetEpsilon())
  {
    cstart[cc_dubins::T] = new HcCcCircle(c1.start, c1.left, c1.forward, true, hc_cc_circle_param_);
    length[cc_dubins::T] = cstart[cc_dubins::T]->HcTurnLength(c2.start);
    goto label_end;
  }
  // case TT
  if (cc0pm_dubins_->TTExists(c1, c2))
  {
    length[cc_dubins::TT] = cc0pm_dubins_->TTPath(c1, c2, &cstart[cc_dubins::TT], &cend[cc_dubins::TT],
                                                   &qi1[cc_dubins::TT], &qi2[cc_dubins::TT]);
  }
  // ##### Dubins families: #####################################################
  // case TST
  if (cc0pm_dubins_->TSTExists(c1, c2))
  {
    length[cc_dubins::TST] = cc0pm_dubins_->TSTPath(c1, c2, &cstart[cc_dubins::TST], &cend[cc_dubins::TST],
                                                     &qi1[cc_dubins::TST], &qi2[cc_dubins::TST], &qi3[cc_dubins::TST]);
  }
  // case TTT
  if (cc0pm_dubins_->TTTExists(c1, c2))
  {
    length[cc_dubins::TTT] =
        cc0pm_dubins_->TTTPath(c1, c2, &cstart[cc_dubins::TTT], &cend[cc_dubins::TTT], &qi1[cc_dubins::TTT],
                                &qi2[cc_dubins::TTT], &qi3[cc_dubins::TTT], &ci1[cc_dubins::TTT]);
  }
label_end:
  // select shortest path
  cc_dubins::path_type best_path = (cc_dubins::path_type)static_cast<int>(std::min_element(length, (length) + (kNbCcDubinsPaths)) - (length));
  CcDubinsPath *path;
  path =
      new CcDubinsPath(c1.start, c2.start, best_path, kappa_, sigma_, qi1[best_path], qi2[best_path], qi3[best_path],
                         nullptr, cstart[best_path], cend[best_path], ci1[best_path], nullptr, length[best_path]);

  // clean up
  for (int i = 0; i < kNbCcDubinsPaths; i++)
  {
    if (i != best_path)
    {
      delete qi1[i];
      delete qi2[i];
      delete qi3[i];
      delete cstart[i];
      delete ci1[i];
      delete cend[i];
    }
  }
  return path;
}

CcDubinsPath *Cc0pmDubinsStateSpace::Cc0pmDubins(const State &state1, const State &state2) const
{
  // compute the 2 circles at the intial and final configuration
  Configuration start(state1.x, state1.y, state1.theta, 0.0);
  Configuration end1(state2.x, state2.y, state2.theta, kappa_);
  Configuration end2(state2.x, state2.y, state2.theta, -kappa_);

  HcCcCircle *start_circle[2];
  HcCcCircle *end_circle[2];
  if (forwards_)
  {
    start_circle[0] = new HcCcCircle(start, true, true, true, hc_cc_circle_param_);
    start_circle[1] = new HcCcCircle(start, false, true, true, hc_cc_circle_param_);
    end_circle[0] = new HcCcCircle(end1, true, false, true, rs_circle_param_);
    end_circle[1] = new HcCcCircle(end2, false, false, true, rs_circle_param_);
  }
  else
  {
    start_circle[0] = new HcCcCircle(start, true, false, true, hc_cc_circle_param_);
    start_circle[1] = new HcCcCircle(start, false, false, true, hc_cc_circle_param_);
    end_circle[0] = new HcCcCircle(end1, true, true, true, rs_circle_param_);
    end_circle[1] = new HcCcCircle(end2, false, true, true, rs_circle_param_);
  }

  // compute the shortest path for the 4 combinations (2 circles at the beginning and 2 at the end)
  CcDubinsPath *path[] = { nullptr, nullptr, nullptr, nullptr };

  double lg[] = { numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                  numeric_limits<double>::max() };

  for (int i = 0; i < 2; i++)
  {
    for (int j = 0; j < 2; j++)
    {
      // skip circle at the end for curvature continuity
      if (j == 0 && state2.kappa < 0)
        continue;
      else if (j == 1 && state2.kappa > 0)
        continue;
      path[2 * i + j] = Cc0pmCirclesDubinsPath(*start_circle[i], *end_circle[j]);
      if (path[2 * i + j])
      {
        lg[2 * i + j] = path[2 * i + j]->length;
      }
    }
  }

  // select shortest path
  int best_path = static_cast<int>(std::min_element(lg, (lg) + (4)) - (lg));

  //  // display calculations
  //  cout << endl << "Cc0pmDubinsStateSpace" << endl;
  //  for (int i = 0; i < 4; i++)
  //  {
  //    cout << i << ": ";
  //    if (path[i])
  //    {
  //      path[i]->Print(true);
  //      cout << endl;
  //    }
  //  }
  //  cout << "shortest path: " << (int)best_path << endl;
  //  path[best_path]->Print(true);

  // clean up
  for (int i = 0; i < 2; i++)
  {
    delete start_circle[i];
    delete end_circle[i];
  }
  for (int i = 0; i < 4; i++)
  {
    if (i != best_path)
    {
      delete path[i];
    }
  }
  return path[best_path];
}

double Cc0pmDubinsStateSpace::GetDistance(const State &state1, const State &state2) const
{
  CcDubinsPath *p = this->Cc0pmDubins(state1, state2);
  double length = p->length;
  delete p;
  return length;
}

vector<Control> Cc0pmDubinsStateSpace::GetControls(const State &state1, const State &state2) const
{
  vector<Control> cc_dubins_controls;
  cc_dubins_controls.reserve(8);
  CcDubinsPath *p = this->Cc0pmDubins(state1, state2);
  switch (p->type)
  {
    case cc_dubins::E:
      EmptyControls(cc_dubins_controls);
      break;
    case cc_dubins::T:
      HcTurnControls(*(p->cstart), p->end, true, cc_dubins_controls);
      break;
    case cc_dubins::TT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_dubins_controls);
      HcTurnControls(*(p->cend), *(p->qi2), true, cc_dubins_controls);
      break;
    // ##### Dubins families: #####################################################
    case cc_dubins::TST:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_dubins_controls);
      StraightControls(*(p->qi1), *(p->qi2), cc_dubins_controls);
      HcTurnControls(*(p->cend), *(p->qi3), true, cc_dubins_controls);
      break;
    case cc_dubins::TTT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_dubins_controls);
      CcTurnControls(*(p->ci1), *(p->qi2), true, cc_dubins_controls);
      HcTurnControls(*(p->cend), *(p->qi3), true, cc_dubins_controls);
      break;
    default:
      break;
  }
  delete p;
  return cc_dubins_controls;
}

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
