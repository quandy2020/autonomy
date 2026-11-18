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
 * @file hybrid_max_max.cpp
 * @brief Hybrid-curvature Reeds–Shepp with max curvature at both ends (implementation).
 */

#include <cmath>
#include <limits>

#include "autonomy/common/math/steering/clothoid/hybrid_max_max.hpp"
#include "autonomy/common/math/steering/clothoid/configuration.hpp"
#include "autonomy/common/math/steering/utilities.hpp"
#include <algorithm>

constexpr bool kHcRegular = false;
constexpr bool kCcRegular = false;

using namespace std;

namespace autonomy {
namespace common {
namespace math {
namespace steering {


class HcpmpmReedsSheppStateSpace::Impl
{
private:
  HcpmpmReedsSheppStateSpace *parent_;

public:
  explicit Impl(HcpmpmReedsSheppStateSpace *parent)
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
    return fabs(distance - 2 * parent_->radius_) < GetEpsilon();
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
        theta = angle + M_PI_2 - parent_->mu_;
      }
      else
      {
        theta = angle + M_PI_2 + parent_->mu_;
      }
    }
    else
    {
      if (c1.forward)
      {
        theta = angle - M_PI_2 + parent_->mu_;
      }
      else
      {
        theta = angle - M_PI_2 - parent_->mu_;
      }
    }
    *q = new Configuration(x, y, theta, 0);
  }

  double TTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                 Configuration **q1, Configuration **q2, Configuration **q3) const
  {
    TT_tangent_circles(c1, c2, q2);
    *cstart = new HcCcCircle(**q2, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(**q2, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *q1 = new Configuration(c1.start.x, c1.start.y, c1.start.theta, c1.kappa);
    *q3 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);
    return (*cstart)->HcTurnLength(**q1) + (*cend)->HcTurnLength(**q3);
  }

  // ##### TcT ##################################################################
  bool TcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return fabs(distance - 2 * fabs(c1.kappa_inv)) < GetEpsilon();
  }

  void TcT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q) const
  {
    double distance = CenterDistance(c1, c2);
    double delta_x = 0.5 * distance;
    double delta_y = 0.0;
    double angle = atan2(c2.yc - c1.yc, c2.xc - c1.xc);
    double x, y, theta;
    if (c1.left)
    {
      if (c1.forward)
      {
        theta = angle + M_PI_2;
        GlobalFrameChange(c1.xc, c1.yc, angle, delta_x, delta_y, &x, &y);
      }
      else
      {
        theta = angle + M_PI_2;
        GlobalFrameChange(c1.xc, c1.yc, angle, delta_x, -delta_y, &x, &y);
      }
    }
    else
    {
      if (c1.forward)
      {
        theta = angle - M_PI_2;
        GlobalFrameChange(c1.xc, c1.yc, angle, delta_x, -delta_y, &x, &y);
      }
      else
      {
        theta = angle - M_PI_2;
        GlobalFrameChange(c1.xc, c1.yc, angle, delta_x, delta_y, &x, &y);
      }
    }
    *q = new Configuration(x, y, theta, c1.kappa);
  }

  double TcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                  Configuration **q) const
  {
    TcT_tangent_circles(c1, c2, q);
    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(c2);
    return (*cstart)->RsTurnLength(**q) + (*cend)->RsTurnLength(**q);
  }

  // ##### Reeds-Shepp families: ################################################

  // ##### TcTcT ################################################################
  bool TcTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return distance <= 4 * fabs(c1.kappa_inv);
  }

  void TcTcT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                             Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r = 2 * fabs(c1.kappa_inv);
    double delta_x = 0.5 * distance;
    double delta_y = sqrt(pow(r, 2) - pow(delta_x, 2));
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c1.left, !c1.forward, true, parent_->rs_circle_param_);
    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    HcCcCircle tgt2(x, y, !c1.left, !c1.forward, true, parent_->rs_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TcT_tangent_circles(tgt1, c2, q2);
    TcT_tangent_circles(c1, tgt2, q3);
    TcT_tangent_circles(tgt2, c2, q4);
  }

  double TcTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2, HcCcCircle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TcTcT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    HcCcCircle *middle1, *middle2;
    middle1 = new HcCcCircle(*qa, !c1.left, !c1.forward, true, parent_->rs_circle_param_);
    middle2 = new HcCcCircle(*qc, !c1.left, !c1.forward, true, parent_->rs_circle_param_);

    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(c2);

    // select shortest connection
    double length1 = (*cstart)->RsTurnLength(*qa) + middle1->RsTurnLength(*qb) + (*cend)->RsTurnLength(*qb);
    double length2 = (*cstart)->RsTurnLength(*qc) + middle2->RsTurnLength(*qd) + (*cend)->RsTurnLength(*qd);
    if (length1 < length2)
    {
      *q1 = qa;
      *q2 = qb;
      *ci = middle1;
      delete qc;
      delete qd;
      delete middle2;
      return length1;
    }
    else
    {
      *q1 = qc;
      *q2 = qd;
      *ci = middle2;
      delete qa;
      delete qb;
      delete middle1;
      return length2;
    }
    return numeric_limits<double>::max();
  }

  // ##### TcTT #################################################################
  bool TcTTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return (distance <= 2 * parent_->radius_ + 2 * fabs(c1.kappa_inv)) &&
           (distance >= 2 * parent_->radius_ - 2 * fabs(c1.kappa_inv));
  }

  void TcTT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                            Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r1 = 2 * fabs(c1.kappa_inv);
    double r2 = 2 * parent_->radius_;
    double delta_x = (pow(r1, 2) + pow(distance, 2) - pow(r2, 2)) / (2 * distance);
    double delta_y = sqrt(pow(r1, 2) - pow(delta_x, 2));
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    HcCcCircle tgt2(x, y, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TT_tangent_circles(tgt1, c2, q2);
    TcT_tangent_circles(c1, tgt2, q3);
    TT_tangent_circles(tgt2, c2, q4);
  }

  double TcTTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                   Configuration **q1, Configuration **q2, HcCcCircle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TcTT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    HcCcCircle *end1, *end2, *middle1, *middle2;
    end1 = new HcCcCircle(*qb, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);
    end2 = new HcCcCircle(*qd, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);
    middle1 = new HcCcCircle(*qb, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qd, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    *cstart = new HcCcCircle(c1);
    *q2 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);

    // select shortest connection
    double length1 = (*cstart)->RsTurnLength(*qa) + middle1->HcTurnLength(*qa) + end1->HcTurnLength(**q2);
    double length2 = (*cstart)->RsTurnLength(*qc) + middle2->HcTurnLength(*qc) + end2->HcTurnLength(**q2);
    if (length1 < length2)
    {
      *cend = end1;
      *q1 = qa;
      *ci = middle1;
      delete qb;
      delete qc;
      delete qd;
      delete end2;
      delete middle2;
      return length1;
    }
    else
    {
      *cend = end2;
      *q1 = qc;
      *ci = middle2;
      delete qa;
      delete qb;
      delete qd;
      delete end1;
      delete middle1;
      return length2;
    }
    return numeric_limits<double>::max();
  }

  // ##### TTcT #################################################################
  bool TTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return (distance <= 2 * parent_->radius_ + 2 * fabs(c1.kappa_inv)) &&
           (distance >= 2 * parent_->radius_ - 2 * fabs(c1.kappa_inv));
  }

  void TTcT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                            Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r1 = 2 * parent_->radius_;
    double r2 = 2 * fabs(c1.kappa_inv);
    double delta_x = (pow(r1, 2) + pow(distance, 2) - pow(r2, 2)) / (2 * distance);
    double delta_y = sqrt(pow(r1, 2) - pow(delta_x, 2));
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    HcCcCircle tgt2(x, y, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    TT_tangent_circles(c1, tgt1, q1);
    TcT_tangent_circles(tgt1, c2, q2);
    TT_tangent_circles(c1, tgt2, q3);
    TcT_tangent_circles(tgt2, c2, q4);
  }

  double TTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                   Configuration **q1, Configuration **q2, HcCcCircle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TTcT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    HcCcCircle *start1, *start2, *middle1, *middle2;
    start1 = new HcCcCircle(*qa, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    start2 = new HcCcCircle(*qc, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    middle1 = new HcCcCircle(*qa, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qc, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    *cend = new HcCcCircle(c2);
    *q1 = new Configuration(c1.start.x, c1.start.y, c1.start.theta, c1.kappa);

    // select shortest connection
    double length1 = start1->HcTurnLength(**q1) + middle1->HcTurnLength(*qb) + (*cend)->RsTurnLength(*qb);
    double length2 = start2->HcTurnLength(**q1) + middle2->HcTurnLength(*qd) + (*cend)->RsTurnLength(*qd);
    if (length1 < length2)
    {
      *cstart = start1;
      *q2 = qb;
      *ci = middle1;
      delete qa;
      delete qc;
      delete qd;
      delete start2;
      delete middle2;
      return length1;
    }
    else
    {
      *cstart = start2;
      *q2 = qd;
      *ci = middle2;
      delete qa;
      delete qb;
      delete qc;
      delete start1;
      delete middle1;
      return length2;
    }
    return numeric_limits<double>::max();
  }

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
    return distance >= 2 * parent_->radius_;
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
    return distance >= 2 * parent_->radius_ * parent_->sin_mu_;
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
    double alpha = asin(2 * parent_->radius_ * parent_->cos_mu_ / distance);
    double delta_x = parent_->radius_ * parent_->sin_mu_;
    double delta_y = parent_->radius_ * parent_->cos_mu_;
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
    double delta_x = parent_->radius_ * parent_->sin_mu_;
    double delta_y = parent_->radius_ * parent_->cos_mu_;
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
                   Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4) const
  {
    TiST_tangent_circles(c1, c2, q2, q3);
    *cstart = new HcCcCircle(**q2, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(**q3, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *q1 = new Configuration(c1.start.x, c1.start.y, c1.start.theta, c1.kappa);
    *q4 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);
    return (*cstart)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) + (*cend)->HcTurnLength(**q4);
  }

  double TeSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                   Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4) const
  {
    TeST_tangent_circles(c1, c2, q2, q3);
    *cstart = new HcCcCircle(**q2, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(**q3, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *q1 = new Configuration(c1.start.x, c1.start.y, c1.start.theta, c1.kappa);
    *q4 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);
    return (*cstart)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) + (*cend)->HcTurnLength(**q4);
  }

  double TSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                  Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4) const
  {
    if (TiSTExists(c1, c2))
    {
      return TiSTPath(c1, c2, cstart, cend, q1, q2, q3, q4);
    }
    if (TeSTExists(c1, c2))
    {
      return TeSTPath(c1, c2, cstart, cend, q1, q2, q3, q4);
    }
    return numeric_limits<double>::max();
  }

  // ##### TSTcT ################################################################
  bool TiSTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return distance >= sqrt(pow(2 * parent_->radius_ * parent_->sin_mu_ + 2 * fabs(c1.kappa_inv), 2) +
                            pow(2 * parent_->radius_ * parent_->cos_mu_, 2));
  }

  bool TeSTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return distance >= 2 * (fabs(c1.kappa_inv) + parent_->radius_ * parent_->sin_mu_);
  }

  bool TSTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TiSTcTExists(c1, c2) || TeSTcTExists(c1, c2);
  }

  double TiSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                     HcCcCircle **ci) const
  {
    double theta = angle;
    double delta_y = (4 * parent_->radius_ * parent_->cos_mu_) / (fabs(c2.kappa) * distance);
    double delta_x = sqrt(pow(2 * c2.kappa_inv, 2) - pow(delta_y, 2));
    double x, y;

    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TiST_tangent_circles(c1, tgt1, q2, q3);
    TcT_tangent_circles(tgt1, c2, q4);

    *cstart = new HcCcCircle(**q2, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2);
    *q1 = new Configuration(c1.start.x, c1.start.y, c1.start.theta, c1.kappa);
    *ci = new HcCcCircle(**q3, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    return (*cstart)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) + (*ci)->HcTurnLength(**q4) +
           (*cend)->RsTurnLength(**q4);
  }

  double TeSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                     HcCcCircle **ci) const
  {
    double theta = angle;
    double delta_x = 2 * fabs(c2.kappa_inv);
    double delta_y = 0;
    double x, y;

    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TeST_tangent_circles(c1, tgt1, q2, q3);
    TcT_tangent_circles(tgt1, c2, q4);

    *cstart = new HcCcCircle(**q2, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2);
    *q1 = new Configuration(c1.start.x, c1.start.y, c1.start.theta, c1.kappa);
    *ci = new HcCcCircle(**q3, c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    return (*cstart)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) + (*ci)->HcTurnLength(**q4) +
           (*cend)->RsTurnLength(**q4);
  }

  double TSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                    HcCcCircle **ci) const
  {
    if (TiSTcTExists(c1, c2))
    {
      return TiSTcTPath(c1, c2, cstart, cend, q1, q2, q3, q4, ci);
    }
    if (TeSTcTExists(c1, c2))
    {
      return TeSTcTPath(c1, c2, cstart, cend, q1, q2, q3, q4, ci);
    }
    return numeric_limits<double>::max();
  }

  // ##### TcTST ################################################################
  bool TcTiSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return (distance >= sqrt(pow(2 * parent_->radius_ * parent_->sin_mu_ + 2 * fabs(c1.kappa_inv), 2) +
                             pow(2 * parent_->radius_ * parent_->cos_mu_, 2)));
  }

  bool TcTeSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return (distance >= 2 * (fabs(c1.kappa_inv) + parent_->radius_ * parent_->sin_mu_));
  }

  bool TcTSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TcTiSTExists(c1, c2) || TcTeSTExists(c1, c2);
  }

  double TcTiSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                     HcCcCircle **ci) const
  {
    double theta = angle;
    double delta_y = (4 * parent_->radius_ * parent_->cos_mu_) / (fabs(c2.kappa) * distance);
    double delta_x = sqrt(pow(2 * c2.kappa_inv, 2) - pow(delta_y, 2));
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c2.left, !c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TiST_tangent_circles(tgt1, c2, q2, q3);

    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(**q3, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *q4 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);
    *ci = new HcCcCircle(**q2, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    return (*cstart)->RsTurnLength(**q1) + (*ci)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) +
           (*cend)->HcTurnLength(**q4);
  }

  double TcTeSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                     HcCcCircle **ci) const
  {
    double theta = angle;
    double delta_x = 2 * fabs(c2.kappa_inv);
    double delta_y = 0;
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, c2.left, !c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TeST_tangent_circles(tgt1, c2, q2, q3);

    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(**q3, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *q4 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);
    *ci = new HcCcCircle(**q2, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    return (*cstart)->RsTurnLength(**q1) + (*ci)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) +
           (*cend)->HcTurnLength(**q4);
  }

  double TcTSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                    HcCcCircle **ci) const
  {
    if (TcTiSTExists(c1, c2))
    {
      return TcTiSTPath(c1, c2, cstart, cend, q1, q2, q3, q4, ci);
    }
    if (TcTeSTExists(c1, c2))
    {
      return TcTeSTPath(c1, c2, cstart, cend, q1, q2, q3, q4, ci);
    }
    return numeric_limits<double>::max();
  }

  // ##### TcTSTcT ##############################################################
  bool TcTiSTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return (distance >= sqrt(pow(2 * parent_->radius_, 2) +
                             16 * parent_->radius_ * parent_->sin_mu_ * fabs(c1.kappa_inv) + pow(4 * c1.kappa_inv, 2)));
  }

  bool TcTeSTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return (distance >= 4 * fabs(c1.kappa_inv) + 2 * parent_->radius_ * parent_->sin_mu_);
  }

  bool TcTSTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TcTiSTcTExists(c1, c2) || TcTeSTcTExists(c1, c2);
  }

  double TcTiSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                       HcCcCircle **ci1, HcCcCircle **ci2) const
  {
    double theta = angle;
    double delta_y = (4 * parent_->radius_ * parent_->cos_mu_) / (distance * fabs(c1.kappa));
    double delta_x = sqrt(pow(2 * c1.kappa_inv, 2) - pow(delta_y, 2));
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
    HcCcCircle tgt2(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TiST_tangent_circles(tgt1, tgt2, q2, q3);
    TcT_tangent_circles(tgt2, c2, q4);

    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(c2);
    *ci1 = new HcCcCircle(**q2, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    *ci2 = new HcCcCircle(**q3, !c2.left, c2.forward, true, parent_->hc_cc_circle_param_);

    return (*cstart)->RsTurnLength(**q1) + (*ci1)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) +
           (*ci2)->HcTurnLength(**q4) + (*cend)->RsTurnLength(**q4);
  }

  double TcTeSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                       Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                       HcCcCircle **ci1, HcCcCircle **ci2) const
  {
    double theta = angle;
    double delta_x = 2 * fabs(c1.kappa_inv);
    double delta_y = 0;
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    HcCcCircle tgt2(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TeST_tangent_circles(tgt1, tgt2, q2, q3);
    TcT_tangent_circles(tgt2, c2, q4);

    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(c2);
    *ci1 = new HcCcCircle(**q2, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    *ci2 = new HcCcCircle(**q3, !c2.left, c2.forward, true, parent_->hc_cc_circle_param_);

    return (*cstart)->RsTurnLength(**q1) + (*ci1)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) +
           (*ci2)->HcTurnLength(**q4) + (*cend)->RsTurnLength(**q4);
  }

  double TcTSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                      Configuration **q1, Configuration **q2, Configuration **q3, Configuration **q4,
                      HcCcCircle **ci1, HcCcCircle **ci2) const
  {
    if (TcTiSTcTExists(c1, c2))
    {
      return TcTiSTcTPath(c1, c2, cstart, cend, q1, q2, q3, q4, ci1, ci2);
    }
    if (TcTeSTcTExists(c1, c2))
    {
      return TcTeSTcTPath(c1, c2, cstart, cend, q1, q2, q3, q4, ci1, ci2);
    }
    return numeric_limits<double>::max();
  }

  // ##### TTcTT ###############################################################
  bool TTcTTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return (distance <= 4 * parent_->radius_ + 2 * fabs(c1.kappa_inv));
  }

  void TTcTT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                             Configuration **q3, Configuration **q4, Configuration **q5, Configuration **q6) const
  {
    double theta = angle;
    double r1, r2, delta_x, delta_y, x, y;
    r1 = 2 * fabs(c1.kappa_inv);
    r2 = 2 * parent_->radius_;
    if (distance < 4 * parent_->radius_ - 2 * fabs(c1.kappa_inv))
    {
      delta_x = (distance + r1) / 2;
      delta_y = sqrt(pow(r2, 2) - pow(delta_x, 2));
    }
    else
    {
      delta_x = (distance - r1) / 2;
      delta_y = sqrt(pow(r2, 2) - pow(delta_x, 2));
    }

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    HcCcCircle tgt2(x, y, !c2.left, !c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    HcCcCircle tgt3(x, y, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
    HcCcCircle tgt4(x, y, !c2.left, !c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TT_tangent_circles(c1, tgt1, q1);
    TcT_tangent_circles(tgt1, tgt2, q2);
    TT_tangent_circles(tgt2, c2, q3);

    TT_tangent_circles(c1, tgt3, q4);
    TcT_tangent_circles(tgt3, tgt4, q5);
    TT_tangent_circles(tgt4, c2, q6);
  }

  double TTcTTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3, HcCcCircle **ci1,
                    HcCcCircle **ci2) const
  {
    Configuration *qa, *qb, *qc, *qd, *qe, *qf;
    TTcTT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd, &qe, &qf);
    HcCcCircle *start1, *start2, *end1, *end2, *middle1, *middle2, *middle3, *middle4;
    start1 = new HcCcCircle(*qa, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    middle1 = new HcCcCircle(*qa, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qc, !c2.left, c2.forward, true, parent_->hc_cc_circle_param_);
    end1 = new HcCcCircle(*qc, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);
    start2 = new HcCcCircle(*qd, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    middle3 = new HcCcCircle(*qd, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    middle4 = new HcCcCircle(*qf, !c2.left, c2.forward, true, parent_->hc_cc_circle_param_);
    end2 = new HcCcCircle(*qf, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);

    *q1 = new Configuration(c1.start.x, c1.start.y, c1.start.theta, c1.kappa);
    *q3 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);

    // select shortest connection
    double length1 = start1->HcTurnLength(**q1) + middle1->HcTurnLength(*qb) + middle2->HcTurnLength(*qb) +
                     end1->HcTurnLength(**q3);
    double length2 = start2->HcTurnLength(**q1) + middle3->HcTurnLength(*qe) + middle4->HcTurnLength(*qe) +
                     end2->HcTurnLength(**q3);
    if (length1 < length2)
    {
      *cstart = start1;
      *cend = end1;
      *ci1 = middle1;
      *ci2 = middle2;
      *q2 = qb;
      delete qa;
      delete qc;
      delete qd;
      delete qe;
      delete qf;
      delete start2, delete end2, delete middle3;
      delete middle4;
      return length1;
    }
    else
    {
      *cstart = start2;
      *cend = end2;
      *ci1 = middle3;
      *ci2 = middle4;
      *q2 = qe;
      delete qa;
      delete qb;
      delete qc;
      delete qd;
      delete qf;
      delete start1;
      delete end1;
      delete middle1;
      delete middle2;
      return length2;
    }
    return numeric_limits<double>::max();
  }

  // ##### TcTTcT ###############################################################
  bool TcTTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return (distance <= 4 * fabs(c1.kappa_inv) + 2 * parent_->radius_) &&
           (distance >= 4 * fabs(c1.kappa_inv) - 2 * parent_->radius_);
  }

  void TcTTcT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                              Configuration **q3, Configuration **q4, Configuration **q5, Configuration **q6) const
  {
    double theta = angle;
    double r1 = 2 * fabs(c1.kappa_inv);
    double r2 = parent_->radius_;
    double delta_x = (pow(r1, 2) + pow(distance / 2, 2) - pow(r2, 2)) / distance;
    double delta_y = sqrt(pow(r1, 2) - pow(delta_x, 2));
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
    HcCcCircle tgt2(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    HcCcCircle tgt3(x, y, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    HcCcCircle tgt4(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TT_tangent_circles(tgt1, tgt2, q2);
    TcT_tangent_circles(tgt2, c2, q3);

    TcT_tangent_circles(c1, tgt3, q4);
    TT_tangent_circles(tgt3, tgt4, q5);
    TcT_tangent_circles(tgt4, c2, q6);
  }

  double TcTTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                     Configuration **q1, Configuration **q2, HcCcCircle **ci1, HcCcCircle **ci2) const
  {
    Configuration *qa, *qb, *qc, *qd, *qe, *qf;
    TcTTcT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd, &qe, &qf);
    HcCcCircle *middle1, *middle2, *middle3, *middle4;
    middle1 = new HcCcCircle(*qb, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qb, c1.left, !c1.forward, true, parent_->hc_cc_circle_param_);
    middle3 = new HcCcCircle(*qe, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    middle4 = new HcCcCircle(*qe, c1.left, !c1.forward, true, parent_->hc_cc_circle_param_);

    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(c2);

    // select shortest connection
    double length1 = (*cstart)->RsTurnLength(*qa) + middle1->HcTurnLength(*qa) + middle2->HcTurnLength(*qc) +
                     (*cend)->RsTurnLength(*qc);
    double length2 = (*cstart)->RsTurnLength(*qd) + middle3->HcTurnLength(*qd) + middle4->HcTurnLength(*qf) +
                     (*cend)->RsTurnLength(*qf);
    if (length1 < length2)
    {
      *q1 = qa;
      *q2 = qc;
      *ci1 = middle1;
      *ci2 = middle2;
      delete qb;
      delete qd;
      delete qe;
      delete qf;
      delete middle3;
      delete middle4;
      return length1;
    }
    else
    {
      *q1 = qd;
      *q2 = qf;
      *ci1 = middle3;
      *ci2 = middle4;
      delete qa;
      delete qb;
      delete qc;
      delete qe;
      delete middle1;
      delete middle2;
      return length2;
    }
    return numeric_limits<double>::max();
  }

  // ############################################################################

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
    return distance <= 4 * parent_->radius_;
  }

  void TTT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                           Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r = 2 * parent_->radius_;
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
    HcCcCircle *start1, *start2, *end1, *end2, *middle1, *middle2;
    start1 = new HcCcCircle(*qa, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    middle1 = new HcCcCircle(*qa, !c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    end1 = new HcCcCircle(*qb, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);
    start2 = new HcCcCircle(*qc, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qc, !c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    end2 = new HcCcCircle(*qd, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);

    *q1 = new Configuration(c1.start.x, c1.start.y, c1.start.theta, c1.kappa);
    *q3 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);

    // select shortest connection
    double length1 = start1->HcTurnLength(**q1) + middle1->CcTurnLength(*qb) + end1->HcTurnLength(**q3);
    double length2 = start2->HcTurnLength(**q1) + middle2->CcTurnLength(*qd) + end2->HcTurnLength(**q3);
    if (length1 < length2)
    {
      *cstart = start1;
      *ci = middle1;
      *cend = end1;
      *q2 = qb;
      delete qa;
      delete qc;
      delete qd;
      delete start2;
      delete middle2;
      delete end2;
      return length1;
    }
    else
    {
      *cstart = start2;
      *ci = middle2;
      *cend = end2;
      *q2 = qd;
      delete qa;
      delete qb;
      delete qc;
      delete start1;
      delete middle1;
      delete end1;
      return length2;
    }
    return numeric_limits<double>::max();
  }

  // ##### TcST ################################################################
  bool TciSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return distance >= sqrt(pow(parent_->radius_ * parent_->sin_mu_, 2) +
                            pow(parent_->radius_ * parent_->cos_mu_ + fabs(c1.kappa_inv), 2));
  }

  bool TceSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return distance >= sqrt(pow(parent_->radius_ * parent_->sin_mu_, 2) +
                            pow(parent_->radius_ * parent_->cos_mu_ - fabs(c1.kappa_inv), 2));
  }

  bool TcSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TciSTExists(c1, c2) || TceSTExists(c1, c2);
  }

  double TciSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3) const
  {
    double alpha = asin((parent_->radius_ * parent_->cos_mu_ + fabs(c1.kappa_inv)) / distance);
    double delta_x1 = 0.0;
    double delta_y1 = fabs(c1.kappa_inv);
    double delta_x2 = parent_->radius_ * parent_->sin_mu_;
    double delta_y2 = parent_->radius_ * parent_->cos_mu_;
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    *q3 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);
    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(**q2, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);
    return (*cstart)->RsTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->HcTurnLength(**q3);
  }

  double TceSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3) const
  {
    double alpha = asin((parent_->radius_ * parent_->cos_mu_ - fabs(c1.kappa_inv)) / distance);
    double delta_x1 = 0.0;
    double delta_y1 = fabs(c1.kappa_inv);
    double delta_x2 = parent_->radius_ * parent_->sin_mu_;
    double delta_y2 = parent_->radius_ * parent_->cos_mu_;
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    *q3 = new Configuration(c2.start.x, c2.start.y, c2.start.theta, c2.kappa);
    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(**q2, c2.left, !c2.forward, kHcRegular, parent_->hc_cc_circle_param_);
    return (*cstart)->RsTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->HcTurnLength(**q3);
  }

  double TcSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                   Configuration **q1, Configuration **q2, Configuration **q3) const
  {
    if (TciSTExists(c1, c2))
    {
      return TciSTPath(c1, c2, cstart, cend, q1, q2, q3);
    }
    if (TceSTExists(c1, c2))
    {
      return TceSTPath(c1, c2, cstart, cend, q1, q2, q3);
    }
    return numeric_limits<double>::max();
  }

  // ##### TScT #################################################################
  bool TiScTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return distance >= sqrt(pow(parent_->radius_ * parent_->sin_mu_, 2) +
                            pow(parent_->radius_ * parent_->cos_mu_ + fabs(c1.kappa_inv), 2));
  }

  bool TeScTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward != c2.forward)
    {
      return false;
    }
    return distance >= sqrt(pow(parent_->radius_ * parent_->sin_mu_, 2) +
                            pow(parent_->radius_ * parent_->cos_mu_ - fabs(c1.kappa_inv), 2));
  }

  bool TScTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TiScTExists(c1, c2) || TeScTExists(c1, c2);
  }

  double TiScTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3) const
  {
    double alpha = asin((parent_->radius_ * parent_->cos_mu_ + fabs(c1.kappa_inv)) / distance);
    double delta_x1 = parent_->radius_ * parent_->sin_mu_;
    double delta_y1 = parent_->radius_ * parent_->cos_mu_;
    double delta_x2 = 0.0;
    double delta_y2 = fabs(c1.kappa_inv);
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q3 = new Configuration(x, y, theta, c2.kappa);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q3 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q3 = new Configuration(x, y, theta, c2.kappa);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q3 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    *q1 = new Configuration(c1.start.x, c1.start.y, c1.start.theta, c1.kappa);
    *cstart = new HcCcCircle(**q2, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2);
    return (*cstart)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) + (*cend)->RsTurnLength(**q3);
  }

  double TeScTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3) const
  {
    double alpha = asin((parent_->radius_ * parent_->cos_mu_ - fabs(c1.kappa_inv)) / distance);
    double delta_x1 = parent_->radius_ * parent_->sin_mu_;
    double delta_y1 = parent_->radius_ * parent_->cos_mu_;
    double delta_x2 = 0.0;
    double delta_y2 = fabs(c1.kappa_inv);
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q3 = new Configuration(x, y, theta, c2.kappa);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q3 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q3 = new Configuration(x, y, theta, c2.kappa);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q3 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    *q1 = new Configuration(c1.start.x, c1.start.y, c1.start.theta, c1.kappa);
    *cstart = new HcCcCircle(**q2, c1.left, !c1.forward, kHcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2);
    return (*cstart)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) + (*cend)->RsTurnLength(**q3);
  }

  double TScTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                   Configuration **q1, Configuration **q2, Configuration **q3) const
  {
    if (TiScTExists(c1, c2))
    {
      return TiScTPath(c1, c2, cstart, cend, q1, q2, q3);
    }
    if (TeScTExists(c1, c2))
    {
      return TeScTPath(c1, c2, cstart, cend, q1, q2, q3);
    }
    return numeric_limits<double>::max();
  }

  // ##### TcScT ################################################################
  bool TciScTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left == c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return distance > 2 * fabs(c1.kappa_inv);
  }

  bool TceScTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    if (c1.left != c2.left)
    {
      return false;
    }
    if (c1.forward == c2.forward)
    {
      return false;
    }
    return distance >= GetEpsilon();
  }

  bool TcScTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TciScTExists(c1, c2) || TceScTExists(c1, c2);
  }

  double TciScTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                     Configuration **q1, Configuration **q2) const
  {
    double alpha = asin(2 / (fabs(c1.kappa) * distance));
    double delta_x = 0.0;
    double delta_y = fabs(c1.kappa_inv);
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(c2);
    return (*cstart)->RsTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->RsTurnLength(**q2);
  }

  double TceScTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                     Configuration **q1, Configuration **q2) const
  {
    double theta = angle;
    double delta_x = 0.0;
    double delta_y = fabs(c1.kappa_inv);
    double x, y;
    if (c1.left && c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    if (c1.left && !c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (!c1.left && c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    if (!c1.left && !c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, c1.kappa);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(c2);
    return (*cstart)->RsTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->RsTurnLength(**q2);
  }

  double TcScTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2) const
  {
    if (TciScTExists(c1, c2))
    {
      return TciScTPath(c1, c2, cstart, cend, q1, q2);
    }
    if (TceScTExists(c1, c2))
    {
      return TceScTPath(c1, c2, cstart, cend, q1, q2);
    }
    return numeric_limits<double>::max();
  }
};

// ############################################################################

HcpmpmReedsSheppStateSpace::HcpmpmReedsSheppStateSpace(double kappa, double sigma, double discretization)
  : HcCcStateSpace(kappa, sigma, discretization)
  , hcpmpm_reeds_shepp_{ unique_ptr<Impl>(new Impl(this)) }
{
  rs_circle_param_.SetParam(kappa_, numeric_limits<double>::max(), 1 / kappa_, 0.0, 0.0, 1.0, 0.0);
  radius_ = hc_cc_circle_param_.radius;
  mu_ = hc_cc_circle_param_.mu;
  sin_mu_ = hc_cc_circle_param_.sin_mu;
  cos_mu_ = hc_cc_circle_param_.cos_mu;
}

HcpmpmReedsSheppStateSpace::~HcpmpmReedsSheppStateSpace() = default;

HcCcRsPath *HcpmpmReedsSheppStateSpace::HcpmpmCirclesRsPath(const HcCcCircle &c1,
                                                                      const HcCcCircle &c2) const
{
  // table containing the lengths of the paths, the intermediate configurations and circles
  double length[kNbHcCcRsPaths];
  std::fill(length, (length) + (kNbHcCcRsPaths), numeric_limits<double>::max());
  Configuration *qi1[kNbHcCcRsPaths];
  std::fill(qi1, (qi1) + (kNbHcCcRsPaths), nullptr);
  Configuration *qi2[kNbHcCcRsPaths];
  std::fill(qi2, (qi2) + (kNbHcCcRsPaths), nullptr);
  Configuration *qi3[kNbHcCcRsPaths];
  std::fill(qi3, (qi3) + (kNbHcCcRsPaths), nullptr);
  Configuration *qi4[kNbHcCcRsPaths];
  std::fill(qi4, (qi4) + (kNbHcCcRsPaths), nullptr);
  HcCcCircle *cstart[kNbHcCcRsPaths];
  std::fill(cstart, (cstart) + (kNbHcCcRsPaths), nullptr);
  HcCcCircle *ci1[kNbHcCcRsPaths];
  std::fill(ci1, (ci1) + (kNbHcCcRsPaths), nullptr);
  HcCcCircle *ci2[kNbHcCcRsPaths];
  std::fill(ci2, (ci2) + (kNbHcCcRsPaths), nullptr);
  HcCcCircle *cend[kNbHcCcRsPaths];
  std::fill(cend, (cend) + (kNbHcCcRsPaths), nullptr);

  // precomputations
  hcpmpm_reeds_shepp_->distance = CenterDistance(c1, c2);
  hcpmpm_reeds_shepp_->angle = atan2(c2.yc - c1.yc, c2.xc - c1.xc);

  // case E
  if (ConfigurationEqual(c1.start, c2.start))
  {
    length[hc_cc_rs::E] = 0;
    goto label_end;
  }
  // case T
  if (ConfigurationOnHcCcCircle(c1, c2.start))
  {
    cstart[hc_cc_rs::T] = new HcCcCircle(c1.start, c1.left, c1.forward, true, rs_circle_param_);
    length[hc_cc_rs::T] = cstart[hc_cc_rs::T]->RsTurnLength(c2.start);
    goto label_end;
  }
  // case TT
  if (hcpmpm_reeds_shepp_->TTExists(c1, c2))
  {
    length[hc_cc_rs::TT] = hcpmpm_reeds_shepp_->TTPath(c1, c2, &cstart[hc_cc_rs::TT], &cend[hc_cc_rs::TT],
                                                        &qi1[hc_cc_rs::TT], &qi2[hc_cc_rs::TT], &qi3[hc_cc_rs::TT]);
  }
  // case TcT
  if (hcpmpm_reeds_shepp_->TcTExists(c1, c2))
  {
    length[hc_cc_rs::TcT] =
        hcpmpm_reeds_shepp_->TcTPath(c1, c2, &cstart[hc_cc_rs::TcT], &cend[hc_cc_rs::TcT], &qi1[hc_cc_rs::TcT]);
  }
  // ##### Reeds-Shepp families: ############################################
  // case TcTcT
  if (hcpmpm_reeds_shepp_->TcTcTExists(c1, c2))
  {
    length[hc_cc_rs::TcTcT] =
        hcpmpm_reeds_shepp_->TcTcTPath(c1, c2, &cstart[hc_cc_rs::TcTcT], &cend[hc_cc_rs::TcTcT], &qi1[hc_cc_rs::TcTcT],
                                        &qi2[hc_cc_rs::TcTcT], &ci1[hc_cc_rs::TcTcT]);
  }
  // case TcTT
  if (hcpmpm_reeds_shepp_->TcTTExists(c1, c2))
  {
    length[hc_cc_rs::TcTT] =
        hcpmpm_reeds_shepp_->TcTTPath(c1, c2, &cstart[hc_cc_rs::TcTT], &cend[hc_cc_rs::TcTT], &qi1[hc_cc_rs::TcTT],
                                       &qi2[hc_cc_rs::TcTT], &ci1[hc_cc_rs::TcTT]);
  }
  // case TTcT
  if (hcpmpm_reeds_shepp_->TTcTExists(c1, c2))
  {
    length[hc_cc_rs::TTcT] =
        hcpmpm_reeds_shepp_->TTcTPath(c1, c2, &cstart[hc_cc_rs::TTcT], &cend[hc_cc_rs::TTcT], &qi1[hc_cc_rs::TTcT],
                                       &qi2[hc_cc_rs::TTcT], &ci1[hc_cc_rs::TTcT]);
  }
  // case TST
  if (hcpmpm_reeds_shepp_->TSTExists(c1, c2))
  {
    length[hc_cc_rs::TST] =
        hcpmpm_reeds_shepp_->TSTPath(c1, c2, &cstart[hc_cc_rs::TST], &cend[hc_cc_rs::TST], &qi1[hc_cc_rs::TST],
                                      &qi2[hc_cc_rs::TST], &qi3[hc_cc_rs::TST], &qi4[hc_cc_rs::TST]);
  }
  // case TSTcT
  if (hcpmpm_reeds_shepp_->TSTcTExists(c1, c2))
  {
    length[hc_cc_rs::TSTcT] = hcpmpm_reeds_shepp_->TSTcTPath(
        c1, c2, &cstart[hc_cc_rs::TSTcT], &cend[hc_cc_rs::TSTcT], &qi1[hc_cc_rs::TSTcT], &qi2[hc_cc_rs::TSTcT],
        &qi3[hc_cc_rs::TSTcT], &qi4[hc_cc_rs::TSTcT], &ci1[hc_cc_rs::TSTcT]);
  }
  // case TcTST
  if (hcpmpm_reeds_shepp_->TcTSTExists(c1, c2))
  {
    length[hc_cc_rs::TcTST] = hcpmpm_reeds_shepp_->TcTSTPath(
        c1, c2, &cstart[hc_cc_rs::TcTST], &cend[hc_cc_rs::TcTST], &qi1[hc_cc_rs::TcTST], &qi2[hc_cc_rs::TcTST],
        &qi3[hc_cc_rs::TcTST], &qi4[hc_cc_rs::TcTST], &ci1[hc_cc_rs::TcTST]);
  }
  // case TcTSTcT
  if (hcpmpm_reeds_shepp_->TcTSTcTExists(c1, c2))
  {
    length[hc_cc_rs::TcTSTcT] = hcpmpm_reeds_shepp_->TcTSTcTPath(
        c1, c2, &cstart[hc_cc_rs::TcTSTcT], &cend[hc_cc_rs::TcTSTcT], &qi1[hc_cc_rs::TcTSTcT], &qi2[hc_cc_rs::TcTSTcT],
        &qi3[hc_cc_rs::TcTSTcT], &qi4[hc_cc_rs::TcTSTcT], &ci1[hc_cc_rs::TcTSTcT], &ci2[hc_cc_rs::TcTSTcT]);
  }
  // case TTcTT
  if (hcpmpm_reeds_shepp_->TTcTTExists(c1, c2))
  {
    length[hc_cc_rs::TTcTT] = hcpmpm_reeds_shepp_->TTcTTPath(
        c1, c2, &cstart[hc_cc_rs::TTcTT], &cend[hc_cc_rs::TTcTT], &qi1[hc_cc_rs::TTcTT], &qi2[hc_cc_rs::TTcTT],
        &qi3[hc_cc_rs::TTcTT], &ci1[hc_cc_rs::TTcTT], &ci2[hc_cc_rs::TTcTT]);
  }
  // case TcTTcT
  if (hcpmpm_reeds_shepp_->TcTTcTExists(c1, c2))
  {
    length[hc_cc_rs::TcTTcT] = hcpmpm_reeds_shepp_->TcTTcTPath(
        c1, c2, &cstart[hc_cc_rs::TcTTcT], &cend[hc_cc_rs::TcTTcT], &qi1[hc_cc_rs::TcTTcT], &qi2[hc_cc_rs::TcTTcT],
        &ci1[hc_cc_rs::TcTTcT], &ci2[hc_cc_rs::TcTTcT]);
  }
  //  ############################################################################
  // case TTT
  if (hcpmpm_reeds_shepp_->TTTExists(c1, c2))
  {
    length[hc_cc_rs::TTT] =
        hcpmpm_reeds_shepp_->TTTPath(c1, c2, &cstart[hc_cc_rs::TTT], &cend[hc_cc_rs::TTT], &qi1[hc_cc_rs::TTT],
                                      &qi2[hc_cc_rs::TTT], &qi3[hc_cc_rs::TTT], &ci1[hc_cc_rs::TTT]);
  }
  // case TcST
  if (hcpmpm_reeds_shepp_->TcSTExists(c1, c2))
  {
    length[hc_cc_rs::TcST] =
        hcpmpm_reeds_shepp_->TcSTPath(c1, c2, &cstart[hc_cc_rs::TcST], &cend[hc_cc_rs::TcST], &qi1[hc_cc_rs::TcST],
                                       &qi2[hc_cc_rs::TcST], &qi3[hc_cc_rs::TcST]);
  }
  // case TScT
  if (hcpmpm_reeds_shepp_->TScTExists(c1, c2))
  {
    length[hc_cc_rs::TScT] =
        hcpmpm_reeds_shepp_->TScTPath(c1, c2, &cstart[hc_cc_rs::TScT], &cend[hc_cc_rs::TScT], &qi1[hc_cc_rs::TScT],
                                       &qi2[hc_cc_rs::TScT], &qi3[hc_cc_rs::TScT]);
  }
  // case TcScT
  if (hcpmpm_reeds_shepp_->TcScTExists(c1, c2))
  {
    length[hc_cc_rs::TcScT] = hcpmpm_reeds_shepp_->TcScTPath(c1, c2, &cstart[hc_cc_rs::TcScT], &cend[hc_cc_rs::TcScT],
                                                              &qi1[hc_cc_rs::TcScT], &qi2[hc_cc_rs::TcScT]);
  }
label_end:
  // select shortest path
  hc_cc_rs::path_type best_path = (hc_cc_rs::path_type)static_cast<int>(std::min_element(length, (length) + (kNbHcCcRsPaths)) - (length));
  HcCcRsPath *path;
  path = new HcCcRsPath(c1.start, c2.start, best_path, kappa_, sigma_, qi1[best_path], qi2[best_path],
                           qi3[best_path], qi4[best_path], cstart[best_path], cend[best_path], ci1[best_path],
                           ci2[best_path], length[best_path]);

  // clean up
  for (int i = 0; i < kNbHcCcRsPaths; i++)
  {
    if (i != best_path)
    {
      delete qi1[i];
      delete qi2[i];
      delete qi3[i];
      delete qi4[i];
      delete cstart[i];
      delete ci1[i];
      delete ci2[i];
      delete cend[i];
    }
  }
  return path;
}

HcCcRsPath *HcpmpmReedsSheppStateSpace::HcpmpmReedsShepp(const State &state1, const State &state2) const
{
  // compute the 4 circles at the intial and final configuration
  Configuration start1(state1.x, state1.y, state1.theta, kappa_);
  Configuration start2(state1.x, state1.y, state1.theta, -kappa_);
  Configuration end1(state2.x, state2.y, state2.theta, kappa_);
  Configuration end2(state2.x, state2.y, state2.theta, -kappa_);

  HcCcCircle *start_circle[4];
  HcCcCircle *end_circle[4];
  start_circle[0] = new HcCcCircle(start1, true, true, true, rs_circle_param_);
  start_circle[1] = new HcCcCircle(start2, false, true, true, rs_circle_param_);
  start_circle[2] = new HcCcCircle(start1, true, false, true, rs_circle_param_);
  start_circle[3] = new HcCcCircle(start2, false, false, true, rs_circle_param_);
  end_circle[0] = new HcCcCircle(end1, true, true, true, rs_circle_param_);
  end_circle[1] = new HcCcCircle(end2, false, true, true, rs_circle_param_);
  end_circle[2] = new HcCcCircle(end1, true, false, true, rs_circle_param_);
  end_circle[3] = new HcCcCircle(end2, false, false, true, rs_circle_param_);

  // compute the shortest path for the 16 combinations (4 circles at the beginning and 4 at the end)
  HcCcRsPath *path[] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                            nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };

  double lg[] = { numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                  numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                  numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                  numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                  numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max(),
                  numeric_limits<double>::max() };

  for (int i = 0; i < 4; i++)
  {
    // skip circle at the beginning for curvature continuity
    if (i == 0 && state1.kappa < 0)
      continue;
    else if (i == 1 && state1.kappa > 0)
      continue;
    else if (i == 2 && state1.kappa < 0)
      continue;
    else if (i == 3 && state1.kappa > 0)
      continue;
    for (int j = 0; j < 4; j++)
    {
      // skip circle at the end for curvature continuity
      if (j == 0 && state2.kappa < 0)
        continue;
      else if (j == 1 && state2.kappa > 0)
        continue;
      else if (j == 2 && state2.kappa < 0)
        continue;
      else if (j == 3 && state2.kappa > 0)
        continue;
      path[4 * i + j] = HcpmpmCirclesRsPath(*start_circle[i], *end_circle[j]);
      if (path[4 * i + j])
      {
        lg[4 * i + j] = path[4 * i + j]->length;
      }
    }
  }

  // select shortest path
  int best_path = static_cast<int>(std::min_element(lg, (lg) + (16)) - (lg));

  //  // display calculations
  //  cout << "HcpmpmReedsSheppStateSpace" << endl;
  //  for (int i = 0; i < 16; i++)
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
  for (int i = 0; i < 4; i++)
  {
    delete start_circle[i];
    delete end_circle[i];
  }
  for (int i = 0; i < 16; i++)
  {
    if (i != best_path)
    {
      delete path[i];
    }
  }
  return path[best_path];
}

double HcpmpmReedsSheppStateSpace::GetDistance(const State &state1, const State &state2) const
{
  HcCcRsPath *p = this->HcpmpmReedsShepp(state1, state2);
  double length = p->length;
  delete p;
  return length;
}

vector<Control> HcpmpmReedsSheppStateSpace::GetControls(const State &state1, const State &state2) const
{
  vector<Control> hc_rs_controls;
  hc_rs_controls.reserve(8);
  HcCcRsPath *p = this->HcpmpmReedsShepp(state1, state2);
  switch (p->type)
  {
    case hc_cc_rs::E:
      EmptyControls(hc_rs_controls);
      break;
    case hc_cc_rs::T:
      RsTurnControls(*(p->cstart), p->end, true, hc_rs_controls);
      break;
    case hc_cc_rs::TT:
      HcTurnControls(*(p->cstart), *(p->qi1), false, hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi3), true, hc_rs_controls);
      break;
    case hc_cc_rs::TcT:
      RsTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      RsTurnControls(*(p->cend), *(p->qi1), false, hc_rs_controls);
      break;
    // ##### Reeds-Shepp families: ############################################
    case hc_cc_rs::TcTcT:
      RsTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      RsTurnControls(*(p->ci1), *(p->qi2), true, hc_rs_controls);
      RsTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
      break;
    case hc_cc_rs::TcTT:
      RsTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi1), false, hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi2), true, hc_rs_controls);
      break;
    case hc_cc_rs::TTcT:
      HcTurnControls(*(p->cstart), *(p->qi1), false, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi2), true, hc_rs_controls);
      RsTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
      break;
    case hc_cc_rs::TST:
      HcTurnControls(*(p->cstart), *(p->qi1), false, hc_rs_controls);
      StraightControls(*(p->qi2), *(p->qi3), hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi4), true, hc_rs_controls);
      break;
    case hc_cc_rs::TSTcT:
      HcTurnControls(*(p->cstart), *(p->qi1), false, hc_rs_controls);
      StraightControls(*(p->qi2), *(p->qi3), hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi4), true, hc_rs_controls);
      RsTurnControls(*(p->cend), *(p->qi4), false, hc_rs_controls);
      break;
    case hc_cc_rs::TcTST:
      RsTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi1), false, hc_rs_controls);
      StraightControls(*(p->qi2), *(p->qi3), hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi4), true, hc_rs_controls);
      break;
    case hc_cc_rs::TcTSTcT:
      RsTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi1), false, hc_rs_controls);
      StraightControls(*(p->qi2), *(p->qi3), hc_rs_controls);
      HcTurnControls(*(p->ci2), *(p->qi4), true, hc_rs_controls);
      RsTurnControls(*(p->cend), *(p->qi4), false, hc_rs_controls);
      break;
    case hc_cc_rs::TTcTT:
      HcTurnControls(*(p->cstart), *(p->qi1), false, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi2), true, hc_rs_controls);
      HcTurnControls(*(p->ci2), *(p->qi2), false, hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi3), true, hc_rs_controls);
      break;
    case hc_cc_rs::TcTTcT:
      RsTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi1), false, hc_rs_controls);
      HcTurnControls(*(p->ci2), *(p->qi2), true, hc_rs_controls);
      RsTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
      break;
    // ########################################################################
    case hc_cc_rs::TTT:
      HcTurnControls(*(p->cstart), *(p->qi1), false, hc_rs_controls);
      CcTurnControls(*(p->ci1), *(p->qi2), true, hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi3), true, hc_rs_controls);
      break;
    case hc_cc_rs::TcST:
      RsTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      StraightControls(*(p->qi1), *(p->qi2), hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi3), true, hc_rs_controls);
      break;
    case hc_cc_rs::TScT:
      HcTurnControls(*(p->cstart), *(p->qi1), false, hc_rs_controls);
      StraightControls(*(p->qi2), *(p->qi3), hc_rs_controls);
      RsTurnControls(*(p->cend), *(p->qi3), false, hc_rs_controls);
      break;
    case hc_cc_rs::TcScT:
      RsTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      StraightControls(*(p->qi1), *(p->qi2), hc_rs_controls);
      RsTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
      break;
    default:
      break;
  }
  delete p;
  return hc_rs_controls;
}

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
