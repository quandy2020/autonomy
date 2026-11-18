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
 * @file hybrid_zero_zero.cpp
 * @brief Hybrid-curvature Reeds–Shepp with zero curvature at both ends (implementation).
 */

#include <cmath>
#include <limits>

#include "autonomy/common/math/steering/clothoid/hybrid_zero_zero.hpp"
#include "autonomy/common/math/steering/clothoid/configuration.hpp"
#include "autonomy/common/math/steering/utilities.hpp"
#include <algorithm>

constexpr bool kCcRegular = false;

using namespace std;

namespace autonomy {
namespace common {
namespace math {
namespace steering {


class Hc00ReedsSheppStateSpace::Impl
{
private:
  Hc00ReedsSheppStateSpace *parent_;

public:
  explicit Impl(Hc00ReedsSheppStateSpace *parent)
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
                 Configuration **q) const
  {
    TT_tangent_circles(c1, c2, q);
    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2.start, c2.left, c2.forward, kCcRegular, parent_->hc_cc_circle_param_);
    return (*cstart)->CcTurnLength(**q) + (*cend)->CcTurnLength(**q);
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
    return (*cstart)->HcTurnLength(**q) + (*cend)->HcTurnLength(**q);
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
    HcCcCircle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    HcCcCircle tgt2(x, y, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);

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
    double length1 = (*cstart)->HcTurnLength(*qa) + middle1->RsTurnLength(*qb) + (*cend)->HcTurnLength(*qb);
    double length2 = (*cstart)->HcTurnLength(*qc) + middle2->RsTurnLength(*qd) + (*cend)->HcTurnLength(*qd);
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
    return (distance <= 2 * c1.radius + 2 * fabs(c1.kappa_inv)) && (distance >= 2 * c1.radius - 2 * fabs(c1.kappa_inv));
  }

  void TcTT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                            Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r1 = 2 * fabs(c1.kappa_inv);
    double r2 = 2 * c1.radius;
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
    HcCcCircle *middle1, *middle2;
    middle1 = new HcCcCircle(*qb, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qd, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(c2.start, c2.left, c2.forward, kCcRegular, parent_->hc_cc_circle_param_);

    // select shortest connection
    double length1 = (*cstart)->HcTurnLength(*qa) + middle1->HcTurnLength(*qa) + (*cend)->CcTurnLength(*qb);
    double length2 = (*cstart)->HcTurnLength(*qc) + middle2->HcTurnLength(*qc) + (*cend)->CcTurnLength(*qd);
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
    return (distance <= 2 * c1.radius + 2 * fabs(c1.kappa_inv)) && (distance >= 2 * c1.radius - 2 * fabs(c1.kappa_inv));
  }

  void TTcT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                            Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r1 = 2 * c1.radius;
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
    HcCcCircle *middle1, *middle2;
    middle1 = new HcCcCircle(*qa, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qc, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2);

    // select shortest connection
    double length1 = (*cstart)->CcTurnLength(*qa) + middle1->HcTurnLength(*qb) + (*cend)->HcTurnLength(*qb);
    double length2 = (*cstart)->CcTurnLength(*qc) + middle2->HcTurnLength(*qd) + (*cend)->HcTurnLength(*qd);
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
                   Configuration **q1, Configuration **q2) const
  {
    TiST_tangent_circles(c1, c2, q1, q2);
    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2.start, c2.left, c2.forward, kCcRegular, parent_->hc_cc_circle_param_);
    return (*cstart)->CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->CcTurnLength(**q2);
  }

  double TeSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                   Configuration **q1, Configuration **q2) const
  {
    TeST_tangent_circles(c1, c2, q1, q2);
    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2.start, c2.left, c2.forward, kCcRegular, parent_->hc_cc_circle_param_);
    return (*cstart)->CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->CcTurnLength(**q2);
  }

  double TSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                  Configuration **q1, Configuration **q2) const
  {
    if (TiSTExists(c1, c2))
    {
      return TiSTPath(c1, c2, cstart, cend, q1, q2);
    }
    if (TeSTExists(c1, c2))
    {
      return TeSTPath(c1, c2, cstart, cend, q1, q2);
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
    return (distance >=
            sqrt(pow(2 * c1.radius * c1.sin_mu + 2 * fabs(c1.kappa_inv), 2) + pow(2 * c1.radius * c1.cos_mu, 2)));
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
    return (distance >= 2 * (fabs(c1.kappa_inv) + c1.radius * c1.sin_mu));
  }

  bool TSTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TiSTcTExists(c1, c2) || TeSTcTExists(c1, c2);
  }

  double TiSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, HcCcCircle **ci) const
  {
    double theta = angle;
    double delta_y = (4 * c2.radius * c2.cos_mu) / (fabs(c2.kappa) * distance);
    double delta_x = sqrt(pow(2 * c2.kappa_inv, 2) - pow(delta_y, 2));
    double x, y;

    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TiST_tangent_circles(c1, tgt1, q1, q2);
    TcT_tangent_circles(tgt1, c2, q3);

    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2);
    *ci = new HcCcCircle(**q2, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    return (*cstart)->CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*ci)->HcTurnLength(**q3) +
           (*cend)->HcTurnLength(**q3);
  }

  double TeSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, HcCcCircle **ci) const
  {
    double theta = angle;
    double delta_x = 2 * fabs(c2.kappa_inv);
    double delta_y = 0;
    double x, y;

    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TeST_tangent_circles(c1, tgt1, q1, q2);
    TcT_tangent_circles(tgt1, c2, q3);

    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2);
    *ci = new HcCcCircle(**q2, c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    return (*cstart)->CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*ci)->HcTurnLength(**q3) +
           (*cend)->HcTurnLength(**q3);
  }

  double TSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3, HcCcCircle **ci) const
  {
    if (TiSTcTExists(c1, c2))
    {
      return TiSTcTPath(c1, c2, cstart, cend, q1, q2, q3, ci);
    }
    if (TeSTcTExists(c1, c2))
    {
      return TeSTcTPath(c1, c2, cstart, cend, q1, q2, q3, ci);
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
    return (distance >=
            sqrt(pow(2 * c1.radius * c1.sin_mu + 2 * fabs(c1.kappa_inv), 2) + pow(2 * c1.radius * c1.cos_mu, 2)));
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
    return (distance >= 2 * (fabs(c1.kappa_inv) + c1.radius * c1.sin_mu));
  }

  bool TcTSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TcTiSTExists(c1, c2) || TcTeSTExists(c1, c2);
  }

  double TcTiSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, HcCcCircle **ci) const
  {
    double theta = angle;
    double delta_y = (4 * c2.radius * c2.cos_mu) / (fabs(c2.kappa) * distance);
    double delta_x = sqrt(pow(2 * c2.kappa_inv, 2) - pow(delta_y, 2));
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c2.left, !c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TiST_tangent_circles(tgt1, c2, q2, q3);

    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(c2.start, c2.left, c2.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *ci = new HcCcCircle(**q2, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    return (*cstart)->HcTurnLength(**q1) + (*ci)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) +
           (*cend)->CcTurnLength(**q3);
  }

  double TcTeSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                     Configuration **q1, Configuration **q2, Configuration **q3, HcCcCircle **ci) const
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
    *cend = new HcCcCircle(c2.start, c2.left, c2.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *ci = new HcCcCircle(**q2, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    return (*cstart)->HcTurnLength(**q1) + (*ci)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) +
           (*cend)->CcTurnLength(**q3);
  }

  double TcTSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2, Configuration **q3, HcCcCircle **ci) const
  {
    if (TcTiSTExists(c1, c2))
    {
      return TcTiSTPath(c1, c2, cstart, cend, q1, q2, q3, ci);
    }
    if (TcTeSTExists(c1, c2))
    {
      return TcTeSTPath(c1, c2, cstart, cend, q1, q2, q3, ci);
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
    return (distance >=
            sqrt(pow(2 * c1.radius, 2) + 16 * c1.radius * c1.sin_mu * fabs(c1.kappa_inv) + pow(4 * c1.kappa_inv, 2)));
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
    return (distance >= 4 * fabs(c1.kappa_inv) + 2 * c1.radius * c1.sin_mu);
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
    double delta_y = (4 * c1.radius * c1.cos_mu) / (distance * fabs(c1.kappa));
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

    return (*cstart)->HcTurnLength(**q1) + (*ci1)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) +
           (*ci2)->HcTurnLength(**q4) + (*cend)->HcTurnLength(**q4);
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

    return (*cstart)->HcTurnLength(**q1) + (*ci1)->HcTurnLength(**q1) + ConfigurationDistance(**q2, **q3) +
           (*ci2)->HcTurnLength(**q4) + (*cend)->HcTurnLength(**q4);
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
    return (distance <= 4 * c1.radius + 2 * fabs(c1.kappa_inv));
  }

  void TTcTT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                             Configuration **q3, Configuration **q4, Configuration **q5, Configuration **q6) const
  {
    double theta = angle;
    double r1, r2, delta_x, delta_y, x, y;
    r1 = 2 * fabs(c1.kappa_inv);
    r2 = 2 * c1.radius;
    if (distance < 4 * c1.radius - 2 * fabs(c1.kappa_inv))
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
    HcCcCircle *middle1, *middle2, *middle3, *middle4;
    middle1 = new HcCcCircle(*qa, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qc, !c2.left, c2.forward, true, parent_->hc_cc_circle_param_);
    middle3 = new HcCcCircle(*qd, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    middle4 = new HcCcCircle(*qf, !c2.left, c2.forward, true, parent_->hc_cc_circle_param_);

    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2.start, c2.left, c2.forward, kCcRegular, parent_->hc_cc_circle_param_);

    // select shortest connection
    double length1 = (*cstart)->CcTurnLength(*qa) + middle1->HcTurnLength(*qb) + middle2->HcTurnLength(*qb) +
                     (*cend)->CcTurnLength(*qc);
    double length2 = (*cstart)->CcTurnLength(*qd) + middle3->HcTurnLength(*qe) + middle4->HcTurnLength(*qe) +
                     (*cend)->CcTurnLength(*qf);
    if (length1 < length2)
    {
      *q1 = qa;
      *q2 = qb;
      *q3 = qc;
      *ci1 = middle1;
      *ci2 = middle2;
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
      *q2 = qe;
      *q3 = qf;
      *ci1 = middle3;
      *ci2 = middle4;
      delete qa;
      delete qb;
      delete qc;
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
    return (distance <= 4 * fabs(c1.kappa_inv) + 2 * c1.radius) && (distance >= 4 * fabs(c1.kappa_inv) - 2 * c1.radius);
  }

  void TcTTcT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                              Configuration **q3, Configuration **q4, Configuration **q5, Configuration **q6) const
  {
    double theta = angle;
    double r1 = 2 * fabs(c1.kappa_inv);
    double r2 = c1.radius;
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
                     Configuration **q1, Configuration **q2, Configuration **q3, HcCcCircle **ci1,
                     HcCcCircle **ci2) const
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
    double length1 = (*cstart)->HcTurnLength(*qa) + middle1->HcTurnLength(*qa) + middle2->HcTurnLength(*qc) +
                     (*cend)->HcTurnLength(*qc);
    double length2 = (*cstart)->HcTurnLength(*qd) + middle3->HcTurnLength(*qd) + middle4->HcTurnLength(*qf) +
                     (*cend)->HcTurnLength(*qf);
    if (length1 < length2)
    {
      *q1 = qa;
      *q2 = qb;
      *q3 = qc;
      *ci1 = middle1;
      *ci2 = middle2;
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
      *q2 = qe;
      *q3 = qf;
      *ci1 = middle3;
      *ci2 = middle4;
      delete qa;
      delete qb;
      delete qc;
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
                  Configuration **q1, Configuration **q2, HcCcCircle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TTT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    HcCcCircle *middle1, *middle2;
    middle1 = new HcCcCircle(*qa, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qc, !c1.left, c1.forward, true, parent_->hc_cc_circle_param_);

    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2.start, c2.left, c2.forward, kCcRegular, parent_->hc_cc_circle_param_);

    // select shortest connection
    double length1 = (*cstart)->CcTurnLength(*qa) + middle1->CcTurnLength(*qb) + (*cend)->CcTurnLength(*qb);
    double length2 = (*cstart)->CcTurnLength(*qc) + middle2->CcTurnLength(*qd) + (*cend)->CcTurnLength(*qd);
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
    return distance >= sqrt(pow(c1.radius * c1.sin_mu, 2) + pow(c1.radius * c1.cos_mu + fabs(c1.kappa_inv), 2));
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
    return distance >= sqrt(pow(c1.radius * c1.sin_mu, 2) + pow(c1.radius * c1.cos_mu - fabs(c1.kappa_inv), 2));
  }

  bool TcSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TciSTExists(c1, c2) || TceSTExists(c1, c2);
  }

  double TciSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2) const
  {
    double alpha = asin((c1.radius * c1.cos_mu + fabs(c1.kappa_inv)) / distance);
    double delta_x1 = 0.0;
    double delta_y1 = fabs(c1.kappa_inv);
    double delta_x2 = c1.radius * c1.sin_mu;
    double delta_y2 = c1.radius * c1.cos_mu;
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
    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(c2.start, c2.left, c2.forward, kCcRegular, parent_->hc_cc_circle_param_);
    return (*cstart)->HcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->CcTurnLength(**q2);
  }

  double TceSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2) const
  {
    double alpha = asin((c1.radius * c1.cos_mu - fabs(c1.kappa_inv)) / distance);
    double delta_x1 = 0.0;
    double delta_y1 = fabs(c1.kappa_inv);
    double delta_x2 = c1.radius * c1.sin_mu;
    double delta_y2 = c1.radius * c1.cos_mu;
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
    *cstart = new HcCcCircle(c1);
    *cend = new HcCcCircle(c2.start, c2.left, c2.forward, kCcRegular, parent_->hc_cc_circle_param_);
    return (*cstart)->HcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->CcTurnLength(**q2);
  }

  double TcSTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                   Configuration **q1, Configuration **q2) const
  {
    if (TciSTExists(c1, c2))
    {
      return TciSTPath(c1, c2, cstart, cend, q1, q2);
    }
    if (TceSTExists(c1, c2))
    {
      return TceSTPath(c1, c2, cstart, cend, q1, q2);
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
    return distance >= sqrt(pow(c1.radius * c1.sin_mu, 2) + pow(c1.radius * c1.cos_mu + fabs(c1.kappa_inv), 2));
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
    return distance >= sqrt(pow(c1.radius * c1.sin_mu, 2) + pow(c1.radius * c1.cos_mu - fabs(c1.kappa_inv), 2));
  }

  bool TScTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TiScTExists(c1, c2) || TeScTExists(c1, c2);
  }

  double TiScTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2) const
  {
    double alpha = asin((c1.radius * c1.cos_mu + fabs(c1.kappa_inv)) / distance);
    double delta_x1 = c1.radius * c1.sin_mu;
    double delta_y1 = c1.radius * c1.cos_mu;
    double delta_x2 = 0.0;
    double delta_y2 = fabs(c1.kappa_inv);
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2);
    return (*cstart)->CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->HcTurnLength(**q2);
  }

  double TeScTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                    Configuration **q1, Configuration **q2) const
  {
    double alpha = asin((c1.radius * c1.cos_mu - fabs(c1.kappa_inv)) / distance);
    double delta_x1 = c1.radius * c1.sin_mu;
    double delta_y1 = c1.radius * c1.cos_mu;
    double delta_x2 = 0.0;
    double delta_y2 = fabs(c1.kappa_inv);
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta, c2.kappa);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x1, -delta_y1, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x2, -delta_y2, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, c2.kappa);
    }
    *cstart = new HcCcCircle(c1.start, c1.left, c1.forward, kCcRegular, parent_->hc_cc_circle_param_);
    *cend = new HcCcCircle(c2);
    return (*cstart)->CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->HcTurnLength(**q2);
  }

  double TScTPath(const HcCcCircle &c1, const HcCcCircle &c2, HcCcCircle **cstart, HcCcCircle **cend,
                   Configuration **q1, Configuration **q2) const
  {
    if (TiScTExists(c1, c2))
    {
      return TiScTPath(c1, c2, cstart, cend, q1, q2);
    }
    if (TeScTExists(c1, c2))
    {
      return TeScTPath(c1, c2, cstart, cend, q1, q2);
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
    return (*cstart)->HcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->HcTurnLength(**q2);
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
    return (*cstart)->HcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*cend)->HcTurnLength(**q2);
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

Hc00ReedsSheppStateSpace::Hc00ReedsSheppStateSpace(double kappa, double sigma, double discretization)
  : HcCcStateSpace(kappa, sigma, discretization)
  , hc00_reeds_shepp_{ unique_ptr<Impl>(new Impl(this)) }
{
  rs_circle_param_.SetParam(kappa_, numeric_limits<double>::max(), 1 / kappa_, 0.0, 0.0, 1.0, 0.0);
}

Hc00ReedsSheppStateSpace::~Hc00ReedsSheppStateSpace() = default;

HcCcRsPath *Hc00ReedsSheppStateSpace::Hc00CirclesRsPath(const HcCcCircle &c1, const HcCcCircle &c2) const
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
  hc00_reeds_shepp_->distance = CenterDistance(c1, c2);
  hc00_reeds_shepp_->angle = atan2(c2.yc - c1.yc, c2.xc - c1.xc);

  // case E
  if (ConfigurationEqual(c1.start, c2.start))
  {
    length[hc_cc_rs::E] = 0;
    goto label_end;
  }
  // case S forwards
  if (ConfigurationAligned(c1.start, c2.start))
  {
    length[hc_cc_rs::S] = ConfigurationDistance(c1.start, c2.start);
    goto label_end;
  }
  // case S backwards
  if (ConfigurationAligned(c2.start, c1.start))
  {
    length[hc_cc_rs::S] = ConfigurationDistance(c2.start, c1.start);
    goto label_end;
  }
  // case T
  if (ConfigurationOnHcCcCircle(c1, c2.start))
  {
    cstart[hc_cc_rs::T] = new HcCcCircle(c1.start, c1.left, c1.forward, kCcRegular, hc_cc_circle_param_);
    length[hc_cc_rs::T] = cstart[hc_cc_rs::T]->CcTurnLength(c2.start);
    goto label_end;
  }
  // case TT
  if (hc00_reeds_shepp_->TTExists(c1, c2))
  {
    length[hc_cc_rs::TT] =
        hc00_reeds_shepp_->TTPath(c1, c2, &cstart[hc_cc_rs::TT], &cend[hc_cc_rs::TT], &qi1[hc_cc_rs::TT]);
  }
  // case TcT
  if (hc00_reeds_shepp_->TcTExists(c1, c2))
  {
    length[hc_cc_rs::TcT] =
        hc00_reeds_shepp_->TcTPath(c1, c2, &cstart[hc_cc_rs::TcT], &cend[hc_cc_rs::TcT], &qi1[hc_cc_rs::TcT]);
  }
  // ##### Reeds-Shepp families: ############################################
  // case TcTcT
  if (hc00_reeds_shepp_->TcTcTExists(c1, c2))
  {
    length[hc_cc_rs::TcTcT] =
        hc00_reeds_shepp_->TcTcTPath(c1, c2, &cstart[hc_cc_rs::TcTcT], &cend[hc_cc_rs::TcTcT], &qi1[hc_cc_rs::TcTcT],
                                      &qi2[hc_cc_rs::TcTcT], &ci1[hc_cc_rs::TcTcT]);
  }
  // case TcTT
  if (hc00_reeds_shepp_->TcTTExists(c1, c2))
  {
    length[hc_cc_rs::TcTT] =
        hc00_reeds_shepp_->TcTTPath(c1, c2, &cstart[hc_cc_rs::TcTT], &cend[hc_cc_rs::TcTT], &qi1[hc_cc_rs::TcTT],
                                     &qi2[hc_cc_rs::TcTT], &ci1[hc_cc_rs::TcTT]);
  }
  // case TTcT
  if (hc00_reeds_shepp_->TTcTExists(c1, c2))
  {
    length[hc_cc_rs::TTcT] =
        hc00_reeds_shepp_->TTcTPath(c1, c2, &cstart[hc_cc_rs::TTcT], &cend[hc_cc_rs::TTcT], &qi1[hc_cc_rs::TTcT],
                                     &qi2[hc_cc_rs::TTcT], &ci1[hc_cc_rs::TTcT]);
  }
  // case TST
  if (hc00_reeds_shepp_->TSTExists(c1, c2))
  {
    length[hc_cc_rs::TST] = hc00_reeds_shepp_->TSTPath(c1, c2, &cstart[hc_cc_rs::TST], &cend[hc_cc_rs::TST],
                                                        &qi1[hc_cc_rs::TST], &qi2[hc_cc_rs::TST]);
  }

  // case TSTcT
  if (hc00_reeds_shepp_->TSTcTExists(c1, c2))
  {
    length[hc_cc_rs::TSTcT] =
        hc00_reeds_shepp_->TSTcTPath(c1, c2, &cstart[hc_cc_rs::TSTcT], &cend[hc_cc_rs::TSTcT], &qi1[hc_cc_rs::TSTcT],
                                      &qi2[hc_cc_rs::TSTcT], &qi3[hc_cc_rs::TSTcT], &ci1[hc_cc_rs::TSTcT]);
  }
  // case TcTST
  if (hc00_reeds_shepp_->TcTSTExists(c1, c2))
  {
    length[hc_cc_rs::TcTST] =
        hc00_reeds_shepp_->TcTSTPath(c1, c2, &cstart[hc_cc_rs::TcTST], &cend[hc_cc_rs::TcTST], &qi1[hc_cc_rs::TcTST],
                                      &qi2[hc_cc_rs::TcTST], &qi3[hc_cc_rs::TcTST], &ci1[hc_cc_rs::TcTST]);
  }
  // case TcTSTcT
  if (hc00_reeds_shepp_->TcTSTcTExists(c1, c2))
  {
    length[hc_cc_rs::TcTSTcT] = hc00_reeds_shepp_->TcTSTcTPath(
        c1, c2, &cstart[hc_cc_rs::TcTSTcT], &cend[hc_cc_rs::TcTSTcT], &qi1[hc_cc_rs::TcTSTcT], &qi2[hc_cc_rs::TcTSTcT],
        &qi3[hc_cc_rs::TcTSTcT], &qi4[hc_cc_rs::TcTSTcT], &ci1[hc_cc_rs::TcTSTcT], &ci2[hc_cc_rs::TcTSTcT]);
  }
  // case TTcTT
  if (hc00_reeds_shepp_->TTcTTExists(c1, c2))
  {
    length[hc_cc_rs::TTcTT] = hc00_reeds_shepp_->TTcTTPath(
        c1, c2, &cstart[hc_cc_rs::TTcTT], &cend[hc_cc_rs::TTcTT], &qi1[hc_cc_rs::TTcTT], &qi2[hc_cc_rs::TTcTT],
        &qi3[hc_cc_rs::TTcTT], &ci1[hc_cc_rs::TTcTT], &ci2[hc_cc_rs::TTcTT]);
  }
  // case TcTTcT
  if (hc00_reeds_shepp_->TcTTcTExists(c1, c2))
  {
    length[hc_cc_rs::TcTTcT] = hc00_reeds_shepp_->TcTTcTPath(
        c1, c2, &cstart[hc_cc_rs::TcTTcT], &cend[hc_cc_rs::TcTTcT], &qi1[hc_cc_rs::TcTTcT], &qi2[hc_cc_rs::TcTTcT],
        &qi3[hc_cc_rs::TcTTcT], &ci1[hc_cc_rs::TcTTcT], &ci2[hc_cc_rs::TcTTcT]);
  }
  // ############################################################################
  // case TTT
  if (hc00_reeds_shepp_->TTTExists(c1, c2))
  {
    length[hc_cc_rs::TTT] = hc00_reeds_shepp_->TTTPath(c1, c2, &cstart[hc_cc_rs::TTT], &cend[hc_cc_rs::TTT],
                                                        &qi1[hc_cc_rs::TTT], &qi2[hc_cc_rs::TTT], &ci1[hc_cc_rs::TTT]);
  }
  // case TcST
  if (hc00_reeds_shepp_->TcSTExists(c1, c2))
  {
    length[hc_cc_rs::TcST] = hc00_reeds_shepp_->TcSTPath(c1, c2, &cstart[hc_cc_rs::TcST], &cend[hc_cc_rs::TcST],
                                                          &qi1[hc_cc_rs::TcST], &qi2[hc_cc_rs::TcST]);
  }
  // case TScT
  if (hc00_reeds_shepp_->TScTExists(c1, c2))
  {
    length[hc_cc_rs::TScT] = hc00_reeds_shepp_->TScTPath(c1, c2, &cstart[hc_cc_rs::TScT], &cend[hc_cc_rs::TScT],
                                                          &qi1[hc_cc_rs::TScT], &qi2[hc_cc_rs::TScT]);
  }
  // case TcScT
  if (hc00_reeds_shepp_->TcScTExists(c1, c2))
  {
    length[hc_cc_rs::TcScT] = hc00_reeds_shepp_->TcScTPath(c1, c2, &cstart[hc_cc_rs::TcScT], &cend[hc_cc_rs::TcScT],
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

HcCcRsPath *Hc00ReedsSheppStateSpace::Hc00ReedsShepp(const State &state1, const State &state2) const
{
  // compute the 4 circles at the intial and final configuration
  Configuration start(state1.x, state1.y, state1.theta, 0.0);
  Configuration end(state2.x, state2.y, state2.theta, 0.0);

  HcCcCircle *start_circle[4];
  HcCcCircle *end_circle[4];
  start_circle[0] = new HcCcCircle(start, true, true, true, hc_cc_circle_param_);
  start_circle[1] = new HcCcCircle(start, false, true, true, hc_cc_circle_param_);
  start_circle[2] = new HcCcCircle(start, true, false, true, hc_cc_circle_param_);
  start_circle[3] = new HcCcCircle(start, false, false, true, hc_cc_circle_param_);
  end_circle[0] = new HcCcCircle(end, true, true, true, hc_cc_circle_param_);
  end_circle[1] = new HcCcCircle(end, false, true, true, hc_cc_circle_param_);
  end_circle[2] = new HcCcCircle(end, true, false, true, hc_cc_circle_param_);
  end_circle[3] = new HcCcCircle(end, false, false, true, hc_cc_circle_param_);

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
    for (int j = 0; j < 4; j++)
    {
      path[4 * i + j] = Hc00CirclesRsPath(*start_circle[i], *end_circle[j]);
      if (path[4 * i + j])
      {
        lg[4 * i + j] = path[4 * i + j]->length;
      }
    }
  }

  // select shortest path
  int best_path = static_cast<int>(std::min_element(lg, (lg) + (16)) - (lg));

  //  // display calculations
  //  cout << "Hc00ReedsSheppStateSpace" << endl;
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

double Hc00ReedsSheppStateSpace::GetDistance(const State &state1, const State &state2) const
{
  HcCcRsPath *p = this->Hc00ReedsShepp(state1, state2);
  double length = p->length;
  delete p;
  return length;
}

vector<Control> Hc00ReedsSheppStateSpace::GetControls(const State &state1, const State &state2) const
{
  vector<Control> hc_rs_controls;
  hc_rs_controls.reserve(10);
  HcCcRsPath *p = this->Hc00ReedsShepp(state1, state2);
  switch (p->type)
  {
    case hc_cc_rs::E:
      EmptyControls(hc_rs_controls);
      break;
    case hc_cc_rs::S:
      StraightControls(p->start, p->end, hc_rs_controls);
      break;
    case hc_cc_rs::T:
      CcTurnControls(*(p->cstart), p->end, true, hc_rs_controls);
      break;
    case hc_cc_rs::TT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi1), false, hc_rs_controls);
      break;
    case hc_cc_rs::TcT:
      HcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi1), false, hc_rs_controls);
      break;
    // ##### Reeds-Shepp families: ############################################
    case hc_cc_rs::TcTcT:
      HcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      RsTurnControls(*(p->ci1), *(p->qi2), true, hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
      break;
    case hc_cc_rs::TcTT:
      HcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi1), false, hc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
      break;
    case hc_cc_rs::TTcT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi2), true, hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
      break;
    case hc_cc_rs::TST:
      CcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      StraightControls(*(p->qi1), *(p->qi2), hc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
      break;
    case hc_cc_rs::TSTcT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      StraightControls(*(p->qi1), *(p->qi2), hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi3), true, hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi3), false, hc_rs_controls);
      break;
    case hc_cc_rs::TcTST:
      HcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi1), false, hc_rs_controls);
      StraightControls(*(p->qi2), *(p->qi3), hc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi3), false, hc_rs_controls);
      break;
    case hc_cc_rs::TcTSTcT:
      HcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi1), false, hc_rs_controls);
      StraightControls(*(p->qi2), *(p->qi3), hc_rs_controls);
      HcTurnControls(*(p->ci2), *(p->qi4), true, hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi4), false, hc_rs_controls);
      break;
    case hc_cc_rs::TTcTT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi2), true, hc_rs_controls);
      HcTurnControls(*(p->ci2), *(p->qi2), false, hc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi3), false, hc_rs_controls);
      break;
    case hc_cc_rs::TcTTcT:
      HcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      HcTurnControls(*(p->ci1), *(p->qi1), false, hc_rs_controls);
      HcTurnControls(*(p->ci2), *(p->qi3), true, hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi3), false, hc_rs_controls);
      break;
    // ########################################################################
    case hc_cc_rs::TTT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      CcTurnControls(*(p->ci1), *(p->qi2), true, hc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
      break;
    case hc_cc_rs::TcST:
      HcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      StraightControls(*(p->qi1), *(p->qi2), hc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
      break;
    case hc_cc_rs::TScT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      StraightControls(*(p->qi1), *(p->qi2), hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
      break;
    case hc_cc_rs::TcScT:
      HcTurnControls(*(p->cstart), *(p->qi1), true, hc_rs_controls);
      StraightControls(*(p->qi1), *(p->qi2), hc_rs_controls);
      HcTurnControls(*(p->cend), *(p->qi2), false, hc_rs_controls);
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
