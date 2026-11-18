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
 * @file reeds_shepp.cpp
 * @brief Continuous-curvature Reeds–Shepp with zero curvature at both ends (implementation).
 */

#include <cmath>
#include <limits>

#include "autonomy/common/math/steering/clothoid/reeds_shepp.hpp"
#include "autonomy/common/math/steering/clothoid/configuration.hpp"
#include "autonomy/common/math/steering/utilities.hpp"
#include <algorithm>

constexpr bool kCcRegular = false;

using namespace std;

namespace autonomy {
namespace common {
namespace math {
namespace steering {


class Cc00ReedsSheppStateSpace::Impl
{
private:
  Cc00ReedsSheppStateSpace *parent_;

public:
  explicit Impl(Cc00ReedsSheppStateSpace *parent)
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

  double TTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q) const
  {
    TT_tangent_circles(c1, c2, q);
    return c1.CcTurnLength(**q) + c2.CcTurnLength(**q);
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
    return fabs(distance - 2 * c1.radius * c1.cos_mu) < GetEpsilon();
  }

  void TcT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q) const
  {
    double distance = CenterDistance(c1, c2);
    double delta_x = 0.5 * distance;
    double delta_y = sqrt(pow(c1.radius, 2) - pow(delta_x, 2));
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
    *q = new Configuration(x, y, theta, 0);
  }

  double TcTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q) const
  {
    TcT_tangent_circles(c1, c2, q);
    return c1.CcTurnLength(**q) + c2.CcTurnLength(**q);
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
    return distance <= 4 * c1.radius * c1.cos_mu;
  }

  void TcTcT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                             Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r = 2 * c1.radius * c1.cos_mu;
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

  double TcTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                    HcCcCircle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TcTcT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    HcCcCircle *middle1, *middle2;
    middle1 = new HcCcCircle(*qa, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qc, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    // select shortest connection
    double length1 = c1.CcTurnLength(*qa) + middle1->CcTurnLength(*qb) + c2.CcTurnLength(*qb);
    double length2 = c1.CcTurnLength(*qc) + middle2->CcTurnLength(*qd) + c2.CcTurnLength(*qd);
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
    return (distance <= 2 * c1.radius * (1 + c1.cos_mu)) && (distance >= 2 * c1.radius * (1 - c1.cos_mu));
  }

  void TcTT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                            Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r1 = 2 * c1.radius * c1.cos_mu;
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

  double TcTTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                   HcCcCircle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TcTT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    HcCcCircle *middle1, *middle2;
    middle1 = new HcCcCircle(*qa, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qc, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    // select shortest connection
    double length1 = c1.CcTurnLength(*qa) + middle1->CcTurnLength(*qb) + c2.CcTurnLength(*qb);
    double length2 = c1.CcTurnLength(*qc) + middle2->CcTurnLength(*qd) + c2.CcTurnLength(*qd);
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
    return (distance <= 2 * c1.radius * (1 + c1.cos_mu)) && (distance >= 2 * c1.radius * (1 - c1.cos_mu));
  }

  void TTcT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                            Configuration **q3, Configuration **q4) const
  {
    double theta = angle;
    double r1 = 2 * c1.radius;
    double r2 = 2 * c1.radius * c1.cos_mu;
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

  double TTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                   HcCcCircle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TTcT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    HcCcCircle *middle1, *middle2;
    middle1 = new HcCcCircle(*qa, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qc, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    // select shortest connection
    double length1 = c1.CcTurnLength(*qa) + middle1->CcTurnLength(*qb) + c2.CcTurnLength(*qb);
    double length2 = c1.CcTurnLength(*qc) + middle2->CcTurnLength(*qd) + c2.CcTurnLength(*qd);
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

  double TiSTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    TiST_tangent_circles(c1, c2, q1, q2);
    return c1.CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + c2.CcTurnLength(**q2);
  }

  double TeSTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    TeST_tangent_circles(c1, c2, q1, q2);
    return c1.CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + c2.CcTurnLength(**q2);
  }

  double TSTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    if (TiSTExists(c1, c2))
    {
      return TiSTPath(c1, c2, q1, q2);
    }
    if (TeSTExists(c1, c2))
    {
      return TeSTPath(c1, c2, q1, q2);
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
    return (distance >= 2 * c1.radius * sqrt(1 + 2 * c1.sin_mu * c1.cos_mu + pow(c1.cos_mu, 2)));
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
    return (distance >= 2 * c1.radius * (c1.cos_mu + c1.sin_mu));
  }

  bool TSTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TiSTcTExists(c1, c2) || TeSTcTExists(c1, c2);
  }

  double TiSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                     Configuration **q3, HcCcCircle **ci) const
  {
    double theta = angle;
    double r = c2.radius * c2.cos_mu;
    double delta_y = pow(2 * r, 2) / distance;
    double delta_x = 2 * r * sqrt(1 - delta_y / distance);
    double x, y;

    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TiST_tangent_circles(c1, tgt1, q1, q2);
    TcT_tangent_circles(tgt1, c2, q3);

    *ci = new HcCcCircle(**q2, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    return c1.CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*ci)->CcTurnLength(**q3) +
           c2.CcTurnLength(**q3);
  }

  double TeSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                     Configuration **q3, HcCcCircle **ci) const
  {
    double theta = angle;
    double delta_x = 2 * c2.radius * c2.cos_mu;
    double delta_y = 0;
    double x, y;

    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TeST_tangent_circles(c1, tgt1, q1, q2);
    TcT_tangent_circles(tgt1, c2, q3);

    *ci = new HcCcCircle(**q2, c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    return c1.CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + (*ci)->CcTurnLength(**q3) +
           c2.CcTurnLength(**q3);
  }

  double TSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                    Configuration **q3, HcCcCircle **ci) const
  {
    if (TiSTcTExists(c1, c2))
    {
      return TiSTcTPath(c1, c2, q1, q2, q3, ci);
    }
    if (TeSTcTExists(c1, c2))
    {
      return TeSTcTPath(c1, c2, q1, q2, q3, ci);
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
    return (distance >= 2 * c1.radius * sqrt(1 + 2 * c1.sin_mu * c1.cos_mu + pow(c1.cos_mu, 2)));
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
    return (distance >= 2 * c1.radius * (c1.cos_mu + c1.sin_mu));
  }

  bool TcTSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TcTiSTExists(c1, c2) || TcTeSTExists(c1, c2);
  }

  double TcTiSTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                     Configuration **q3, HcCcCircle **ci) const
  {
    double theta = angle;
    double r = c1.radius * c1.cos_mu;
    double delta_y = pow(2 * r, 2) / distance;
    double delta_x = 2 * r * sqrt(1 - delta_y / distance);
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c2.left, !c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TiST_tangent_circles(tgt1, c2, q2, q3);

    *ci = new HcCcCircle(**q1, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    return c1.CcTurnLength(**q1) + (*ci)->CcTurnLength(**q2) + ConfigurationDistance(**q2, **q3) +
           c2.CcTurnLength(**q3);
  }

  double TcTeSTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                     Configuration **q3, HcCcCircle **ci) const
  {
    double theta = angle;
    double delta_x = 2 * c2.radius * c2.cos_mu;
    double delta_y = 0;
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, c2.left, !c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TeST_tangent_circles(tgt1, c2, q2, q3);

    *ci = new HcCcCircle(**q1, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    return c1.CcTurnLength(**q1) + (*ci)->CcTurnLength(**q2) + ConfigurationDistance(**q2, **q3) +
           c2.CcTurnLength(**q3);
  }

  double TcTSTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                    Configuration **q3, HcCcCircle **ci) const
  {
    if (TcTiSTExists(c1, c2))
    {
      return TcTiSTPath(c1, c2, q1, q2, q3, ci);
    }
    if (TcTeSTExists(c1, c2))
    {
      return TcTeSTPath(c1, c2, q1, q2, q3, ci);
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
    return (distance >= 2 * c1.radius * sqrt(1 + 4 * c1.cos_mu * c1.sin_mu + 4 * pow(c1.cos_mu, 2)));
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
    return (distance >= 2 * c1.radius * (2 * c1.cos_mu + c1.sin_mu));
  }

  bool TcTSTcTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TcTiSTcTExists(c1, c2) || TcTeSTcTExists(c1, c2);
  }

  double TcTiSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                       Configuration **q3, Configuration **q4, HcCcCircle **ci1, HcCcCircle **ci2) const
  {
    double theta = angle;
    double r = c1.radius * c1.cos_mu;
    double delta_y = pow(2 * r, 2) / distance;
    double delta_x = 2 * r * sqrt(1 - delta_y / distance);
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
    HcCcCircle tgt2(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TiST_tangent_circles(tgt1, tgt2, q2, q3);
    TcT_tangent_circles(tgt2, c2, q4);

    *ci1 = new HcCcCircle(**q1, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    *ci2 = new HcCcCircle(**q3, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    return c1.CcTurnLength(**q1) + (*ci1)->CcTurnLength(**q2) + ConfigurationDistance(**q2, **q3) +
           (*ci2)->CcTurnLength(**q4) + c2.CcTurnLength(**q4);
  }

  double TcTeSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                       Configuration **q3, Configuration **q4, HcCcCircle **ci1, HcCcCircle **ci2) const
  {
    double theta = angle;
    double delta_x = 2 * c1.radius * c1.cos_mu;
    double delta_y = 0;
    double x, y;

    GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
    HcCcCircle tgt1(x, y, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
    HcCcCircle tgt2(x, y, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    TcT_tangent_circles(c1, tgt1, q1);
    TeST_tangent_circles(tgt1, tgt2, q2, q3);
    TcT_tangent_circles(tgt2, c2, q4);

    *ci1 = new HcCcCircle(**q1, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    *ci2 = new HcCcCircle(**q3, !c2.left, c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    return c1.CcTurnLength(**q1) + (*ci1)->CcTurnLength(**q2) + ConfigurationDistance(**q2, **q3) +
           (*ci2)->CcTurnLength(**q4) + c2.CcTurnLength(**q4);
  }

  double TcTSTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                      Configuration **q3, Configuration **q4, HcCcCircle **ci1, HcCcCircle **ci2) const
  {
    if (TcTiSTcTExists(c1, c2))
    {
      return TcTiSTcTPath(c1, c2, q1, q2, q3, q4, ci1, ci2);
    }
    if (TcTeSTcTExists(c1, c2))
    {
      return TcTeSTcTPath(c1, c2, q1, q2, q3, q4, ci1, ci2);
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
    return (distance <= 2 * c1.radius * (c1.cos_mu + 2));
  }

  void TTcTT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                             Configuration **q3, Configuration **q4, Configuration **q5, Configuration **q6) const
  {
    double theta = angle;
    double r1, r2, delta_x, delta_y, x, y;
    r1 = 2 * c1.radius * c1.cos_mu;
    r2 = 2 * c1.radius;
    if (distance < 2 * c1.radius * (-c1.cos_mu + 2))
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

  double TTcTTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                    Configuration **q3, HcCcCircle **ci1, HcCcCircle **ci2) const
  {
    Configuration *qa, *qb, *qc, *qd, *qe, *qf;
    TTcTT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd, &qe, &qf);
    HcCcCircle *middle1, *middle2, *middle3, *middle4;
    middle1 = new HcCcCircle(*qa, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qb, !c2.left, !c2.forward, c2.regular, parent_->hc_cc_circle_param_);
    middle3 = new HcCcCircle(*qd, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    middle4 = new HcCcCircle(*qe, !c2.left, !c2.forward, c2.regular, parent_->hc_cc_circle_param_);

    // select shortest connection
    double length1 =
        c1.CcTurnLength(*qa) + middle1->CcTurnLength(*qb) + middle2->CcTurnLength(*qc) + c2.CcTurnLength(*qc);
    double length2 =
        c1.CcTurnLength(*qd) + middle3->CcTurnLength(*qe) + middle4->CcTurnLength(*qf) + c2.CcTurnLength(*qf);
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
    return (distance <= 2 * c1.radius * (2 * c1.cos_mu + 1)) && (distance >= 2 * c1.radius * (2 * c1.cos_mu - 1));
  }

  void TcTTcT_tangent_circles(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                              Configuration **q3, Configuration **q4, Configuration **q5, Configuration **q6) const
  {
    double theta = angle;
    double r1 = 2 * c1.radius * c1.cos_mu;
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

  double TcTTcTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                     Configuration **q3, HcCcCircle **ci1, HcCcCircle **ci2) const
  {
    Configuration *qa, *qb, *qc, *qd, *qe, *qf;
    TcTTcT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd, &qe, &qf);
    HcCcCircle *middle1, *middle2, *middle3, *middle4;
    middle1 = new HcCcCircle(*qa, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qb, c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    middle3 = new HcCcCircle(*qd, !c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    middle4 = new HcCcCircle(*qe, c1.left, !c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    // select shortest connection
    double length1 =
        c1.CcTurnLength(*qa) + middle1->CcTurnLength(*qb) + middle2->CcTurnLength(*qc) + c2.CcTurnLength(*qc);
    double length2 =
        c1.CcTurnLength(*qd) + middle3->CcTurnLength(*qe) + middle4->CcTurnLength(*qf) + c2.CcTurnLength(*qf);
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

  double TTTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2,
                  HcCcCircle **ci) const
  {
    Configuration *qa, *qb, *qc, *qd;
    TTT_tangent_circles(c1, c2, &qa, &qb, &qc, &qd);
    HcCcCircle *middle1, *middle2;
    middle1 = new HcCcCircle(*qa, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);
    middle2 = new HcCcCircle(*qc, !c1.left, c1.forward, c1.regular, parent_->hc_cc_circle_param_);

    // select shortest connection
    double length1 = c1.CcTurnLength(*qa) + middle1->CcTurnLength(*qb) + c2.CcTurnLength(*qb);
    double length2 = c1.CcTurnLength(*qc) + middle2->CcTurnLength(*qd) + c2.CcTurnLength(*qd);
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
    return distance >= 2 * c1.radius * c1.cos_mu;
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
    return distance >= GetEpsilon();
  }

  bool TcSTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TciSTExists(c1, c2) || TceSTExists(c1, c2);
  }

  double TciSTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    double alpha = asin(2 * c1.radius * c1.cos_mu / distance);
    double delta_x = c1.radius * c1.sin_mu;
    double delta_y = c1.radius * c1.cos_mu;
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    return c1.CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + c2.CcTurnLength(**q2);
  }

  double TceSTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    double theta = angle;
    double delta_x = c1.radius * c1.sin_mu;
    double delta_y = c1.radius * c1.cos_mu;
    double x, y;
    if (c1.left && c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (c1.left && !c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (!c1.left && !c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, -delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    return c1.CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + c2.CcTurnLength(**q2);
  }

  double TcSTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    if (TciSTExists(c1, c2))
    {
      return TciSTPath(c1, c2, q1, q2);
    }
    if (TceSTExists(c1, c2))
    {
      return TceSTPath(c1, c2, q1, q2);
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
    return distance >= 2 * c1.radius * c1.cos_mu;
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
    return distance >= GetEpsilon();
  }

  bool TScTExists(const HcCcCircle &c1, const HcCcCircle &c2) const
  {
    return TiScTExists(c1, c2) || TeScTExists(c1, c2);
  }

  double TiScTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    double alpha = asin(2 * c1.radius * c1.cos_mu / distance);
    double delta_x = c1.radius * c1.sin_mu;
    double delta_y = c1.radius * c1.cos_mu;
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    return c1.CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + c2.CcTurnLength(**q2);
  }

  double TeScTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    double theta = angle;
    double delta_x = c1.radius * c1.sin_mu;
    double delta_y = c1.radius * c1.cos_mu;
    double x, y;
    if (c1.left && c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (c1.left && !c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (!c1.left && c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && !c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    return c1.CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + c2.CcTurnLength(**q2);
  }

  double TScTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    if (TiScTExists(c1, c2))
    {
      return TiScTPath(c1, c2, q1, q2);
    }
    if (TeScTExists(c1, c2))
    {
      return TeScTPath(c1, c2, q1, q2);
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
    return distance >= 2 * c1.radius * c1.cos_mu;
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

  double TciScTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    double alpha = asin(2 * c1.radius * c1.cos_mu / distance);
    double delta_x = c1.radius * c1.sin_mu;
    double delta_y = c1.radius * c1.cos_mu;
    double x, y, theta;
    if (c1.left && c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (c1.left && !c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && c1.forward)
    {
      theta = angle + alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (!c1.left && !c1.forward)
    {
      theta = angle - alpha;
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    return c1.CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + c2.CcTurnLength(**q2);
  }

  double TceScTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    double theta = angle;
    double delta_x = c1.radius * c1.sin_mu;
    double delta_y = c1.radius * c1.cos_mu;
    double x, y;
    if (c1.left && c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (c1.left && !c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    if (!c1.left && c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, -delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta + M_PI, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, -delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta + M_PI, 0);
    }
    if (!c1.left && !c1.forward)
    {
      GlobalFrameChange(c1.xc, c1.yc, theta, -delta_x, delta_y, &x, &y);
      *q1 = new Configuration(x, y, theta, 0);
      GlobalFrameChange(c2.xc, c2.yc, theta, delta_x, delta_y, &x, &y);
      *q2 = new Configuration(x, y, theta, 0);
    }
    return c1.CcTurnLength(**q1) + ConfigurationDistance(**q1, **q2) + c2.CcTurnLength(**q2);
  }

  double TcScTPath(const HcCcCircle &c1, const HcCcCircle &c2, Configuration **q1, Configuration **q2) const
  {
    if (TciScTExists(c1, c2))
    {
      return TciScTPath(c1, c2, q1, q2);
    }
    if (TceScTExists(c1, c2))
    {
      return TceScTPath(c1, c2, q1, q2);
    }
    return numeric_limits<double>::max();
  }
};

// ############################################################################

Cc00ReedsSheppStateSpace::Cc00ReedsSheppStateSpace(double kappa, double sigma, double discretization)
  : HcCcStateSpace(kappa, sigma, discretization)
  , cc00_reeds_shepp_{ unique_ptr<Impl>(new Impl(this)) }
{
}

Cc00ReedsSheppStateSpace::~Cc00ReedsSheppStateSpace() = default;

HcCcRsPath *Cc00ReedsSheppStateSpace::Cc00CirclesRsPath(const HcCcCircle &c1, const HcCcCircle &c2) const
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
  cc00_reeds_shepp_->distance = CenterDistance(c1, c2);
  cc00_reeds_shepp_->angle = atan2(c2.yc - c1.yc, c2.xc - c1.xc);

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
    cstart[hc_cc_rs::T] = new HcCcCircle(c1);
    length[hc_cc_rs::T] = cstart[hc_cc_rs::T]->CcTurnLength(c2.start);
    goto label_end;
  }
  // case TT
  if (cc00_reeds_shepp_->TTExists(c1, c2))
  {
    cstart[hc_cc_rs::TT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TT] = new HcCcCircle(c2);
    length[hc_cc_rs::TT] = cc00_reeds_shepp_->TTPath(*cstart[hc_cc_rs::TT], *cend[hc_cc_rs::TT], &qi1[hc_cc_rs::TT]);
  }
  // case TcT
  if (cc00_reeds_shepp_->TcTExists(c1, c2))
  {
    cstart[hc_cc_rs::TcT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TcT] = new HcCcCircle(c2);
    length[hc_cc_rs::TcT] =
        cc00_reeds_shepp_->TcTPath(*cstart[hc_cc_rs::TcT], *cend[hc_cc_rs::TcT], &qi1[hc_cc_rs::TcT]);
  }
  // ##### Reeds-Shepp families: ############################################
  // case TcTcT
  if (cc00_reeds_shepp_->TcTcTExists(c1, c2))
  {
    cstart[hc_cc_rs::TcTcT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TcTcT] = new HcCcCircle(c2);
    length[hc_cc_rs::TcTcT] =
        cc00_reeds_shepp_->TcTcTPath(*cstart[hc_cc_rs::TcTcT], *cend[hc_cc_rs::TcTcT], &qi1[hc_cc_rs::TcTcT],
                                      &qi2[hc_cc_rs::TcTcT], &ci1[hc_cc_rs::TcTcT]);
  }
  // case TcTT
  if (cc00_reeds_shepp_->TcTTExists(c1, c2))
  {
    cstart[hc_cc_rs::TcTT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TcTT] = new HcCcCircle(c2);
    length[hc_cc_rs::TcTT] =
        cc00_reeds_shepp_->TcTTPath(*cstart[hc_cc_rs::TcTT], *cend[hc_cc_rs::TcTT], &qi1[hc_cc_rs::TcTT],
                                     &qi2[hc_cc_rs::TcTT], &ci1[hc_cc_rs::TcTT]);
  }
  // case TTcT
  if (cc00_reeds_shepp_->TTcTExists(c1, c2))
  {
    cstart[hc_cc_rs::TTcT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TTcT] = new HcCcCircle(c2);
    length[hc_cc_rs::TTcT] =
        cc00_reeds_shepp_->TTcTPath(*cstart[hc_cc_rs::TTcT], *cend[hc_cc_rs::TTcT], &qi1[hc_cc_rs::TTcT],
                                     &qi2[hc_cc_rs::TTcT], &ci1[hc_cc_rs::TTcT]);
  }
  // case TST
  if (cc00_reeds_shepp_->TSTExists(c1, c2))
  {
    cstart[hc_cc_rs::TST] = new HcCcCircle(c1);
    cend[hc_cc_rs::TST] = new HcCcCircle(c2);
    length[hc_cc_rs::TST] = cc00_reeds_shepp_->TSTPath(*cstart[hc_cc_rs::TST], *cend[hc_cc_rs::TST],
                                                        &qi1[hc_cc_rs::TST], &qi2[hc_cc_rs::TST]);
  }
  // case TSTcT
  if (cc00_reeds_shepp_->TSTcTExists(c1, c2))
  {
    cstart[hc_cc_rs::TSTcT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TSTcT] = new HcCcCircle(c2);
    length[hc_cc_rs::TSTcT] =
        cc00_reeds_shepp_->TSTcTPath(*cstart[hc_cc_rs::TSTcT], *cend[hc_cc_rs::TSTcT], &qi1[hc_cc_rs::TSTcT],
                                      &qi2[hc_cc_rs::TSTcT], &qi3[hc_cc_rs::TSTcT], &ci1[hc_cc_rs::TSTcT]);
  }
  // case TcTST
  if (cc00_reeds_shepp_->TcTSTExists(c1, c2))
  {
    cstart[hc_cc_rs::TcTST] = new HcCcCircle(c1);
    cend[hc_cc_rs::TcTST] = new HcCcCircle(c2);
    length[hc_cc_rs::TcTST] =
        cc00_reeds_shepp_->TcTSTPath(*cstart[hc_cc_rs::TcTST], *cend[hc_cc_rs::TcTST], &qi1[hc_cc_rs::TcTST],
                                      &qi2[hc_cc_rs::TcTST], &qi3[hc_cc_rs::TcTST], &ci1[hc_cc_rs::TcTST]);
  }
  // case TcTSTcT
  if (cc00_reeds_shepp_->TcTSTcTExists(c1, c2))
  {
    cstart[hc_cc_rs::TcTSTcT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TcTSTcT] = new HcCcCircle(c2);
    length[hc_cc_rs::TcTSTcT] = cc00_reeds_shepp_->TcTSTcTPath(
        *cstart[hc_cc_rs::TcTSTcT], *cend[hc_cc_rs::TcTSTcT], &qi1[hc_cc_rs::TcTSTcT], &qi2[hc_cc_rs::TcTSTcT],
        &qi3[hc_cc_rs::TcTSTcT], &qi4[hc_cc_rs::TcTSTcT], &ci1[hc_cc_rs::TcTSTcT], &ci2[hc_cc_rs::TcTSTcT]);
  }
  // case TTcTT
  if (cc00_reeds_shepp_->TTcTTExists(c1, c2))
  {
    cstart[hc_cc_rs::TTcTT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TTcTT] = new HcCcCircle(c2);
    length[hc_cc_rs::TTcTT] = cc00_reeds_shepp_->TTcTTPath(
        *cstart[hc_cc_rs::TTcTT], *cend[hc_cc_rs::TTcTT], &qi1[hc_cc_rs::TTcTT], &qi2[hc_cc_rs::TTcTT],
        &qi3[hc_cc_rs::TTcTT], &ci1[hc_cc_rs::TTcTT], &ci2[hc_cc_rs::TTcTT]);
  }
  // case TcTTcT
  if (cc00_reeds_shepp_->TcTTcTExists(c1, c2))
  {
    cstart[hc_cc_rs::TcTTcT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TcTTcT] = new HcCcCircle(c2);
    length[hc_cc_rs::TcTTcT] = cc00_reeds_shepp_->TcTTcTPath(
        *cstart[hc_cc_rs::TcTTcT], *cend[hc_cc_rs::TcTTcT], &qi1[hc_cc_rs::TcTTcT], &qi2[hc_cc_rs::TcTTcT],
        &qi3[hc_cc_rs::TcTTcT], &ci1[hc_cc_rs::TcTTcT], &ci2[hc_cc_rs::TcTTcT]);
  }
  // ############################################################################
  // case TTT
  if (cc00_reeds_shepp_->TTTExists(c1, c2))
  {
    cstart[hc_cc_rs::TTT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TTT] = new HcCcCircle(c2);
    length[hc_cc_rs::TTT] = cc00_reeds_shepp_->TTTPath(*cstart[hc_cc_rs::TTT], *cend[hc_cc_rs::TTT],
                                                        &qi1[hc_cc_rs::TTT], &qi2[hc_cc_rs::TTT], &ci1[hc_cc_rs::TTT]);
  }
  // case TcST
  if (cc00_reeds_shepp_->TcSTExists(c1, c2))
  {
    cstart[hc_cc_rs::TcST] = new HcCcCircle(c1);
    cend[hc_cc_rs::TcST] = new HcCcCircle(c2);
    length[hc_cc_rs::TcST] = cc00_reeds_shepp_->TcSTPath(*cstart[hc_cc_rs::TcST], *cend[hc_cc_rs::TcST],
                                                          &qi1[hc_cc_rs::TcST], &qi2[hc_cc_rs::TcST]);
  }
  // case TScT
  if (cc00_reeds_shepp_->TScTExists(c1, c2))
  {
    cstart[hc_cc_rs::TScT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TScT] = new HcCcCircle(c2);
    length[hc_cc_rs::TScT] = cc00_reeds_shepp_->TScTPath(*cstart[hc_cc_rs::TScT], *cend[hc_cc_rs::TScT],
                                                          &qi1[hc_cc_rs::TScT], &qi2[hc_cc_rs::TScT]);
  }
  // case TcScT
  if (cc00_reeds_shepp_->TcScTExists(c1, c2))
  {
    cstart[hc_cc_rs::TcScT] = new HcCcCircle(c1);
    cend[hc_cc_rs::TcScT] = new HcCcCircle(c2);
    length[hc_cc_rs::TcScT] = cc00_reeds_shepp_->TcScTPath(*cstart[hc_cc_rs::TcScT], *cend[hc_cc_rs::TcScT],
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

HcCcRsPath *Cc00ReedsSheppStateSpace::Cc00ReedsShepp(const State &state1, const State &state2) const
{
  // compute the 4 circles at the intial and final configuration
  Configuration start(state1.x, state1.y, state1.theta, 0.0);
  Configuration end(state2.x, state2.y, state2.theta, 0.0);

  HcCcCircle *start_circle[4];
  HcCcCircle *end_circle[4];
  start_circle[0] = new HcCcCircle(start, true, true, kCcRegular, hc_cc_circle_param_);
  start_circle[1] = new HcCcCircle(start, false, true, kCcRegular, hc_cc_circle_param_);
  start_circle[2] = new HcCcCircle(start, true, false, kCcRegular, hc_cc_circle_param_);
  start_circle[3] = new HcCcCircle(start, false, false, kCcRegular, hc_cc_circle_param_);
  end_circle[0] = new HcCcCircle(end, true, true, kCcRegular, hc_cc_circle_param_);
  end_circle[1] = new HcCcCircle(end, false, true, kCcRegular, hc_cc_circle_param_);
  end_circle[2] = new HcCcCircle(end, true, false, kCcRegular, hc_cc_circle_param_);
  end_circle[3] = new HcCcCircle(end, false, false, kCcRegular, hc_cc_circle_param_);

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
      path[4 * i + j] = Cc00CirclesRsPath(*start_circle[i], *end_circle[j]);
      if (path[4 * i + j])
      {
        lg[4 * i + j] = path[4 * i + j]->length;
      }
    }
  }

  // select shortest path
  int best_path = static_cast<int>(std::min_element(lg, (lg) + (16)) - (lg));

  //  // display calculations
  //  cout << "Cc00ReedsSheppStateSpace" << endl;
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

double Cc00ReedsSheppStateSpace::GetDistance(const State &state1, const State &state2) const
{
  HcCcRsPath *p = this->Cc00ReedsShepp(state1, state2);
  double length = p->length;
  delete p;
  return length;
}

vector<Control> Cc00ReedsSheppStateSpace::GetControls(const State &state1, const State &state2) const
{
  vector<Control> cc_rs_controls;
  cc_rs_controls.reserve(13);
  HcCcRsPath *p = this->Cc00ReedsShepp(state1, state2);
  switch (p->type)
  {
    case hc_cc_rs::E:
      EmptyControls(cc_rs_controls);
      break;
    case hc_cc_rs::S:
      StraightControls(p->start, p->end, cc_rs_controls);
      break;
    case hc_cc_rs::T:
      CcTurnControls(*(p->cstart), p->end, true, cc_rs_controls);
      break;
    case hc_cc_rs::TT:
    case hc_cc_rs::TcT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi1), false, cc_rs_controls);
      break;
    // ##### Reeds-Shepp families: ############################################
    case hc_cc_rs::TcTcT:
    case hc_cc_rs::TcTT:
    case hc_cc_rs::TTcT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_rs_controls);
      CcTurnControls(*(p->ci1), *(p->qi2), true, cc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi2), false, cc_rs_controls);
      break;
    case hc_cc_rs::TST:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_rs_controls);
      StraightControls(*(p->qi1), *(p->qi2), cc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi2), false, cc_rs_controls);
      break;
    case hc_cc_rs::TSTcT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_rs_controls);
      StraightControls(*(p->qi1), *(p->qi2), cc_rs_controls);
      CcTurnControls(*(p->ci1), *(p->qi3), true, cc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi3), false, cc_rs_controls);
      break;
    case hc_cc_rs::TcTST:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_rs_controls);
      CcTurnControls(*(p->ci1), *(p->qi2), true, cc_rs_controls);
      StraightControls(*(p->qi2), *(p->qi3), cc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi3), false, cc_rs_controls);
      break;
    case hc_cc_rs::TcTSTcT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_rs_controls);
      CcTurnControls(*(p->ci1), *(p->qi2), true, cc_rs_controls);
      StraightControls(*(p->qi2), *(p->qi3), cc_rs_controls);
      CcTurnControls(*(p->ci2), *(p->qi4), true, cc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi4), false, cc_rs_controls);
      break;
    case hc_cc_rs::TTcTT:
    case hc_cc_rs::TcTTcT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_rs_controls);
      CcTurnControls(*(p->ci1), *(p->qi2), true, cc_rs_controls);
      CcTurnControls(*(p->ci2), *(p->qi3), true, cc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi3), false, cc_rs_controls);
      break;
    // ########################################################################
    case hc_cc_rs::TTT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_rs_controls);
      CcTurnControls(*(p->ci1), *(p->qi2), true, cc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi2), false, cc_rs_controls);
      break;
    case hc_cc_rs::TcST:
    case hc_cc_rs::TScT:
    case hc_cc_rs::TcScT:
      CcTurnControls(*(p->cstart), *(p->qi1), true, cc_rs_controls);
      StraightControls(*(p->qi1), *(p->qi2), cc_rs_controls);
      CcTurnControls(*(p->cend), *(p->qi2), false, cc_rs_controls);
      break;
    default:
      break;
  }
  delete p;
  return cc_rs_controls;
}

}  // namespace steering
}  // namespace math
}  // namespace common
}  // namespace autonomy
