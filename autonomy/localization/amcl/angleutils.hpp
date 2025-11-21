/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <cmath>

namespace autonomy {
namespace localization {
namespace amcl {

/*
 * @class angleutils
 * @brief Some utilities for working with angles
 */
class angleutils
{
 public:
   /*
    * @brief Normalize angles
    * @brief z Angle to normalize
    * @return normalized angle
    */
   static double normalize(double z);
 
   /*
    * @brief Find minimum distance between 2 angles
    * @brief a Angle 1
    * @brief b Angle 2
    * @return normalized angle difference
    */
   static double angle_diff(double a, double b);
};
 
inline double angleutils::normalize(double z)
{
   return atan2(sin(z), cos(z));
}
 
inline double angleutils::angle_diff(double a, double b)
{
    a = normalize(a);
    b = normalize(b);
    double d1 = a - b;
    double d2 = 2 * M_PI - fabs(d1);
    if (d1 > 0) {
        d2 *= -1.0;
    }
    if (fabs(d1) < fabs(d2)) {
        return d1;
    } else {
        return d2;
    }
}


}   // namespace amcl
}   // namespace localization
}   // namespace autonomy