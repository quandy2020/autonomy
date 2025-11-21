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

#include "autonomy/localization/amcl/pf/pf.hpp"
#include "autonomy/localization/amcl/pf/pf_vector.hpp"

namespace autonomy {
namespace localization {
namespace amcl {
namespace motion_model {

/**
 * @class nav2_amcl::MotionModel
 * @brief An abstract motion model class
 */
class MotionModel
{
 public:
   virtual ~MotionModel() = default;
 
   /**
    * @brief An factory to create motion models
    * @param type Type of motion model to create in factory
    * @param alpha1 error parameters, see documentation
    * @param alpha2 error parameters, see documentation
    * @param alpha3 error parameters, see documentation
    * @param alpha4 error parameters, see documentation
    * @param alpha5 error parameters, see documentation
    * @return MotionModel A pointer to the motion model it created
    */
   virtual void initialize(
     double alpha1, double alpha2, double alpha3, double alpha4,
     double alpha5) = 0;
 
   /**
    * @brief Update on new odometry data
    * @param pf The particle filter to update
    * @param pose pose of robot in odometry update
    * @param delta change in pose in odometry update
    */
   virtual void odometryUpdate(pf::pf_t * pf, const pf::pf_vector_t & pose, const pf::pf_vector_t & delta) = 0;
};

}   // namespace motion_model
}   // namespace amcl
}   // namespace localization
}   // namespace autonomy