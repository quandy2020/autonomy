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

#include "autonomy/localization/amcl/motion_model/motion_model.hpp"
#include "autonomy/localization/amcl/pf/pf.hpp"
#include "autonomy/localization/amcl/pf/pf_vector.hpp"


namespace autonomy {
namespace localization {
namespace amcl {
namespace motion_model {

class DifferentialMotionModel : public MotionModel
{
public:
    virtual void initialize(
        double alpha1, double alpha2, double alpha3, double alpha4,
        double alpha5);
    virtual void odometryUpdate(pf::pf_t* pf, const pf::pf_vector_t& pose, const pf::pf_vector_t& delta);

private:
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
};

}   // namespace motion_model
}   // namespace amcl
}   // namespace localization
}   // namespace autonomy