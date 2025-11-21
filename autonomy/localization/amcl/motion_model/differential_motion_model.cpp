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

#include "autonomy/localization/amcl/motion_model/differential_motion_model.hpp"

#include "autonomy/localization/amcl/angleutils.hpp"
#include "autonomy/localization/amcl/pf/pf_pdf.hpp"

namespace autonomy {
namespace localization {
namespace amcl {
namespace motion_model {

// void DifferentialMotionModel::initialize(
//     double alpha1, double alpha2, double alpha3, double alpha4,
//     double alpha5)
// {
//     alpha1_ = alpha1;
//     alpha2_ = alpha2;
//     alpha3_ = alpha3;
//     alpha4_ = alpha4;
//     alpha5_ = alpha5;
// }

// void DifferentialMotionModel::odometryUpdate(pf::pf_t* pf, const pf::pf_vector_t& pose,
//     const pf::pf_vector_t& delta)
// {
//     // Compute the new sample poses
//     pf::pf_sample_set_t * set;

//     set = pf->sets + pf->current_set;
//     pf::pf_vector_t old_pose = pf::pf_vector_sub(pose, delta);

//     // Implement sample_motion_odometry (Prob Rob p 136)
//     double delta_rot1, delta_trans, delta_rot2;
//     double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
//     double delta_rot1_noise, delta_rot2_noise;

//     // Avoid computing a bearing from two poses that are extremely near each
//     // other (happens on in-place rotation).
//     if (sqrt(
//         delta.v[1] * delta.v[1] +
//         delta.v[0] * delta.v[0]) < 0.01)
//     {
//         delta_rot1 = 0.0;
//     } else {
//         delta_rot1 = angleutils::angle_diff(atan2(delta.v[1], delta.v[0]), old_pose.v[2]);
//     }
//     delta_trans = sqrt(delta.v[0] * delta.v[0] + delta.v[1] * delta.v[1]);
//     delta_rot2 = angleutils::angle_diff(delta.v[2], delta_rot1);

//     // We want to treat backward and forward motion symmetrically for the
//     // noise model to be applied below.  The standard model seems to assume
//     // forward motion.
//     delta_rot1_noise = std::min(
//         fabs(angleutils::angle_diff(delta_rot1, 0.0)),
//         fabs(angleutils::angle_diff(delta_rot1, M_PI)));
//     delta_rot2_noise = std::min(
//         fabs(angleutils::angle_diff(delta_rot2, 0.0)),
//         fabs(angleutils::angle_diff(delta_rot2, M_PI)));

//     for (int i = 0; i < set->sample_count; i++) {
//         pf::pf_sample_t * sample = set->samples + i;

//         // Sample pose differences
//         delta_rot1_hat = angleutils::angle_diff(
//             delta_rot1,
//             pf::pf_ran_gaussian(sqrt(alpha1_ * delta_rot1_noise * delta_rot1_noise +
//                 alpha2_ * delta_trans * delta_trans)));
//         delta_trans_hat = delta_trans -
//             pf::pf_ran_gaussian(sqrt(alpha3_ * delta_trans * delta_trans +
//                 alpha4_ * delta_rot1_noise * delta_rot1_noise +
//                 alpha4_ * delta_rot2_noise * delta_rot2_noise));
//         delta_rot2_hat = angleutils::angle_diff(delta_rot2,
//             pf::pf_ran_gaussian(sqrt(alpha1_ * delta_rot2_noise * delta_rot2_noise +
//                 alpha2_ * delta_trans * delta_trans)));

//         // Apply sampled update to particle pose
//         sample->pose.v[0] += delta_trans_hat * cos(sample->pose.v[2] + delta_rot1_hat);
//         sample->pose.v[1] += delta_trans_hat * sin(sample->pose.v[2] + delta_rot1_hat);
//         sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
//     }
// }

    
}   // namespace motion_model
}   // namespace amcl
}   // namespace localization
}   // namespace autonomy