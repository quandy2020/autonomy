#pragma once

#include "autonomy/localization/atlas/type.hpp"

#include <memory>

namespace autonomy::localization::atlas::data {
class landmark_line;
}

namespace autonomy::localization::atlas::optimize {

Mat33_t line_skew(const Vec3_t& t);

bool line_endpoint_trimming(const std::shared_ptr<data::landmark_line>& lm_line,
                            const Vec6_t& pluecker_coord,
                            Vec6_t& updated_pose_w);

Vec6_t transform_pluecker_with_sim3(const Vec6_t& pluecker,
                                    const Mat33_t& rot_cw,
                                    const Vec3_t& trans_cw,
                                    double scale_cw);

}  // namespace autonomy::localization::atlas::optimize
