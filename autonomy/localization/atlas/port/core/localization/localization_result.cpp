//
// Created by xiang on 24-4-11.
//

#include "localization_result.hpp"

namespace atlas_lio::loc {

NavState LocalizationResult::ToNavState() const {
    NavState ret;
    ret.timestamp_ = timestamp_;
    ret.confidence_ = confidence_;
    ret.pos_ = pose_.translation();
    ret.rot_ = pose_.so3();
    ret.pose_is_ok_ = status_ == LocalizationStatus::GOOD;
    ret.vel_ = (pose_.so3() * vel_b_);
    return ret;
}

}  // namespace atlas_lio::loc
