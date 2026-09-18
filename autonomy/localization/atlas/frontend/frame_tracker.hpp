#pragma once

#include "autonomy/localization/atlas/type.hpp"
#include "autonomy/localization/atlas/optimize/pose_optimizer.hpp"

#include <memory>

namespace autonomy::localization::atlas {

namespace camera {
class base;
}  // namespace camera

namespace data {
class frame;
class keyframe;
class map_database;
}  // namespace data

namespace optimize {
class pose_optimizer_extended_line;
}  // namespace optimize

namespace module {

class frame_tracker {
public:
    explicit frame_tracker(camera::base* camera,
                           const std::shared_ptr<optimize::pose_optimizer>& pose_optimizer,
                           unsigned int num_matches_thr = 20,
                           bool use_fixed_seed = false,
                           float margin = 20.0);

    void set_line_tracking(data::map_database* map_db,
                           const std::shared_ptr<optimize::pose_optimizer_extended_line>& pose_optimizer_extended_line);

    bool motion_based_track(data::frame& curr_frm, const data::frame& last_frm, const Mat44_t& velocity) const;

    bool bow_match_based_track(data::frame& curr_frm, const data::frame& last_frm,
                               const std::shared_ptr<data::keyframe>& ref_keyfrm) const;

    bool robust_match_based_track(data::frame& curr_frm, const data::frame& last_frm,
                                  const std::shared_ptr<data::keyframe>& ref_keyfrm) const;

private:
    unsigned int discard_outliers(const std::vector<bool>& outlier_flags, data::frame& curr_frm) const;
    unsigned int discard_outliers_line(data::frame& curr_frm) const;

    const camera::base* camera_;
    const unsigned int num_matches_thr_;
    const bool use_fixed_seed_;
    const float margin_;

    std::shared_ptr<optimize::pose_optimizer> pose_optimizer_ = nullptr;
    data::map_database* map_db_ = nullptr;
    std::shared_ptr<optimize::pose_optimizer_extended_line> pose_optimizer_extended_line_;
};

}  // namespace module
}  // namespace autonomy::localization::atlas
