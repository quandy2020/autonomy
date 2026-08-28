#include "autonomy/localization/atlas/camera/base.hpp"
#include "autonomy/localization/atlas/data/frame.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/landmark_line.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/match/bow_tree.hpp"
#include "autonomy/localization/atlas/match/projection.hpp"
#include "autonomy/localization/atlas/match/robust.hpp"
#include "autonomy/localization/atlas/module/frame_tracker.hpp"
#include "autonomy/localization/atlas/optimize/pose_optimizer_extended_line.hpp"
#include "autonomy/localization/atlas/optimize/pose_optimizer_g2o.hpp"
#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas::module {

frame_tracker::frame_tracker(camera::base* camera, const std::shared_ptr<optimize::pose_optimizer>& pose_optimizer,
                             const unsigned int num_matches_thr, const bool use_fixed_seed, const float margin)
    : camera_(camera), num_matches_thr_(num_matches_thr), use_fixed_seed_(use_fixed_seed), margin_(margin),
      pose_optimizer_(pose_optimizer) {}

void frame_tracker::set_line_tracking(
    data::map_database* map_db,
    const std::shared_ptr<optimize::pose_optimizer_extended_line>& pose_optimizer_extended_line) {
    map_db_ = map_db;
    pose_optimizer_extended_line_ = pose_optimizer_extended_line;
}

unsigned int frame_tracker::discard_outliers_line(data::frame& curr_frm) const {
    unsigned int num_valid_matches = 0;
    for (unsigned int idx = 0; idx < curr_frm.line_obs_.size(); ++idx) {
        if (idx < curr_frm.outlier_flags_line_.size() && curr_frm.outlier_flags_line_.at(idx)) {
            curr_frm.erase_landmark_line_with_index(idx);
            continue;
        }
        if (curr_frm.get_landmark_line(idx)) {
            ++num_valid_matches;
        }
    }
    return num_valid_matches;
}

bool frame_tracker::motion_based_track(data::frame& curr_frm, const data::frame& last_frm, const Mat44_t& velocity) const {
    match::projection projection_matcher(0.9, true);

    curr_frm.set_pose_cw(velocity * last_frm.get_pose_cw());
    curr_frm.erase_landmarks();

    auto num_matches = projection_matcher.match_current_and_last_frames(curr_frm, last_frm, margin_);

    if (num_matches < num_matches_thr_) {
        curr_frm.erase_landmarks();
        num_matches = projection_matcher.match_current_and_last_frames(curr_frm, last_frm, 2 * margin_);
    }

    if (num_matches < num_matches_thr_) {
        ADEBUG << "motion based tracking failed: " << num_matches << " matches < " << num_matches_thr_;
        return false;
    }

    Mat44_t optimized_pose;
    std::vector<bool> outlier_flags;
    std::vector<bool> outlier_flags_line;

    if (map_db_ && map_db_->use_line_tracking() && pose_optimizer_extended_line_ && !curr_frm.line_obs_.empty()) {
        curr_frm.erase_landmarks_line();
        projection_matcher.match_current_and_last_frames_line(curr_frm, last_frm, margin_);
        const auto num_point_inliers = pose_optimizer_extended_line_->optimize(curr_frm, optimized_pose, outlier_flags, outlier_flags_line);
        if (num_point_inliers >= 5) {
            curr_frm.set_pose_cw(optimized_pose);
            curr_frm.outlier_flags_line_ = std::move(outlier_flags_line);
        } else {
            pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
            curr_frm.set_pose_cw(optimized_pose);
        }
    } else {
        pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
        curr_frm.set_pose_cw(optimized_pose);
    }

    const auto num_valid_matches = discard_outliers(outlier_flags, curr_frm);
    if (map_db_ && map_db_->use_line_tracking() && pose_optimizer_extended_line_) {
        discard_outliers_line(curr_frm);
    }

    if (num_valid_matches < num_matches_thr_) {
        ADEBUG << "motion based tracking failed: " << num_valid_matches << " inlier matches < " << num_matches_thr_;
        return false;
    }
    return true;
}

bool frame_tracker::bow_match_based_track(data::frame& curr_frm, const data::frame& last_frm,
                                          const std::shared_ptr<data::keyframe>& ref_keyfrm) const {
    match::bow_tree bow_matcher(0.7, true);

    std::vector<std::shared_ptr<data::landmark>> matched_lms_in_curr;
    auto num_matches = bow_matcher.match_frame_and_keyframe(ref_keyfrm, curr_frm, matched_lms_in_curr);

    if (num_matches < num_matches_thr_) {
        ADEBUG << "bow match based tracking failed: " << num_matches << " matches < " << num_matches_thr_;
        return false;
    }

    curr_frm.set_landmarks(matched_lms_in_curr);
    curr_frm.set_pose_cw(last_frm.get_pose_cw());

    Mat44_t optimized_pose;
    std::vector<bool> outlier_flags;
    std::vector<bool> outlier_flags_line;

    if (map_db_ && map_db_->use_line_tracking() && pose_optimizer_extended_line_ && !curr_frm.line_obs_.empty()) {
        curr_frm.erase_landmarks_line();
        match::projection projection_matcher(0.9, true);
        projection_matcher.match_current_and_last_frames_line(curr_frm, last_frm, margin_);
        const auto num_point_inliers = pose_optimizer_extended_line_->optimize(curr_frm, optimized_pose, outlier_flags, outlier_flags_line);
        if (num_point_inliers >= 5) {
            curr_frm.set_pose_cw(optimized_pose);
            curr_frm.outlier_flags_line_ = std::move(outlier_flags_line);
        } else {
            pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
            curr_frm.set_pose_cw(optimized_pose);
        }
    } else {
        pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
        curr_frm.set_pose_cw(optimized_pose);
    }

    const auto num_valid_matches = discard_outliers(outlier_flags, curr_frm);
    if (map_db_ && map_db_->use_line_tracking() && pose_optimizer_extended_line_) {
        discard_outliers_line(curr_frm);
    }

    if (num_valid_matches < num_matches_thr_) {
        ADEBUG << "bow match based tracking failed: " << num_valid_matches << " inlier matches < " << num_matches_thr_;
        return false;
    }
    return true;
}

bool frame_tracker::robust_match_based_track(data::frame& curr_frm, const data::frame& last_frm,
                                             const std::shared_ptr<data::keyframe>& ref_keyfrm) const {
    match::robust robust_matcher(0.8, true);

    std::vector<std::shared_ptr<data::landmark>> matched_lms_in_curr;
    auto num_matches = robust_matcher.match_frame_and_keyframe(curr_frm, ref_keyfrm, matched_lms_in_curr, use_fixed_seed_);

    if (num_matches < num_matches_thr_) {
        ADEBUG << "robust match based tracking failed: " << num_matches << " matches < " << num_matches_thr_;
        return false;
    }

    curr_frm.set_landmarks(matched_lms_in_curr);
    curr_frm.set_pose_cw(last_frm.get_pose_cw());

    Mat44_t optimized_pose;
    std::vector<bool> outlier_flags;
    std::vector<bool> outlier_flags_line;

    if (map_db_ && map_db_->use_line_tracking() && pose_optimizer_extended_line_ && !curr_frm.line_obs_.empty()) {
        curr_frm.erase_landmarks_line();
        match::projection projection_matcher(0.9, true);
        projection_matcher.match_current_and_last_frames_line(curr_frm, last_frm, margin_);
        const auto num_point_inliers = pose_optimizer_extended_line_->optimize(curr_frm, optimized_pose, outlier_flags, outlier_flags_line);
        if (num_point_inliers >= 5) {
            curr_frm.set_pose_cw(optimized_pose);
            curr_frm.outlier_flags_line_ = std::move(outlier_flags_line);
        } else {
            pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
            curr_frm.set_pose_cw(optimized_pose);
        }
    } else {
        pose_optimizer_->optimize(curr_frm, optimized_pose, outlier_flags);
        curr_frm.set_pose_cw(optimized_pose);
    }

    const auto num_valid_matches = discard_outliers(outlier_flags, curr_frm);
    if (map_db_ && map_db_->use_line_tracking() && pose_optimizer_extended_line_) {
        discard_outliers_line(curr_frm);
    }

    if (num_valid_matches < num_matches_thr_) {
        ADEBUG << "robust match based tracking failed: " << num_valid_matches << " inlier matches < " << num_matches_thr_;
        return false;
    }
    return true;
}

unsigned int frame_tracker::discard_outliers(const std::vector<bool>& outlier_flags, data::frame& curr_frm) const {
    unsigned int num_valid_matches = 0;
    for (unsigned int idx = 0; idx < curr_frm.frm_obs_.undist_keypts_.size(); ++idx) {
        if (outlier_flags.at(idx)) {
            curr_frm.erase_landmark_with_index(idx);
            continue;
        }
        if (curr_frm.get_landmark(idx)) {
            ++num_valid_matches;
        }
    }
    return num_valid_matches;
}

}  // namespace autonomy::localization::atlas::module
