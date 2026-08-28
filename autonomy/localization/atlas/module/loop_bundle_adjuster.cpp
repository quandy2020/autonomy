#include "autonomy/localization/atlas/mapping_module.hpp"
#include "autonomy/localization/atlas/tracking_module.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/marker.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/module/loop_bundle_adjuster.hpp"
#include "autonomy/localization/atlas/data/landmark_line.hpp"
#include "autonomy/localization/atlas/optimize/global_bundle_adjuster.hpp"
#include "autonomy/localization/atlas/optimize/line_geometry_util.hpp"

#include <thread>
#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace module {

loop_bundle_adjuster::loop_bundle_adjuster(data::map_database* map_db,
                                           const unsigned int num_iter,
                                           const bool use_huber_kernel,
                                           const bool verbose)
    : map_db_(map_db),
      num_iter_(num_iter),
      use_huber_kernel_(use_huber_kernel),
      verbose_(verbose) {}

void loop_bundle_adjuster::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
}

void loop_bundle_adjuster::set_tracking_module(tracking_module* tracker) {
    tracker_ = tracker;
}

void loop_bundle_adjuster::abort() {
    std::lock_guard<std::mutex> lock(mtx_thread_);
    abort_loop_BA_ = true;
}

bool loop_bundle_adjuster::is_running() const {
    std::lock_guard<std::mutex> lock(mtx_thread_);
    return loop_BA_is_running_;
}

void loop_bundle_adjuster::optimize(const std::shared_ptr<data::keyframe>& curr_keyfrm) {
    AINFO << "start loop bundle adjustment";

    {
        std::lock_guard<std::mutex> lock(mtx_thread_);
        loop_BA_is_running_ = true;
        abort_loop_BA_ = false;
    }

    struct resume_mapping_guard {
        mapping_module* mapper = nullptr;
        bool active = false;
        ~resume_mapping_guard() {
            if (active && mapper != nullptr) {
                mapper->resume();
            }
        }
    } resume_guard;
    resume_guard.mapper = mapper_;
    resume_guard.active = (mapper_ != nullptr);

    struct resume_tracker_guard {
        tracking_module* tracker = nullptr;
        bool active = false;
        ~resume_tracker_guard() {
            if (active && tracker != nullptr) {
                tracker->resume();
            }
        }
    } resume_tracker;
    resume_tracker.tracker = tracker_;

    // Pause tracking before locking the map (mapper is already paused by correct_loop).
    if (tracker_ != nullptr) {
        auto future_pause = tracker_->async_pause();
        tracker_->wait_until_track_idle();
        future_pause.get();
        resume_tracker.active = true;
        AINFO << "loop bundle adjustment: tracking paused, map idle";
    }

    std::unordered_set<unsigned int> optimized_keyfrm_ids;
    std::unordered_set<unsigned int> optimized_landmark_ids;
    std::unordered_set<unsigned int> optimized_landmark_line_ids;
    std::unordered_set<unsigned int> optimized_marker_ids;
    eigen_alloc_unord_map<unsigned int, Vec3_t> lm_to_pos_w_after_global_BA;
    eigen_alloc_unord_map<unsigned int, Vec6_t> lm_line_to_plucker_after_global_BA;
    eigen_alloc_unord_map<unsigned int, Mat44_t> keyfrm_to_pose_cw_after_global_BA;
    eigen_alloc_unord_map<unsigned int, std::array<Vec3_t, 4>> marker_to_pos_w_after_global_BA;

    bool ok = false;
    {
        // Mapping is paused by global_optimization_module::correct_loop. Hold the map lock
        // while reading landmarks/planes for GBA and applying pose/landmark updates.
        std::lock_guard<std::mutex> lock_db(data::map_database::mtx_database_);

        const auto keyfrms = curr_keyfrm->graph_node_->get_keyframes_from_root();
        AINFO << "loop bundle adjustment: global BA on " << keyfrms.size() << " keyframes"
              << (map_db_->use_line_tracking() ? " (with line landmarks)" : "");
        const auto global_BA = optimize::global_bundle_adjuster(map_db_, num_iter_, use_huber_kernel_, verbose_);
        ok = global_BA.optimize(keyfrms,
                                optimized_keyfrm_ids, optimized_landmark_ids, optimized_landmark_line_ids,
                                optimized_marker_ids,
                                lm_to_pos_w_after_global_BA,
                                lm_line_to_plucker_after_global_BA,
                                keyfrm_to_pose_cw_after_global_BA,
                                marker_to_pos_w_after_global_BA,
                                &abort_loop_BA_);

        if (!ok) {
            AINFO << "abort loop bundle adjustment";
        } else {
            AINFO << "finish loop bundle adjustment";
            AINFO << "updating the map with pose propagation";

            ADEBUG << "update the camera pose along the spanning tree from the root";
            eigen_alloc_unord_map<unsigned int, Mat44_t> keyfrm_to_cam_pose_cw_before_BA;
            std::list<std::shared_ptr<data::keyframe>> keyfrms_to_check;
            keyfrms_to_check.push_back(curr_keyfrm->graph_node_->get_spanning_root());
            while (!keyfrms_to_check.empty()) {
                auto parent = keyfrms_to_check.front();
                const Mat44_t cam_pose_wp = parent->get_pose_wc();

                const auto children = parent->graph_node_->get_spanning_children();
                for (auto child : children) {
                    if (!optimized_keyfrm_ids.count(child->id_)) {
                        const Mat44_t cam_pose_cp = child->get_pose_cw() * cam_pose_wp;
                        keyfrm_to_pose_cw_after_global_BA[child->id_] =
                            cam_pose_cp * keyfrm_to_pose_cw_after_global_BA.at(parent->id_);
                        optimized_keyfrm_ids.insert(child->id_);
                    }

                    keyfrms_to_check.push_back(child);
                }

                keyfrm_to_cam_pose_cw_before_BA[parent->id_] = parent->get_pose_cw();
                parent->set_pose_cw(keyfrm_to_pose_cw_after_global_BA.at(parent->id_));
                keyfrms_to_check.pop_front();
            }

            ADEBUG << "update the positions of the landmarks";
            auto keyfrms = curr_keyfrm->graph_node_->get_keyframes_from_root();
            std::unordered_set<unsigned int> already_found_landmark_ids;
            std::vector<std::shared_ptr<data::landmark>> lms;
            for (const auto& keyfrm : keyfrms) {
                for (const auto& lm : keyfrm->get_landmarks()) {
                    if (!lm) {
                        continue;
                    }
                    if (lm->will_be_erased()) {
                        continue;
                    }
                    if (already_found_landmark_ids.count(lm->id_)) {
                        continue;
                    }

                    already_found_landmark_ids.insert(lm->id_);
                    lms.push_back(lm);
                }
            }

            for (const auto& lm : lms) {
                if (lm->will_be_erased()) {
                    continue;
                }

                if (optimized_landmark_ids.count(lm->id_)) {
                    lm->set_pos_in_world(lm_to_pos_w_after_global_BA.at(lm->id_));
                } else {
                    auto ref_keyfrm = lm->get_ref_keyframe();

                    assert(optimized_keyfrm_ids.count(ref_keyfrm->id_));

                    const Mat44_t pose_cw_before_BA = keyfrm_to_cam_pose_cw_before_BA.at(ref_keyfrm->id_);
                    const Mat33_t rot_cw_before_BA = pose_cw_before_BA.block<3, 3>(0, 0);
                    const Vec3_t trans_cw_before_BA = pose_cw_before_BA.block<3, 1>(0, 3);
                    const Vec3_t pos_c = rot_cw_before_BA * lm->get_pos_in_world() + trans_cw_before_BA;

                    const Mat44_t cam_pose_wc = ref_keyfrm->get_pose_wc();
                    const Mat33_t rot_wc = cam_pose_wc.block<3, 3>(0, 0);
                    const Vec3_t trans_wc = cam_pose_wc.block<3, 1>(0, 3);
                    lm->set_pos_in_world(rot_wc * pos_c + trans_wc);
                }
                lm->update_mean_normal_and_obs_scale_variance();
            }

            if (map_db_->use_line_tracking()) {
                std::unordered_set<unsigned int> already_found_line_ids;
                std::vector<std::shared_ptr<data::landmark_line>> lms_line;
                for (const auto& keyfrm : keyfrms) {
                    for (const auto& lm_line : keyfrm->get_landmarks_line()) {
                        if (!lm_line || lm_line->will_be_erased() || already_found_line_ids.count(lm_line->id_)) {
                            continue;
                        }
                        already_found_line_ids.insert(lm_line->id_);
                        lms_line.push_back(lm_line);
                    }
                }

                for (const auto& lm_line : lms_line) {
                    if (lm_line->will_be_erased()) {
                        continue;
                    }

                    if (optimized_landmark_line_ids.count(lm_line->id_)) {
                        const Vec6_t pluecker = lm_line_to_plucker_after_global_BA.at(lm_line->id_);
                        lm_line->set_pluecker_coord_without_update_endpoints(pluecker);
                        Vec6_t updated_pose_w;
                        if (optimize::line_endpoint_trimming(lm_line, pluecker, updated_pose_w)) {
                            lm_line->set_pos_in_world_without_update_pluecker(updated_pose_w);
                            lm_line->update_information();
                        } else {
                            lm_line->prepare_for_erasing(map_db_);
                        }
                    } else {
                        auto ref_keyfrm = lm_line->get_ref_keyframe();
                        if (!ref_keyfrm || !optimized_keyfrm_ids.count(ref_keyfrm->id_)) {
                            continue;
                        }

                        const Mat44_t pose_cw_before_BA = keyfrm_to_cam_pose_cw_before_BA.at(ref_keyfrm->id_);
                        const Mat33_t rot_cw_before_BA = pose_cw_before_BA.block<3, 3>(0, 0);
                        const Vec3_t trans_cw_before_BA = pose_cw_before_BA.block<3, 1>(0, 3);
                        const Vec6_t pos_w = lm_line->get_pos_in_world();
                        const Vec3_t sp_c = rot_cw_before_BA * pos_w.head<3>() + trans_cw_before_BA;
                        const Vec3_t ep_c = rot_cw_before_BA * pos_w.tail<3>() + trans_cw_before_BA;

                        const Mat44_t cam_pose_wc = ref_keyfrm->get_pose_wc();
                        const Mat33_t rot_wc = cam_pose_wc.block<3, 3>(0, 0);
                        const Vec3_t trans_wc = cam_pose_wc.block<3, 1>(0, 3);
                        Vec6_t corrected;
                        corrected.head<3>() = rot_wc * sp_c + trans_wc;
                        corrected.tail<3>() = rot_wc * ep_c + trans_wc;
                        lm_line->set_pos_in_world(corrected);
                        lm_line->update_information();
                    }
                }
            }

            ADEBUG << "update the positions of the markers";

            std::unordered_set<unsigned int> already_found_marker_ids;
            std::vector<std::shared_ptr<data::marker>> markers;
            for (const auto& keyfrm : keyfrms) {
                for (const auto& mkr : keyfrm->get_markers()) {
                    if (!mkr) {
                        continue;
                    }
                    if (already_found_marker_ids.count(mkr->id_)) {
                        continue;
                    }

                    already_found_marker_ids.insert(mkr->id_);
                    markers.push_back(mkr);
                }
            }

            for (const auto& mkr : markers) {
                if (!optimized_marker_ids.count(mkr->id_)) {
                    continue;
                }

                const std::array<Vec3_t, 4>& new_corners = marker_to_pos_w_after_global_BA.at(mkr->id_);
                for (size_t corner_idx = 0; corner_idx < 4; corner_idx++) {
                    mkr->corners_pos_w_[corner_idx] = new_corners[corner_idx];
                }
            }

            AINFO << "updated the map";
        }
    }

    {
        std::lock_guard<std::mutex> lock1(mtx_thread_);
        loop_BA_is_running_ = false;
        abort_loop_BA_ = false;
    }
}

} // namespace module
}  // namespace autonomy::localization::atlas
