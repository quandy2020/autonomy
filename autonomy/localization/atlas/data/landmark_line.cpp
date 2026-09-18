#include "autonomy/localization/atlas/data/landmark_line.hpp"

#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/frontend/match/base.hpp"

#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>

namespace autonomy::localization::atlas::data {

namespace {

Vec6_t endpoints_to_pluecker(const Vec6_t& pos_w) {
    const double px = pos_w(0), py = pos_w(1), pz = pos_w(2);
    const double qx = pos_w(3), qy = pos_w(4), qz = pos_w(5);
    Vec6_t pluecker;
    pluecker << qy * pz - py * qz,
        qz * px - pz * qx,
        qx * py - px * qy,
        px - qx,
        py - qy,
        pz - qz;
    return pluecker;
}

}  // namespace

landmark_line::landmark_line(unsigned int id, const Vec6_t& pos_w, const std::shared_ptr<keyframe>& ref_keyfrm)
    : id_(id), first_keyfrm_id_(ref_keyfrm->id_), pos_w_(pos_w), ref_keyfrm_(ref_keyfrm) {
    to_pluecker_coord();
}

landmark_line::landmark_line(unsigned int id, unsigned int first_keyfrm_id, const Vec6_t& pos_w,
                             const std::shared_ptr<keyframe>& ref_keyfrm,
                             unsigned int num_visible, unsigned int num_found)
    : id_(id), first_keyfrm_id_(first_keyfrm_id), pos_w_(pos_w), ref_keyfrm_(ref_keyfrm),
      num_observable_(num_visible), num_observed_(num_found) {
    to_pluecker_coord();
}

void landmark_line::set_pos_in_world(const Vec6_t& pos_w) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    pos_w_ = pos_w;
    pluecker_coordinates_ = endpoints_to_pluecker(pos_w_);
}

void landmark_line::set_pos_in_world_without_update_pluecker(const Vec6_t& pos_w) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    pos_w_ = pos_w;
}

Vec6_t landmark_line::get_pos_in_world() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return pos_w_;
}

void landmark_line::to_pluecker_coord() {
    std::lock_guard<std::mutex> lock(mtx_position_);
    pluecker_coordinates_ = endpoints_to_pluecker(pos_w_);
}

void landmark_line::set_pluecker_coord_without_update_endpoints(const Vec6_t& pluecker) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    pluecker_coordinates_ = pluecker;
}

Vec6_t landmark_line::get_pluecker_coord() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return pluecker_coordinates_;
}

std::shared_ptr<keyframe> landmark_line::get_ref_keyframe() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return ref_keyfrm_;
}

void landmark_line::add_observation(const std::shared_ptr<keyframe>& keyfrm, unsigned int idx) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    const auto weak = std::weak_ptr<keyframe>(keyfrm);
    if (observations_.count(weak)) {
        return;
    }
    observations_[weak] = idx;
    ++num_observations_;
}

void landmark_line::erase_observation(const std::shared_ptr<keyframe>& keyfrm) {
    bool discard = false;
    {
        std::lock_guard<std::mutex> lock(mtx_observations_);
        const auto weak = std::weak_ptr<keyframe>(keyfrm);
        const auto it = observations_.find(weak);
        if (it == observations_.end()) {
            return;
        }
        observations_.erase(it);
        if (num_observable_ > 0) {
            --num_observable_;
        }
        if (ref_keyfrm_ == keyfrm && !observations_.empty()) {
            if (auto new_ref = observations_.begin()->first.lock()) {
                ref_keyfrm_ = new_ref;
            }
        }
        if (num_observations_ > 0) {
            --num_observations_;
        }
        if (num_observations_ <= 2) {
            discard = true;
        }
    }
    (void)discard;
}

landmark_line::observations_t landmark_line::get_observations() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return observations_;
}

unsigned int landmark_line::num_observations() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observations_;
}

bool landmark_line::has_observation() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return num_observations_ > 0;
}

int landmark_line::get_index_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    const auto it = observations_.find(std::weak_ptr<keyframe>(keyfrm));
    return it != observations_.end() ? static_cast<int>(it->second) : -1;
}

bool landmark_line::is_observed_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const {
    return get_index_in_keyframe(keyfrm) >= 0;
}

cv::Mat landmark_line::get_descriptor() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return descriptor_.clone();
}

void landmark_line::compute_descriptor() {
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock(mtx_observations_);
        if (will_be_erased_) {
            return;
        }
        observations = observations_;
    }
    if (observations.empty()) {
        return;
    }

    std::vector<cv::Mat> descriptors;
    descriptors.reserve(observations.size());
    for (const auto& obs : observations) {
        auto keyfrm = obs.first.lock();
        if (!keyfrm || keyfrm->will_be_erased()) {
            continue;
        }
        const auto idx = obs.second;
        if (idx < static_cast<unsigned int>(keyfrm->line_obs_.lbd_descriptors.rows)) {
            descriptors.push_back(keyfrm->line_obs_.lbd_descriptors.row(static_cast<int>(idx)));
        }
    }
    if (descriptors.empty()) {
        return;
    }

    const auto num_descs = descriptors.size();
    std::vector<std::vector<unsigned int>> hamm_dists(num_descs, std::vector<unsigned int>(num_descs));
    for (unsigned int i = 0; i < num_descs; ++i) {
        hamm_dists[i][i] = 0;
        for (unsigned int j = i + 1; j < num_descs; ++j) {
            const auto dist = match::compute_descriptor_distance_32(descriptors[i], descriptors[j]);
            hamm_dists[i][j] = dist;
            hamm_dists[j][i] = dist;
        }
    }

    unsigned int best_median_dist = match::MAX_HAMMING_DIST;
    unsigned int best_idx = 0;
    for (unsigned int idx = 0; idx < num_descs; ++idx) {
        auto partial = hamm_dists[idx];
        std::sort(partial.begin(), partial.end());
        const auto median_dist = partial[static_cast<unsigned int>(0.5 * (num_descs - 1))];
        if (median_dist < best_median_dist) {
            best_median_dist = median_dist;
            best_idx = idx;
        }
    }

    std::lock_guard<std::mutex> lock(mtx_observations_);
    descriptor_ = descriptors[best_idx].clone();
}

void landmark_line::prepare_for_erasing(map_database* map_db) {
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        std::lock_guard<std::mutex> lock2(mtx_position_);
        observations = observations_;
        observations_.clear();
        will_be_erased_ = true;
    }
    for (const auto& keyfrm_and_idx : observations) {
        if (auto keyfrm = keyfrm_and_idx.first.lock()) {
            keyfrm->erase_landmark_line_with_index(keyfrm_and_idx.second);
        }
    }
    if (map_db) {
        map_db->erase_landmark_line(id_);
    }
}

bool landmark_line::will_be_erased() {
    std::lock_guard<std::mutex> lock1(mtx_observations_);
    std::lock_guard<std::mutex> lock2(mtx_position_);
    return will_be_erased_;
}

void landmark_line::update_information() {
    observations_t observations;
    std::shared_ptr<keyframe> ref_kf;
    Vec6_t pose;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        std::lock_guard<std::mutex> lock2(mtx_position_);
        if (will_be_erased_) {
            return;
        }
        observations = observations_;
        ref_kf = ref_keyfrm_;
        pose = pos_w_;
    }
    if (!ref_kf || observations.empty()) {
        return;
    }

    const Vec3_t sp = pose.head<3>();
    const Vec3_t ep = pose.tail<3>();
    const Vec3_t mp = 0.5 * (sp + ep);
    const double distance = (mp - ref_kf->get_trans_wc()).norm();

    const auto ref_weak = std::weak_ptr<keyframe>(ref_kf);
    const auto ref_it = observations.find(ref_weak);
    if (ref_it == observations.end()) {
        return;
    }
    const int level = ref_kf->line_obs_.keylines.at(ref_it->second).octave;
    const float level_scale_factor = ref_kf->scale_factors_lsd_.empty()
                                         ? 1.f
                                         : ref_kf->scale_factors_lsd_.at(static_cast<size_t>(level));
    const int nlevels = static_cast<int>(ref_kf->num_line_scale_levels_);

    std::lock_guard<std::mutex> lock3(mtx_position_);
    max_valid_dist_ = static_cast<float>(distance * level_scale_factor);
    min_valid_dist_ = max_valid_dist_ / ref_kf->orb_params_->scale_factors_.at(static_cast<size_t>(std::max(0, nlevels - 1)));
}

float landmark_line::get_min_valid_distance() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return 0.8f * min_valid_dist_;
}

float landmark_line::get_max_valid_distance() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return 1.2f * max_valid_dist_;
}

unsigned int landmark_line::predict_scale_level(float current_dist, float log_scale_factor,
                                                unsigned int num_scale_levels) const {
    float ratio;
    {
        std::lock_guard<std::mutex> lock(mtx_position_);
        ratio = max_valid_dist_ / current_dist;
    }
    const auto pred = static_cast<int>(std::ceil(std::log(ratio) / log_scale_factor));
    if (pred < 0) {
        return 0;
    }
    if (static_cast<unsigned int>(pred) >= num_scale_levels) {
        return num_scale_levels - 1;
    }
    return static_cast<unsigned int>(pred);
}

void landmark_line::replace(const std::shared_ptr<landmark_line>& line) {
    if (!line || line->id_ == id_) {
        return;
    }
    observations_t observations;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        std::lock_guard<std::mutex> lock2(mtx_position_);
        observations = observations_;
        observations_.clear();
        replaced_ = line;
        will_be_erased_ = true;
    }
    for (const auto& obs : observations) {
        if (auto keyfrm = obs.first.lock()) {
            keyfrm->replace_landmark_line(line, obs.second);
            line->add_observation(keyfrm, obs.second);
        }
    }
}

std::shared_ptr<landmark_line> landmark_line::get_replaced() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return replaced_;
}

void landmark_line::increase_num_observable(unsigned int num_observable) {
    num_observable_ += num_observable;
}

void landmark_line::increase_num_observed(unsigned int num_observed) {
    num_observed_ += num_observed;
}

float landmark_line::get_observed_ratio() const {
    if (num_observable_ == 0) {
        return 0.f;
    }
    return static_cast<float>(num_observed_) / static_cast<float>(num_observable_);
}

nlohmann::json landmark_line::to_json() const {
    nlohmann::json j;
    j["id"] = id_;
    j["first_keyfrm_id"] = first_keyfrm_id_;
    j["pos_w"] = {pos_w_(0), pos_w_(1), pos_w_(2), pos_w_(3), pos_w_(4), pos_w_(5)};
    j["ref_keyfrm_id"] = ref_keyfrm_ ? ref_keyfrm_->id_ : 0;
    j["n_vis"] = num_observable_;
    j["n_fnd"] = num_observed_;
    return j;
}

}  // namespace autonomy::localization::atlas::data
