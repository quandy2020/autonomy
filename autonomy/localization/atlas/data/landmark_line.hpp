/*
 * 3D line landmark (Structure-PLP-SLAM).
 */
#pragma once

#include "autonomy/localization/atlas/type.hpp"

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <opencv2/core/mat.hpp>

#include <nlohmann/json_fwd.hpp>

namespace autonomy::localization::atlas::data {

class keyframe;
class map_database;

class landmark_line : public std::enable_shared_from_this<landmark_line> {
public:
    using observations_t = std::map<std::weak_ptr<keyframe>, unsigned int, id_less<std::weak_ptr<keyframe>>>;

    landmark_line(unsigned int id, const Vec6_t& pos_w, const std::shared_ptr<keyframe>& ref_keyfrm);

    landmark_line(unsigned int id, unsigned int first_keyfrm_id, const Vec6_t& pos_w,
                  const std::shared_ptr<keyframe>& ref_keyfrm,
                  unsigned int num_visible, unsigned int num_found);

    void set_pos_in_world(const Vec6_t& pos_w);
    void set_pos_in_world_without_update_pluecker(const Vec6_t& pos_w);
    Vec6_t get_pos_in_world() const;

    void to_pluecker_coord();
    void set_pluecker_coord_without_update_endpoints(const Vec6_t& pluecker);
    Vec6_t get_pluecker_coord() const;

    std::shared_ptr<keyframe> get_ref_keyframe() const;

    void add_observation(const std::shared_ptr<keyframe>& keyfrm, unsigned int idx);
    void erase_observation(const std::shared_ptr<keyframe>& keyfrm);

    observations_t get_observations() const;
    unsigned int num_observations() const;
    bool has_observation() const;

    int get_index_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const;
    bool is_observed_in_keyframe(const std::shared_ptr<keyframe>& keyfrm) const;

    cv::Mat get_descriptor() const;
    void compute_descriptor();

    void prepare_for_erasing(map_database* map_db);
    bool will_be_erased();

    void update_information();
    float get_min_valid_distance() const;
    float get_max_valid_distance() const;

    unsigned int predict_scale_level(float current_dist, float log_scale_factor, unsigned int num_scale_levels) const;

    void replace(const std::shared_ptr<landmark_line>& line);
    std::shared_ptr<landmark_line> get_replaced() const;

    void increase_num_observable(unsigned int num_observable = 1);
    void increase_num_observed(unsigned int num_observed = 1);
    float get_observed_ratio() const;

    bool is_inside_in_feature_scale(float cam_to_lm_dist) const {
        return get_min_valid_distance() <= cam_to_lm_dist && cam_to_lm_dist <= get_max_valid_distance();
    }

    nlohmann::json to_json() const;

    // tracking / local map (PLP-compatible public fields)
    unsigned int id_ = 0;
    unsigned int first_keyfrm_id_ = 0;
    unsigned int num_observations_ = 0;

    Vec2_t reproj_in_tracking_sp_{Vec2_t::Zero()};
    Vec2_t reproj_in_tracking_ep_{Vec2_t::Zero()};
    bool is_observable_in_tracking_ = false;
    int scale_level_in_tracking_ = 0;
    unsigned int identifier_in_local_map_update_ = 0;
    unsigned int identifier_in_local_lm_search_ = 0;

    unsigned int loop_fusion_identifier_ = 0;
    unsigned int ref_keyfrm_id_in_loop_fusion_ = 0;

private:
    Vec6_t pos_w_{Vec6_t::Zero()};
    Vec6_t pluecker_coordinates_{Vec6_t::Zero()};
    std::shared_ptr<keyframe> ref_keyfrm_;
    observations_t observations_;
    cv::Mat descriptor_;
    unsigned int num_observable_ = 1;
    unsigned int num_observed_ = 1;
    bool will_be_erased_ = false;
    std::shared_ptr<landmark_line> replaced_;
    float min_valid_dist_ = 0.f;
    float max_valid_dist_ = 0.f;
    mutable std::mutex mtx_position_;
    mutable std::mutex mtx_observations_;
};

// Backward compatibility for ported PLP code
using Line = landmark_line;

}  // namespace autonomy::localization::atlas::data
