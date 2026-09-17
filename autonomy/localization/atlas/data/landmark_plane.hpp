/*
 * 3D plane landmark (Structure-PLP-SLAM).
 */
#pragma once

#include "autonomy/localization/atlas/type.hpp"

#include <nlohmann/json.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace autonomy::localization::atlas::data {

class keyframe;
class landmark;
class map_database;

class landmark_plane : public std::enable_shared_from_this<landmark_plane> {
public:
    explicit landmark_plane(const std::shared_ptr<keyframe>& ref_keyfrm, map_database* map_db);

    void add_landmark(const std::shared_ptr<landmark>& lm);
    std::vector<std::shared_ptr<landmark>> get_landmarks() const;
    void set_landmarks(const std::vector<std::shared_ptr<landmark>>& lms);
    unsigned int num_landmarks() const;

    void add_fit_sample(const Vec3_t& pos_w);
    void clear_fit_samples();
    const std::vector<Vec3_t>& get_fit_samples() const;

    void set_equation(double a, double b, double c, double d);
    void get_equation(double& a, double& b, double& c, double& d) const;

    bool is_valid() const;
    void set_valid();
    void set_invalid();

    Vec3_t get_normal() const;
    double get_offset() const;
    double calculate_distance(const Vec3_t& pos_w) const;

    void set_landmarks_ownership();
    void remove_landmarks_ownership();

    void set_best_error(double error);
    double get_best_error() const;

    double get_normal_norm() const;
    void merge(const std::shared_ptr<landmark_plane>& other);

    bool need_refinement() const;
    void set_need_refinement();
    void set_refinement_is_done();

    nlohmann::json to_json() const;
    static std::shared_ptr<landmark_plane> from_json(const nlohmann::json& json,
                                                     map_database* map_db,
                                                     const std::shared_ptr<keyframe>& ref_keyfrm);

    unsigned int id_ = 0;

    long seg_label_ = 0;
    bool has_display_color_ = false;
    float display_r_ = 0.5f;
    float display_g_ = 0.5f;
    float display_b_ = 0.5f;

    void set_seg_label(long label);
    long get_seg_label() const;
    void get_display_color(float& r, float& g, float& b) const;

private:
    Vec3_t normal_{1.0, 0.0, 0.0};
    double offset_ = -1.0;
    double abs_normal_ = 1.0;

    std::unordered_map<unsigned int, std::shared_ptr<landmark>> landmarks_;
    std::shared_ptr<keyframe> ref_keyfrm_;
    map_database* map_db_ = nullptr;

    bool valid_ = false;
    bool needs_refinement_ = false;
    double best_error_ = 0.0;
    std::vector<Vec3_t> fit_samples_;

    mutable std::mutex mtx_position_;
    mutable std::mutex mtx_observations_;
};

}  // namespace autonomy::localization::atlas::data
