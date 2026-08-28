#pragma once

#include "autonomy/localization/atlas/type.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>

namespace autonomy::localization::atlas::data {
class keyframe;
class map_database;
class landmark_line;
}  // namespace autonomy::localization::atlas::data

namespace autonomy::localization::atlas::optimize {

/** Joint local BA for 3D points and 3D lines (Structure-PLP-SLAM). */
class local_bundle_adjuster_extended_line {
public:
    explicit local_bundle_adjuster_extended_line(const YAML::Node& yaml_node,
                                                 unsigned int num_first_iter = 5,
                                                 unsigned int num_second_iter = 10);

    void optimize(data::map_database* map_db,
                  const std::shared_ptr<data::keyframe>& curr_keyfrm,
                  bool* force_stop_flag) const;

private:
    bool endpoint_trimming(const std::shared_ptr<data::landmark_line>& local_lm_line,
                           const Vec6_t& pluecker_coord,
                           Vec6_t& updated_pose_w) const;

    static Mat33_t skew(const Vec3_t& t);

    const unsigned int num_first_iter_;
    const unsigned int num_second_iter_;
    bool use_additional_keyframes_for_monocular_ = false;
};

}  // namespace autonomy::localization::atlas::optimize
