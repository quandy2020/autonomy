#pragma once

#include "autonomy/localization/atlas/type.hpp"

#include <yaml-cpp/yaml.h>

#include <memory>

namespace autonomy::localization::atlas::data {
class keyframe;
class map_database;
}  // namespace autonomy::localization::atlas::data

namespace autonomy::localization::atlas::optimize {

/** Local BA for 3D points with fixed plane constraints (Structure-PLP-SLAM). */
class local_bundle_adjuster_extended_plane {
public:
    explicit local_bundle_adjuster_extended_plane(const YAML::Node& yaml_node,
                                                  unsigned int num_first_iter = 5,
                                                  unsigned int num_second_iter = 10);

    void optimize(data::map_database* map_db,
                  const std::shared_ptr<data::keyframe>& curr_keyfrm,
                  bool* force_stop_flag) const;

private:
    const unsigned int num_first_iter_;
    const unsigned int num_second_iter_;
    bool use_additional_keyframes_for_monocular_ = false;
};

}  // namespace autonomy::localization::atlas::optimize
