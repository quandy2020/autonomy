#pragma once

#include "autonomy/localization/atlas/plp/plp_options.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <opencv2/core/types.hpp>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace autonomy::localization::atlas::data {
class keyframe;
class map_database;
class landmark;
class landmark_plane;
}  // namespace autonomy::localization::atlas::data

namespace autonomy::localization::atlas::plp {

/** Segmentation-driven plane mapping (Structure-PLP-SLAM). */
class planar_mapping_module {
public:
    planar_mapping_module(data::map_database* map_db, bool is_monocular, const Options& options);

    bool process_new_keyframe(const std::shared_ptr<data::keyframe>& keyfrm);
    void refinement();

private:
    void estimate_map_scale();
    void estimate_map_scale(const std::shared_ptr<data::keyframe>& keyfrm);

    bool create_color_to_plane(const std::shared_ptr<data::keyframe>& keyfrm,
                               std::unordered_map<long, std::shared_ptr<data::landmark_plane>>& color_to_planes);
    bool create_new_planes(std::unordered_map<long, std::shared_ptr<data::landmark_plane>>& color_to_planes);

    bool estimate_plane_sequential_ransac(const std::shared_ptr<data::landmark_plane>& plane);
    bool estimate_plane_sequential_graph_cut_ransac(const std::shared_ptr<data::landmark_plane>& plane);
    bool update_plane_via_ransac(const std::shared_ptr<data::landmark_plane>& plane);

    bool merge_planes();
    bool refine_planes();
    bool refine_points();

    double estimate_plane_svd(const std::vector<std::shared_ptr<data::landmark>>& landmarks,
                              const std::vector<int>& indexes,
                              double& a, double& b, double& c, double& d) const;

    static long color_hash(const cv::Vec3b& color);

    data::map_database* map_db_ = nullptr;
    bool is_monocular_ = false;
    Options options_;

    double map_scale_ = 1.0;
    double planar_distance_thresh_ = 0.02;
    double final_error_thresh_ = 0.01;
    double offset_delta_threshold_ = 0.12;

    std::mutex mtx_plane_;
};

}  // namespace autonomy::localization::atlas::plp
