/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_PUBLISH_FRAME_PUBLISHER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_PUBLISH_FRAME_PUBLISHER_HPP_

#include "autonomy/localization/atlas/config.hpp"
#include "autonomy/localization/atlas/tracking_module.hpp"

#include <mutex>
#include <vector>
#include <memory>
#include <utility>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace autonomy::localization::atlas {

class tracking_module;

namespace data {
class map_database;
} // namespace data

namespace publish {

class frame_publisher {
public:
    /**
     * Constructor
     */
    frame_publisher(const std::shared_ptr<config>& cfg, data::map_database* map_db,
                    const unsigned int img_width = 1024);

    /**
     * Destructor
     */
    virtual ~frame_publisher();

    /**
     * Update tracking information
     * NOTE: should be accessed from system thread
     */
    void update(const std::vector<std::shared_ptr<data::landmark>>& curr_lms,
                bool mapping_is_enabled,
                tracker_state_t tracking_state,
                std::vector<cv::KeyPoint>& keypts,
                std::vector<data::marker2d>& mkrs2d,
                const cv::Mat& img,
                double tracking_time_elapsed_ms,
                double extraction_time_elapsed_ms);

    /**
     * Get the current image with tracking information
     * NOTE: should be accessed from viewer thread
     */
    cv::Mat draw_frame();

    /**
     * Side-by-side previous/current frame with matched feature lines.
     */
    cv::Mat draw_frame_matches();

    std::string get_tracking_state();

    std::vector<cv::KeyPoint> get_keypoints();

    bool get_mapping_is_enabled();

    std::vector<std::shared_ptr<data::landmark>> get_landmarks();

    std::pair<std::vector<cv::KeyPoint>, std::vector<std::shared_ptr<data::landmark>>> get_keypoints_and_landmarks();

    cv::Mat get_image();

    double get_tracking_time_elapsed_ms();

    double get_extraction_time_elapsed_ms();

protected:
    unsigned int draw_tracked_points(cv::Mat& img, const std::vector<cv::KeyPoint>& curr_keypts,
                                     const std::vector<std::shared_ptr<data::landmark>>& curr_lms,
                                     const bool mapping_is_enabled,
                                     const float mag = 1.0) const;

    void draw_markers2d(cv::Mat& img, const std::vector<data::marker2d>& mkrs2d, const float mag = 1.0);

    void draw_match_lines(cv::Mat& canvas, int prev_panel_width,
                          const std::vector<cv::KeyPoint>& prev_keypts,
                          const std::vector<std::shared_ptr<data::landmark>>& prev_lms,
                          const std::vector<cv::KeyPoint>& curr_keypts,
                          const std::vector<std::shared_ptr<data::landmark>>& curr_lms,
                          float mag) const;

    // colors (BGR)
    const cv::Scalar mapping_color_{0, 255, 255};
    const cv::Scalar localization_color_{255, 255, 0};
    const cv::Scalar marker_color_{255, 0, 255};
    const cv::Scalar match_line_color_{0, 255, 0};
    const cv::Scalar match_point_color_{0, 200, 255};

    //! config
    std::shared_ptr<config> cfg_;
    //! map database
    data::map_database* map_db_;
    //! maximum size of output images
    const int img_width_;

    // -------------------------------------------
    //! mutex to access variables below
    std::mutex mtx_;

    //! raw img
    cv::Mat img_;
    //! tracking state
    tracker_state_t tracking_state_;

    //! current keypoints
    std::vector<cv::KeyPoint> curr_keypts_;

    std::vector<data::marker2d> curr_mkrs2d_;

    //! elapsed time for tracking
    double tracking_time_elapsed_ms_ = 0.0;

    //! elapsed time for feature extraction
    double extraction_time_elapsed_ms_ = 0.0;

    //! mapping module status
    bool mapping_is_enabled_;

    std::vector<std::shared_ptr<data::landmark>> curr_lms_;

    bool has_prev_frame_ = false;
    cv::Mat prev_img_;
    std::vector<cv::KeyPoint> prev_keypts_;
    std::vector<std::shared_ptr<data::landmark>> prev_lms_;
};

} // namespace publish
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_PUBLISH_FRAME_PUBLISHER_HPP_
