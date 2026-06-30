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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_UTIL_STEREO_RECTIFIER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_UTIL_STEREO_RECTIFIER_HPP_

#include "autonomy/localization/atlas/config.hpp"
#include "autonomy/localization/atlas/camera/base.hpp"

#include <memory>

#include <opencv2/core/mat.hpp>

namespace autonomy::localization::atlas {
namespace util {

class stereo_rectifier {
public:
    //! Constructor
    explicit stereo_rectifier(const std::shared_ptr<config>& cfg, camera::base* camera);

    //! Constructor
    stereo_rectifier(camera::base* camera, const YAML::Node& yaml_node);

    //! Destructor
    virtual ~stereo_rectifier();

    //! Apply stereo-rectification
    void rectify(const cv::Mat& in_img_l, const cv::Mat& in_img_r,
                 cv::Mat& out_img_l, cv::Mat& out_img_r) const;

private:
    //! Parse std::vector as cv::Mat
    static cv::Mat parse_vector_as_mat(const cv::Size& shape, const std::vector<double>& vec);

    //! Load model type before rectification from YAML
    static camera::model_type_t load_model_type(const YAML::Node& yaml_node);

    //! camera model type before rectification
    const camera::model_type_t model_type_;

    //! undistortion map for x-axis in left image
    cv::Mat undist_map_x_l_;
    //! undistortion map for y-axis in left image
    cv::Mat undist_map_y_l_;
    //! undistortion map for x-axis in right image
    cv::Mat undist_map_x_r_;
    //! undistortion map for y-axis in right image
    cv::Mat undist_map_y_r_;
};

} // namespace util
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_UTIL_STEREO_RECTIFIER_HPP_
