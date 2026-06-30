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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_CAMERA_CAMERA_FACTORY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_CAMERA_CAMERA_FACTORY_HPP_

#include "autonomy/localization/atlas/camera/base.hpp"
#include "autonomy/localization/atlas/camera/perspective.hpp"
#include "autonomy/localization/atlas/camera/fisheye.hpp"
#include "autonomy/localization/atlas/camera/equirectangular.hpp"
#include "autonomy/localization/atlas/camera/radial_division.hpp"
#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {

namespace camera {

class camera_factory {
public:
    static camera::base* create(const YAML::Node& node) {
        const auto camera_model_type = camera::base::load_model_type(node);

        camera::base* camera = nullptr;
        try {
            switch (camera_model_type) {
                case camera::model_type_t::Perspective: {
                    camera = new camera::perspective(node);
                    break;
                }
                case camera::model_type_t::Fisheye: {
                    camera = new camera::fisheye(node);
                    break;
                }
                case camera::model_type_t::Equirectangular: {
                    camera = new camera::equirectangular(node);
                    break;
                }
                case camera::model_type_t::RadialDivision: {
                    camera = new camera::radial_division(node);
                    break;
                }
            }
        }
        catch (const std::exception& e) {
            ADEBUG << "failed in loading camera model parameters: " << e.what();
            if (camera) {
                delete camera;
                camera = nullptr;
            }
            throw;
        }

        assert(camera != nullptr);
        if (camera->setup_type_ == camera::setup_type_t::Stereo || camera->setup_type_ == camera::setup_type_t::RGBD) {
            if (camera->model_type_ == camera::model_type_t::Equirectangular) {
                throw std::runtime_error("Not implemented: Stereo or RGBD of equirectangular camera model");
            }
        }
        return camera;
    }
};

} // namespace camera
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_CAMERA_CAMERA_FACTORY_HPP_
