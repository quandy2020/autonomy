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

#include "autonomy/common/logging.hpp"
#include "autonomy/map/strata/urdf/urdf_loader.hpp"
#include "autonomy/map/strata/urdf/urdf_plugin.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace urdf {

bool UrdfPlugin::UrdfLoad(const std::string& url) {
    UrdfModel model;
    std::string error;
    if (!LoadUrdfFile(url, model, &error)) {
        AERROR << error;
        loaded_ = false;
        return false;
    }
    model_ = std::move(model);
    loaded_ = true;
    joints_.clear();
    AINFO << "URDF loaded: " << model_.name << " (" << model_.links.size() << " links, "
          << model_.joints.size() << " joints) from " << model_.sourcePath;
    return true;
}

bool UrdfPlugin::GltfLoad(const std::string& url) {
    AINFO << "GLTF load: " << url;
    loaded_ = true;
    model_ = UrdfModel{};
    model_.sourcePath = url;
    model_.name = "gltf";
    return true;
}

void UrdfPlugin::SetJoint(const std::string& jointName, double angleRad) {
    joints_[jointName] = angleRad;
}

void UrdfPlugin::SetJoints(const std::unordered_map<std::string, double>& joints) {
    joints_ = joints;
}

void UrdfPlugin::ResetJoints() { joints_.clear(); }

void UrdfPlugin::SetCameraPosition(double, double, double) {}
void UrdfPlugin::SetCameraRotation(double, double, double) {}
void UrdfPlugin::OnResize(int, int) {}
void UrdfPlugin::AddDirectionalLight(double, double, double, double) {}
void UrdfPlugin::AddAmbientLight(double) {}
void UrdfPlugin::AddGridHelper(double, int) {}
void UrdfPlugin::AddGround(double) {}
void UrdfPlugin::AddAxesHelper(double) {}
void UrdfPlugin::SetLinkMaterialColor(const std::string&, float, float, float) {}
void UrdfPlugin::ResetLinkMaterialColor(const std::string&) {}
void UrdfPlugin::SetLookAtRobot(bool) {}
void UrdfPlugin::MoveMouseCamera(double, double) {}
void UrdfPlugin::AddElevation(double) {}
void UrdfPlugin::AddAzimuth(double) {}
void UrdfPlugin::AddDistance(double) {}
void UrdfPlugin::MoveForward(double) {}
void UrdfPlugin::MoveBackward(double) {}
void UrdfPlugin::MoveLeft(double) {}
void UrdfPlugin::MoveRight(double) {}
void UrdfPlugin::TurnLeft(double) {}
void UrdfPlugin::TurnRight(double) {}
void UrdfPlugin::StartRender() { rendering_ = true; }
void UrdfPlugin::StopRender() { rendering_ = false; }
void UrdfPlugin::Destroy() {
    rendering_ = false;
    loaded_ = false;
    joints_.clear();
    model_ = UrdfModel{};
}

}  // namespace urdf
}  // namespace strata
}  // namespace map
}  // namespace autonomy
