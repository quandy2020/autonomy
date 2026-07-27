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

#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "autonomy/common/macros.hpp"
#include "autonomy/map/strata/urdf/urdf_loader.hpp"

namespace autonomy {
namespace map {
namespace strata {
namespace urdf {

class UrdfPlugin
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(UrdfPlugin)

    bool UrdfLoad(const std::string& url);
    bool GltfLoad(const std::string& url);
    void SetJoint(const std::string& jointName, double angleRad);
    void SetJoints(const std::unordered_map<std::string, double>& joints);
    void ResetJoints();
    void SetCameraPosition(double x, double y, double z);
    void SetCameraRotation(double pitch, double yaw, double roll);
    void OnResize(int width, int height);
    void AddDirectionalLight(double x, double y, double z, double intensity);
    void AddAmbientLight(double intensity);
    void AddGridHelper(double size, int divisions);
    void AddGround(double size);
    void AddAxesHelper(double size);
    void SetLinkMaterialColor(const std::string& link, float r, float g, float b);
    void ResetLinkMaterialColor(const std::string& link);
    void SetLookAtRobot(bool enabled);
    void MoveMouseCamera(double deltaX, double deltaY);
    void AddElevation(double delta);
    void AddAzimuth(double delta);
    void AddDistance(double delta);
    void MoveForward(double step);
    void MoveBackward(double step);
    void MoveLeft(double step);
    void MoveRight(double step);
    void TurnLeft(double angle);
    void TurnRight(double angle);
    void StartRender();
    void StopRender();
    void Destroy();

    bool IsLoaded() const { return loaded_; }
    const UrdfModel& Model() const { return model_; }

private:
    bool rendering_{false};
    bool loaded_{false};
    UrdfModel model_;
    std::unordered_map<std::string, double> joints_;
};

}  // namespace urdf
}  // namespace strata
}  // namespace map
}  // namespace autonomy
