/*
 * Copyright 2026 The Openbot Authors
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

/**
 * @file factory.cpp
 * @brief Frontend factory implementation: register VO/VIO and CreateFrontend.
 */

#include "autonomy/localization/atlas/frontend/factory.hpp"

#include "autonomy/localization/atlas/frontend/lidar/lidar_odometry.hpp"
#include "autonomy/localization/atlas/frontend/tracking/visual_inertial.hpp"
#include "autonomy/localization/atlas/frontend/tracking/visual_odometry.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

FrontendBase* CreateVisualOdometry() { return new VisualOdometry(); }

FrontendBase* CreateVisualInertial() { return new VisualInertial(); }

FrontendBase* CreateLidarOdometry() { return new LidarOdometry(); }

}  // namespace

FrontendFactory& FrontendRegistry() {
    static FrontendFactory registry;
    return registry;
}

void EnsureFrontendsRegistered() {
    static bool registered = false;
    if (registered) {
        return;
    }
    FrontendRegistry().Register("vo", &CreateVisualOdometry);
    FrontendRegistry().Register("vio", &CreateVisualInertial);
    FrontendRegistry().Register("lo", &CreateLidarOdometry);
    FrontendRegistry().Register("lio", &CreateLidarOdometry);
    FrontendRegistry().Register("livo", &CreateLidarOdometry);
    registered = true;
}

std::unique_ptr<FrontendBase> CreateFrontend(const AtlasConfig& config) {
    EnsureFrontendsRegistered();
    const std::string mode_id = ToString(config.mode);
    std::unique_ptr<FrontendBase> frontend =
        FrontendRegistry().CreateObject(mode_id);
    if (!frontend) {
        return nullptr;
    }
    if (!frontend->Init(config)) {
        return nullptr;
    }
    return frontend;
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
