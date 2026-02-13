/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/tools/aviz/common/frame_manager.hpp"

#include <algorithm>
#include <mutex>

namespace aviz {
namespace common {

FrameManager::FrameManager() : fixed_frame_("map"), pause_(false), sync_mode_(SyncOff), current_time_(0) {}

FrameManager::~FrameManager() = default;

void FrameManager::setFixedFrame(const std::string& frame) {
    std::lock_guard<std::mutex> lock(transforms_mutex_);
    fixed_frame_ = frame;
}

void FrameManager::setPause(bool pause) {
    pause_ = pause;
}

bool FrameManager::getPause() {
    return pause_;
}

void FrameManager::setSyncMode(SyncMode mode) {
    sync_mode_ = mode;
}

FrameManagerIface::SyncMode FrameManager::getSyncMode() {
    return sync_mode_;
}

void FrameManager::syncTime(uint64_t time_ns) {
    current_time_ = time_ns;
}

uint64_t FrameManager::getTime() {
    return current_time_;
}

bool FrameManager::getTransform(const std::string& frame, QVector3D& position, QQuaternion& orientation) {
    return getTransform(frame, getTime(), position, orientation);
}

bool FrameManager::getTransform(const std::string& frame, uint64_t time_ns, QVector3D& position,
                                QQuaternion& orientation) {
    (void)time_ns;

    std::lock_guard<std::mutex> lock(transforms_mutex_);

    if (frame == fixed_frame_) {
        position = QVector3D(0, 0, 0);
        orientation = QQuaternion(1, 0, 0, 0);
        return true;
    }

    auto it = transforms_.find(frame);
    if (it != transforms_.end()) {
        position = QVector3D(it->second.position.x, it->second.position.y, it->second.position.z);
        orientation = QQuaternion(it->second.orientation.w, it->second.orientation.x, it->second.orientation.y,
                                  it->second.orientation.z);
        return true;
    }

    // If frame not found, return identity transform (for now)
    position = QVector3D(0, 0, 0);
    orientation = QQuaternion(1, 0, 0, 0);
    return false;
}

bool FrameManager::transform(const std::string& frame, uint64_t time_ns, double x, double y, double z, double qx,
                             double qy, double qz, double qw, double& out_x, double& out_y, double& out_z,
                             double& out_qx, double& out_qy, double& out_qz, double& out_qw) {
    QVector3D position;
    QQuaternion orientation;
    if (!getTransform(frame, time_ns, position, orientation)) {
        return false;
    }

    // Apply transform (simplified - would do proper quaternion multiplication in full implementation)
    out_x = x + position.x();
    out_y = y + position.y();
    out_z = z + position.z();
    out_qx = qx;
    out_qy = qy;
    out_qz = qz;
    out_qw = qw;
    return true;
}

void FrameManager::update() {
    // Clear cache if needed
}

bool FrameManager::frameHasProblems(const std::string& frame, std::string& error) {
    std::lock_guard<std::mutex> lock(transforms_mutex_);
    if (frame == fixed_frame_) {
        return false;
    }
    auto it = transforms_.find(frame);
    if (it == transforms_.end()) {
        error = "Frame [" + frame + "] does not exist";
        return true;
    }
    return false;
}

bool FrameManager::transformHasProblems(const std::string& frame, std::string& error) {
    return transformHasProblems(frame, getTime(), error);
}

bool FrameManager::transformHasProblems(const std::string& frame, uint64_t time_ns, std::string& error) {
    (void)time_ns;
    return frameHasProblems(frame, error);
}

const std::string& FrameManager::getFixedFrame() {
    return fixed_frame_;
}

transformation::TransformationLibraryConnector::WeakPtr FrameManager::getConnector() {
    return transformation::TransformationLibraryConnector::WeakPtr();
}

std::shared_ptr<transformation::FrameTransformer> FrameManager::getTransformer() {
    return nullptr;
}

void FrameManager::setTransformerPlugin(std::shared_ptr<transformation::FrameTransformer> transformer) {
    (void)transformer;
    // TODO: Store and use transformer
}

// Helper methods for Ogre types
bool FrameManager::getTransform(const std::string& frame, Ogre::Vector3& position, Ogre::Quaternion& orientation) {
    QVector3D qpos;
    QQuaternion qorient;
    if (!getTransform(frame, qpos, qorient)) {
        return false;
    }
    position = Ogre::Vector3(qpos.x(), qpos.y(), qpos.z());
    orientation = Ogre::Quaternion(qorient.scalar(), qorient.x(), qorient.y(), qorient.z());
    return true;
}

bool FrameManager::getTransform(const std::string& frame, int32_t sec, uint32_t nanosec, Ogre::Vector3& position,
                                Ogre::Quaternion& orientation) {
    (void)sec;
    (void)nanosec;
    return getTransform(frame, position, orientation);
}

std::vector<std::string> FrameManager::getAllFrameNames() {
    std::lock_guard<std::mutex> lock(transforms_mutex_);
    std::vector<std::string> frames;
    frames.push_back(fixed_frame_);
    for (const auto& [frame, transform] : transforms_) {
        frames.push_back(frame);
    }
    return frames;
}

void FrameManager::setTransform(const std::string& frame, const std::string& parent_frame,
                                const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {
    std::lock_guard<std::mutex> lock(transforms_mutex_);
    Transform t;
    t.parent_frame = parent_frame;
    t.position = position;
    t.orientation = orientation;
    transforms_[frame] = t;
}

}  // namespace common
}  // namespace aviz
