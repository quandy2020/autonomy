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

#pragma once

#include <map>
#include <mutex>
#include <string>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "autonomy/tools/aviz/common/frame_manager_iface.hpp"

namespace aviz {
namespace common {

/**
 * @brief Simple FrameManager implementation
 * Similar to rviz_common::FrameManager
 *
 * Manages coordinate frame transformations
 */
class FrameManager : public FrameManagerIface
{
    Q_OBJECT

public:
    FrameManager();
    ~FrameManager() override;

    void setFixedFrame(const std::string& frame) override;
    void setPause(bool pause) override;
    bool getPause() override;
    void setSyncMode(SyncMode mode) override;
    SyncMode getSyncMode() override;
    void syncTime(uint64_t time_ns) override;
    uint64_t getTime() override;

    bool getTransform(const std::string& frame, QVector3D& position, QQuaternion& orientation) override;
    bool getTransform(const std::string& frame, uint64_t time_ns, QVector3D& position,
                      QQuaternion& orientation) override;
    bool transform(const std::string& frame, uint64_t time_ns, double x, double y, double z, double qx, double qy,
                   double qz, double qw, double& out_x, double& out_y, double& out_z, double& out_qx, double& out_qy,
                   double& out_qz, double& out_qw) override;
    void update() override;
    bool frameHasProblems(const std::string& frame, std::string& error) override;
    bool transformHasProblems(const std::string& frame, std::string& error) override;
    bool transformHasProblems(const std::string& frame, uint64_t time_ns, std::string& error) override;
    const std::string& getFixedFrame() override;
    transformation::TransformationLibraryConnector::WeakPtr getConnector() override;
    std::shared_ptr<transformation::FrameTransformer> getTransformer() override;

public Q_SLOTS:
    void setTransformerPlugin(std::shared_ptr<transformation::FrameTransformer> transformer) override;
    std::vector<std::string> getAllFrameNames() override;

    // Helper methods for Ogre types
    bool getTransform(const std::string& frame, Ogre::Vector3& position, Ogre::Quaternion& orientation);
    bool getTransform(const std::string& frame, int32_t sec, uint32_t nanosec, Ogre::Vector3& position,
                      Ogre::Quaternion& orientation);

    // Helper methods
    void setTransform(const std::string& frame, const std::string& parent_frame, const Ogre::Vector3& position,
                      const Ogre::Quaternion& orientation);

private:
    struct Transform {
        std::string parent_frame;
        Ogre::Vector3 position;
        Ogre::Quaternion orientation;
    };

    std::string fixed_frame_;
    bool pause_;
    SyncMode sync_mode_;
    uint64_t current_time_;
    std::map<std::string, Transform> transforms_;
    std::mutex transforms_mutex_;
};

}  // namespace common
}  // namespace aviz
