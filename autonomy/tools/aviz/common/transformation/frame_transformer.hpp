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

#ifndef AVIZ_COMMON__TRANSFORMATION__FRAME_TRANSFORMER_HPP_
#define AVIZ_COMMON__TRANSFORMATION__FRAME_TRANSFORMER_HPP_

#include <QString>  // NOLINT
#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace aviz {
namespace common {
namespace transformation {

class FrameTransformerException : public std::runtime_error
{
public:
    explicit FrameTransformerException(const char* error_message)
        : std::runtime_error(error_message) {}

    ~FrameTransformerException() noexcept override = default;
};

/// Class from which the plugin specific implementation of FrameTransformer
/// should inherit.
class TransformationLibraryConnector
{
public:
    virtual ~TransformationLibraryConnector() = default;

    using WeakPtr = std::weak_ptr<TransformationLibraryConnector>;
};

/// Base class for frame transformation
/**
 * FrameTransformer provides an interface for transforming poses between
 * different coordinate frames. This is adapted for aviz system (not using ROS
 * tf2).
 */
class FrameTransformer
{
public:
    virtual ~FrameTransformer() = default;

    /// Initialize the transformer
    /**
     * The initialization of a FrameTransformer object is delegated to this
     * method.
     */
    virtual void initialize() = 0;

    /// Transform a pose from one frame to another.
    /**
     * \param pose_in The pose to be transformed (x, y, z, qx, qy, qz, qw)
     * \param source_frame The source frame name
     * \param target_frame The target frame name
     * \param time The time at which to get the transform (nanoseconds since
     * epoch) \returns The transformed pose (x, y, z, qx, qy, qz, qw)
     */
    virtual bool transform(const std::string& source_frame,
                           const std::string& target_frame, double x, double y,
                           double z, double qx, double qy, double qz, double qw,
                           double& out_x, double& out_y, double& out_z,
                           double& out_qx, double& out_qy, double& out_qz,
                           double& out_qw, uint64_t time = 0) = 0;

    /// Check if a transform is available.
    /**
     * \param source_frame The source frame name
     * \param target_frame The target frame name
     * \param time The time at which to check (nanoseconds since epoch, 0 for
     * latest) \returns True if transform is available
     */
    virtual bool canTransform(const std::string& source_frame,
                              const std::string& target_frame,
                              uint64_t time = 0) = 0;

    /// A getter for the internal implementation object.
    virtual TransformationLibraryConnector::WeakPtr getConnector() = 0;

    /// Checks that a given frame exists and can be used.
    /**
     * \param frame The frame to check
     * \param error An out string in which an error message (if generated) is
     * saved \returns True if the given frame has some problem
     */
    virtual bool frameHasProblems(const std::string& frame,
                                  std::string& error) const = 0;

    /// Return the class id set by the PluginlibFactory.
    virtual QString getClassId() const {
        return class_id_;
    }

    /// Used by PluginlibFactory to store the class id.
    virtual void setClassId(const QString& class_id) {
        class_id_ = class_id;
    }

    /// Return the description set by the PluginlibFactory.
    virtual QString getDescription() const {
        return description_;
    }

    /// Used by PluginlibFactory to store the description.
    virtual void setDescription(const QString& description) {
        description_ = description;
    }

private:
    QString class_id_;
    QString description_;
};

}  // namespace transformation
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__TRANSFORMATION__FRAME_TRANSFORMER_HPP_
