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

#ifndef AVIZ_COMMON__FRAME_MANAGER_IFACE_HPP_
#define AVIZ_COMMON__FRAME_MANAGER_IFACE_HPP_

#include <QObject>      // NOLINT: cpplint is unable to handle the include order here
#include <QQuaternion>  // NOLINT: cpplint is unable to handle the include order here
#include <QVector3D>    // NOLINT: cpplint is unable to handle the include order here
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "autonomy/tools/aviz/common/transformation/frame_transformer.hpp"

namespace aviz {
namespace common {

class Display;

/// Helper class for transforming data into the fixed frame.
/**
 * During one frame update (nominally 33ms), the transform tree stays consistent and
 * queries are cached for speedup.
 */
class FrameManagerIface : public QObject {
  Q_OBJECT

 public:
  enum SyncMode { SyncOff = 0, SyncExact, SyncApprox };

  /// Set the frame to consider "fixed", into which incoming data is transformed.
  /**
   * The fixed frame serves as the reference for all getTransform() and
   * transform() functions in FrameManager.
   */
  virtual void setFixedFrame(const std::string& frame) = 0;

  /// Enable/disable pause mode.
  virtual void setPause(bool pause) = 0;

  /// Get pause state.
  virtual bool getPause() = 0;

  /// Set synchronization mode (off/exact/approximate).
  virtual void setSyncMode(SyncMode mode) = 0;

  /// Get the synchronization mode.
  virtual SyncMode getSyncMode() = 0;

  /// Synchronize with given time (nanoseconds since epoch).
  virtual void syncTime(uint64_t time_ns) = 0;

  /// Get current time, depending on the sync mode (nanoseconds since epoch).
  virtual uint64_t getTime() = 0;

  /// Return the pose for a frame relative to the fixed frame, at current time.
  /**
   * \param[in] frame The frame to find the pose of.
   * \param[out] position The position of the frame relative to the fixed frame.
   * \param[out] orientation The orientation of the frame relative to the fixed frame.
   * \return true on success, false on failure.
   */
  virtual bool getTransform(const std::string& frame, QVector3D& position, QQuaternion& orientation) = 0;

  /// Return the pose for a frame relative to the fixed frame, at a given time.
  /**
   * \param[in] frame The frame to find the pose of.
   * \param[in] time_ns The time at which to get the pose (nanoseconds since epoch).
   * \param[out] position The position of the frame relative to the fixed frame.
   * \param[out] orientation The orientation of the frame relative to the fixed frame.
   * \return true on success, false on failure.
   */
  virtual bool getTransform(const std::string& frame, uint64_t time_ns, QVector3D& position,
                            QQuaternion& orientation) = 0;

  /// Transform a pose from a frame into the fixed frame.
  /**
   * \param[in] frame The input frame.
   * \param[in] time_ns The time at which to get the pose (nanoseconds since epoch).
   * \param[in] x, y, z The input position, relative to the input frame.
   * \param[in] qx, qy, qz, qw The input orientation (quaternion), relative to the input frame.
   * \param[out] out_x, out_y, out_z Position part of pose relative to the fixed frame.
   * \param[out] out_qx, out_qy, out_qz, out_qw Orientation part of pose relative to the fixed frame.
   * \return true on success, false on failure.
   */
  virtual bool transform(const std::string& frame, uint64_t time_ns, double x, double y, double z, double qx, double qy,
                         double qz, double qw, double& out_x, double& out_y, double& out_z, double& out_qx,
                         double& out_qy, double& out_qz, double& out_qw) = 0;

  /// Clear the internal cache.
  virtual void update() = 0;

  /// Check to see if a frame exists.
  /**
   * \param[in] frame The name of the frame to check.
   * \param[out] error If the frame does not exist, an error message is stored here.
   * \return true if the frame does not exist, false if it does exist.
   */
  virtual bool frameHasProblems(const std::string& frame, std::string& error) = 0;

  /// Check to see if a transform is known between a given frame and the fixed frame.
  /**
   * \param[in] frame The name of the frame to check.
   * \param[out] error If the transform is not known, an error message is stored here.
   * \return true if the transform is not known, false if it is.
   */
  virtual bool transformHasProblems(const std::string& frame, std::string& error) = 0;

  /// Check to see if a transform is known between a given frame and the fixed frame.
  /**
   * \param[in] frame The name of the frame to check.
   * \param[in] time_ns The time at which the transform is desired (nanoseconds since epoch).
   * \param[out] error If the transform is not known, an error message is stored here.
   * \return true if the transform is not known, false if it is.
   */
  virtual bool transformHasProblems(const std::string& frame, uint64_t time_ns, std::string& error) = 0;

  /// Return the current fixed frame name.
  virtual const std::string& getFixedFrame() = 0;

  /// Return a weak pointer to the internal transformation object.
  virtual transformation::TransformationLibraryConnector::WeakPtr getConnector() = 0;

  /// Return a shared pointer to the transformer object.
  virtual std::shared_ptr<transformation::FrameTransformer> getTransformer() = 0;

  virtual std::vector<std::string> getAllFrameNames() = 0;

 public Q_SLOTS:
  virtual void setTransformerPlugin(std::shared_ptr<transformation::FrameTransformer> transformer) = 0;

 Q_SIGNALS:
  void fixedFrameChanged();
};

}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__FRAME_MANAGER_IFACE_HPP_
