/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins robot::TFLinkUpdater
 * (Willow Garage / Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/robot/tf_link_updater.hpp"

#include <QMatrix3x3>

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_context.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/transform/buffer.hpp"

namespace autoviz {
namespace display {
namespace robot {

TFLinkUpdater::TFLinkUpdater(common::DisplayContext* context,
                             StatusCallback status_cb, std::string tf_prefix)
    : context_(context),
      status_callback_(std::move(status_cb)),
      tf_prefix_(std::move(tf_prefix)) {}

bool TFLinkUpdater::getLinkTransforms(
    const std::string& link_name, QVector3D* visual_position,
    QQuaternion* visual_orientation, QVector3D* collision_position,
    QQuaternion* collision_orientation) const {
  if (visual_position == nullptr || visual_orientation == nullptr ||
      collision_position == nullptr || collision_orientation == nullptr) {
    return false;
  }
  if (context_ == nullptr || context_->tf_buffer == nullptr ||
      link_name.empty()) {
    return false;
  }

  std::string frame = link_name;
  if (!tf_prefix_.empty()) {
    frame = tf_prefix_ + "/" + link_name;
  }

  try {
    const QMatrix4x4 matrix = transformToMatrix(context_->tf_buffer->lookupTransform(
        context_->fixed_frame, frame, autoviz::commsgs::ZeroTime()));
    const QVector3D position = matrix.map(QVector3D(0.f, 0.f, 0.f));
    const QMatrix3x3 rot = matrix.toGenericMatrix<3, 3>();
    const QQuaternion orientation =
        QQuaternion::fromRotationMatrix(rot).normalized();

    *visual_position = position;
    *visual_orientation = orientation;
    *collision_position = position;
    *collision_orientation = orientation;
    setLinkStatus(StatusLevel::kOk, frame, "Transform OK");
    return true;
  } catch (...) {
    setLinkStatus(StatusLevel::kError, frame,
                  "No transform from [" + frame + "] to [" +
                      context_->fixed_frame + "]");
    return false;
  }
}

void TFLinkUpdater::setLinkStatus(StatusLevel level,
                                  const std::string& link_name,
                                  const std::string& text) const {
  if (status_callback_) {
    status_callback_(level, link_name, text);
  }
}

}  // namespace robot
}  // namespace display
}  // namespace autoviz
