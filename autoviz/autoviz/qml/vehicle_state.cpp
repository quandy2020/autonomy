/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/qml/vehicle_state.hpp"

namespace autoviz::qml_vehicle {

VehicleState::VehicleState(QObject* parent) : QObject(parent) {}

void VehicleState::setX(double value) {
  if (x_ == value) {
    return;
  }
  x_ = value;
  emit xChanged();
}

void VehicleState::setY(double value) {
  if (y_ == value) {
    return;
  }
  y_ = value;
  emit yChanged();
}

void VehicleState::setZ(double value) {
  if (z_ == value) {
    return;
  }
  z_ = value;
  emit zChanged();
}

void VehicleState::setRoll(double value) {
  if (roll_ == value) {
    return;
  }
  roll_ = value;
  emit rollChanged();
}

void VehicleState::setPitch(double value) {
  if (pitch_ == value) {
    return;
  }
  pitch_ = value;
  emit pitchChanged();
}

void VehicleState::setYaw(double value) {
  if (yaw_ == value) {
    return;
  }
  yaw_ = value;
  emit yawChanged();
}

void VehicleState::setMotionMode(int value) {
  if (motion_mode_ == value) {
    return;
  }
  motion_mode_ = value;
  emit motionModeChanged();
}

void VehicleState::setLinearSpeed(double value) {
  if (linear_speed_ == value) {
    return;
  }
  linear_speed_ = value;
  emit linearSpeedChanged();
}

void VehicleState::setAngularSpeed(double value) {
  if (angular_speed_ == value) {
    return;
  }
  angular_speed_ = value;
  emit angularSpeedChanged();
}

void VehicleState::setModelType(int value) {
  const auto typed = static_cast<VehicleModelType>(value);
  if (model_type_ == typed) {
    return;
  }
  model_type_ = typed;
  emit modelTypeChanged();
}

void VehicleState::setModelTypeEnum(VehicleModelType type) {
  setModelType(static_cast<int>(type));
}

void VehicleState::setSceneScale(double value) {
  if (scene_scale_ == value) {
    return;
  }
  scene_scale_ = value;
  emit sceneScaleChanged();
}

void VehicleState::setLabel(const QString& value) {
  if (label_ == value) {
    return;
  }
  label_ = value;
  emit labelChanged();
}

void VehicleState::setValid(bool value) {
  if (valid_ == value) {
    return;
  }
  valid_ = value;
  emit validChanged();
}

void VehicleState::setUseF450Mesh(bool value) {
  if (use_f450_mesh_ == value) {
    return;
  }
  use_f450_mesh_ = value;
  emit useF450MeshChanged();
}

}  // namespace autoviz::qml_vehicle
