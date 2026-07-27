/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/qml/drone_vehicle_state.hpp"

namespace autoviz::qml_drone {

DroneVehicleState::DroneVehicleState(QObject* parent) : QObject(parent) {}

void DroneVehicleState::setX(double value) {
  if (x_ == value) {
    return;
  }
  x_ = value;
  emit xChanged();
}

void DroneVehicleState::setY(double value) {
  if (y_ == value) {
    return;
  }
  y_ = value;
  emit yChanged();
}

void DroneVehicleState::setZ(double value) {
  if (z_ == value) {
    return;
  }
  z_ = value;
  emit zChanged();
}

void DroneVehicleState::setRoll(double value) {
  if (roll_ == value) {
    return;
  }
  roll_ = value;
  emit rollChanged();
}

void DroneVehicleState::setPitch(double value) {
  if (pitch_ == value) {
    return;
  }
  pitch_ = value;
  emit pitchChanged();
}

void DroneVehicleState::setYaw(double value) {
  if (yaw_ == value) {
    return;
  }
  yaw_ = value;
  emit yawChanged();
}

void DroneVehicleState::setFlightMode(int value) {
  if (flight_mode_ == value) {
    return;
  }
  flight_mode_ = value;
  emit flightModeChanged();
}

void DroneVehicleState::setLabel(const QString& value) {
  if (label_ == value) {
    return;
  }
  label_ = value;
  emit labelChanged();
}

void DroneVehicleState::setValid(bool value) {
  if (valid_ == value) {
    return;
  }
  valid_ = value;
  emit validChanged();
}

void DroneVehicleState::setUseF450Mesh(bool value) {
  if (use_f450_mesh_ == value) {
    return;
  }
  use_f450_mesh_ = value;
  emit useF450MeshChanged();
}

}  // namespace autoviz::qml_drone
