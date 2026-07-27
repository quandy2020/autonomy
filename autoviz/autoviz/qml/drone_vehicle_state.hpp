/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Vehicle pose bridge for QML drone preview (adapted from QGroundControl Viewer3D).
 *****************************************************************************/

#pragma once

#include <QObject>
#include <QString>

namespace autoviz::qml_drone {

/** Pose + mode exposed to Autoviz.Vehicle3D QML (replaces QGC Vehicle Facts). */
class DroneVehicleState : public QObject {
  Q_OBJECT
  Q_PROPERTY(double x READ x WRITE setX NOTIFY xChanged)
  Q_PROPERTY(double y READ y WRITE setY NOTIFY yChanged)
  Q_PROPERTY(double z READ z WRITE setZ NOTIFY zChanged)
  Q_PROPERTY(double roll READ roll WRITE setRoll NOTIFY rollChanged)
  Q_PROPERTY(double pitch READ pitch WRITE setPitch NOTIFY pitchChanged)
  Q_PROPERTY(double yaw READ yaw WRITE setYaw NOTIFY yawChanged)
  Q_PROPERTY(int flightMode READ flightMode WRITE setFlightMode NOTIFY flightModeChanged)
  Q_PROPERTY(QString label READ label WRITE setLabel NOTIFY labelChanged)
  Q_PROPERTY(bool valid READ valid WRITE setValid NOTIFY validChanged)
  Q_PROPERTY(bool useF450Mesh READ useF450Mesh WRITE setUseF450Mesh NOTIFY useF450MeshChanged)

 public:
  explicit DroneVehicleState(QObject* parent = nullptr);

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double roll() const { return roll_; }
  double pitch() const { return pitch_; }
  double yaw() const { return yaw_; }
  int flightMode() const { return flight_mode_; }
  const QString& label() const { return label_; }
  bool valid() const { return valid_; }
  bool useF450Mesh() const { return use_f450_mesh_; }

  void setX(double value);
  void setY(double value);
  void setZ(double value);
  void setRoll(double value);
  void setPitch(double value);
  void setYaw(double value);
  void setFlightMode(int value);
  void setLabel(const QString& value);
  void setValid(bool value);
  void setUseF450Mesh(bool value);

 signals:
  void xChanged();
  void yChanged();
  void zChanged();
  void rollChanged();
  void pitchChanged();
  void yawChanged();
  void flightModeChanged();
  void labelChanged();
  void validChanged();
  void useF450MeshChanged();

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double z_ = 0.0;
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;
  int flight_mode_ = 0;
  QString label_;
  bool valid_ = false;
  bool use_f450_mesh_ = false;
};

}  // namespace autoviz::qml_drone
