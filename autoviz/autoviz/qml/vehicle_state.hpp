/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * TF-driven vehicle pose for Autoviz.Vehicle3D QML (QGC Viewer3D pattern, zero MAVLink).
 *****************************************************************************/

#pragma once

#include <QObject>
#include <QString>

namespace autoviz::qml_vehicle {

/** Matches VehicleViewer3D.qml model selector indices. */
enum class VehicleModelType {
  kGroundDiffDrive = 0,
  kGroundAckermann = 1,
  kDroneSimple = 2,
  kDroneF450 = 3,
};

/** Pose + model selection exposed to Autoviz.Vehicle3D QML. */
class VehicleState : public QObject {
  Q_OBJECT
  Q_PROPERTY(double x READ x WRITE setX NOTIFY xChanged)
  Q_PROPERTY(double y READ y WRITE setY NOTIFY yChanged)
  Q_PROPERTY(double z READ z WRITE setZ NOTIFY zChanged)
  Q_PROPERTY(double roll READ roll WRITE setRoll NOTIFY rollChanged)
  Q_PROPERTY(double pitch READ pitch WRITE setPitch NOTIFY pitchChanged)
  Q_PROPERTY(double yaw READ yaw WRITE setYaw NOTIFY yawChanged)
  Q_PROPERTY(int motionMode READ motionMode WRITE setMotionMode NOTIFY motionModeChanged)
  Q_PROPERTY(double linearSpeed READ linearSpeed WRITE setLinearSpeed NOTIFY linearSpeedChanged)
  Q_PROPERTY(double angularSpeed READ angularSpeed WRITE setAngularSpeed NOTIFY angularSpeedChanged)
  Q_PROPERTY(int modelType READ modelType WRITE setModelType NOTIFY modelTypeChanged)
  Q_PROPERTY(double sceneScale READ sceneScale WRITE setSceneScale NOTIFY sceneScaleChanged)
  Q_PROPERTY(QString label READ label WRITE setLabel NOTIFY labelChanged)
  Q_PROPERTY(bool valid READ valid WRITE setValid NOTIFY validChanged)
  Q_PROPERTY(bool useF450Mesh READ useF450Mesh WRITE setUseF450Mesh NOTIFY useF450MeshChanged)

 public:
  explicit VehicleState(QObject* parent = nullptr);

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double roll() const { return roll_; }
  double pitch() const { return pitch_; }
  double yaw() const { return yaw_; }
  int motionMode() const { return motion_mode_; }
  double linearSpeed() const { return linear_speed_; }
  double angularSpeed() const { return angular_speed_; }
  int modelType() const { return static_cast<int>(model_type_); }
  double sceneScale() const { return scene_scale_; }
  const QString& label() const { return label_; }
  bool valid() const { return valid_; }
  bool useF450Mesh() const { return use_f450_mesh_; }

  void setX(double value);
  void setY(double value);
  void setZ(double value);
  void setRoll(double value);
  void setPitch(double value);
  void setYaw(double value);
  void setMotionMode(int value);
  void setLinearSpeed(double value);
  void setAngularSpeed(double value);
  void setModelType(int value);
  void setSceneScale(double value);
  void setLabel(const QString& value);
  void setValid(bool value);
  void setUseF450Mesh(bool value);

  VehicleModelType modelTypeEnum() const { return model_type_; }
  void setModelTypeEnum(VehicleModelType type);

 signals:
  void xChanged();
  void yChanged();
  void zChanged();
  void rollChanged();
  void pitchChanged();
  void yawChanged();
  void motionModeChanged();
  void linearSpeedChanged();
  void angularSpeedChanged();
  void modelTypeChanged();
  void sceneScaleChanged();
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
  int motion_mode_ = 0;
  double linear_speed_ = 0.0;
  double angular_speed_ = 0.0;
  VehicleModelType model_type_ = VehicleModelType::kGroundDiffDrive;
  double scene_scale_ = 1.0;
  QString label_;
  bool valid_ = false;
  bool use_f450_mesh_ = false;
};

}  // namespace autoviz::qml_vehicle
