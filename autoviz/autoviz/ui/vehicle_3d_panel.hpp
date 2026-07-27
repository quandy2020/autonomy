/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QWidget>
#include <memory>
#include <string>
#include <vector>

class QQuickWidget;
class QShowEvent;

namespace autoviz {
class VehicleStateBridge;
namespace display {
class Display;
}
namespace qml_vehicle {
class VehicleState;
class MissionPlanModel;
}
namespace transform {
class Buffer;
}

/** Qt Quick 3D dock — ground robots, drones, TF-driven (QGC Viewer3D style). */
class Vehicle3DPanel : public QWidget {
  Q_OBJECT

 public:
  explicit Vehicle3DPanel(QWidget* parent = nullptr);
  ~Vehicle3DPanel() override;

  void setTfBuffer(transform::Buffer* buffer);
  void setFixedFrame(const std::string& frame);
  void updateFromTf();
  void syncRootLinkFromDisplays(
      const std::vector<display::Display*>& displays);

  std::string sourceFrame() const;
  void setSourceFrame(const std::string& frame);

 public slots:
  void onQmlSourceFrameChanged(const QString& frame);
  void onQmlModelTypeChanged(int model_type);
  void onResetOriginRequested();

 signals:
  void sourceFrameChanged(const QString& frame);

 private:
  void setupUi();
  void ensureQmlLoaded();
  void detectF450Meshes();
  void syncRootProperties();

  void showEvent(QShowEvent* event) override;

  qml_vehicle::VehicleState* vehicle_state_ = nullptr;
  qml_vehicle::MissionPlanModel* mission_plan_model_ = nullptr;
  std::unique_ptr<VehicleStateBridge> bridge_;
  transform::Buffer* tf_buffer_ = nullptr;
  std::string fixed_frame_;
  QQuickWidget* quick_widget_ = nullptr;
  bool qml_loaded_ = false;
};

using Drone3DPanel = Vehicle3DPanel;

}  // namespace autoviz
