/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/vehicle_3d_panel.hpp"

#include <QDebug>
#include <QFile>
#include <QHBoxLayout>
#include <QLabel>
#include <QLibraryInfo>
#include <QQmlContext>
#include <QQmlEngine>
#include <QQuickItem>
#include <QQuickWidget>
#include <QShowEvent>
#include <QUrl>

#include "autoviz/display/display.hpp"
#include "autoviz/platform/opengl_setup.hpp"
#include "autoviz/qml/mission_plan_model.hpp"
#include "autoviz/qml/vehicle_state.hpp"
#include "autoviz/ui/vehicle_state_bridge.hpp"

namespace autoviz {

Vehicle3DPanel::Vehicle3DPanel(QWidget* parent)
    : QWidget(parent),
      vehicle_state_(new qml_vehicle::VehicleState(this)),
      mission_plan_model_(new qml_vehicle::MissionPlanModel(this)),
      bridge_(std::make_unique<VehicleStateBridge>()) {
  vehicle_state_->setModelTypeEnum(qml_vehicle::VehicleModelType::kGroundDiffDrive);
  setupUi();
  detectF450Meshes();
}

Vehicle3DPanel::~Vehicle3DPanel() = default;

void Vehicle3DPanel::setupUi() {
  auto* layout = new QHBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);

  quick_widget_ = new QQuickWidget(this);
  quick_widget_->setFormat(platform::defaultSurfaceFormat());
  quick_widget_->setResizeMode(QQuickWidget::SizeRootObjectToView);
  quick_widget_->setClearColor(QColor(0x14, 0x18, 0x24));
  if (QQmlEngine* engine = quick_widget_->engine()) {
    engine->addImportPath(QLibraryInfo::path(QLibraryInfo::QmlImportsPath));
    engine->addImportPath(QStringLiteral("qrc:/qml"));
  }
  quick_widget_->rootContext()->setContextProperty(
      QStringLiteral("autovizVehicleState"), vehicle_state_);
  quick_widget_->rootContext()->setContextProperty(
      QStringLiteral("autovizDroneState"), vehicle_state_);
  quick_widget_->rootContext()->setContextProperty(
      QStringLiteral("autovizMissionPlan"), mission_plan_model_);
  layout->addWidget(quick_widget_);

  connect(quick_widget_, &QQuickWidget::statusChanged, this, [this](QQuickWidget::Status status) {
    if (status != QQuickWidget::Ready || quick_widget_->rootObject() == nullptr) {
      return;
    }
    QQuickItem* root = quick_widget_->rootObject();
    connect(root, SIGNAL(sourceFrameChanged(QString)), this,
            SLOT(onQmlSourceFrameChanged(QString)));
    connect(root, SIGNAL(modelTypeChanged(int)), this,
            SLOT(onQmlModelTypeChanged(int)));
    connect(root, SIGNAL(resetOriginRequested()), this,
            SLOT(onResetOriginRequested()));
    bridge_->setSourceFrame(QStringLiteral("base_link").toStdString());
    syncRootProperties();
  });
}

void Vehicle3DPanel::ensureQmlLoaded() {
  if (qml_loaded_ || quick_widget_ == nullptr) {
    return;
  }
  qml_loaded_ = true;
  quick_widget_->setSource(QUrl(
      QStringLiteral("qrc:/qml/Autoviz/Vehicle3D/Models3D/VehicleViewer3D.qml")));
  if (quick_widget_->status() == QQuickWidget::Error) {
    qWarning() << "Vehicle3D QML load failed:"
               << quick_widget_->errors();
  }
}

void Vehicle3DPanel::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  ensureQmlLoaded();
}

void Vehicle3DPanel::syncRootProperties() {
  if (quick_widget_ == nullptr || quick_widget_->rootObject() == nullptr ||
      vehicle_state_ == nullptr) {
    return;
  }
  QQuickItem* root = quick_widget_->rootObject();
  root->setProperty("sourceFrame", QString::fromStdString(bridge_->sourceFrame()));
  root->setProperty("modelType", vehicle_state_->modelType());
}

void Vehicle3DPanel::onQmlSourceFrameChanged(const QString& frame) {
  if (bridge_ != nullptr) {
    bridge_->setSourceFrame(frame.toStdString());
    bridge_->resetOrigin();
  }
  emit sourceFrameChanged(frame);
}

void Vehicle3DPanel::onQmlModelTypeChanged(int model_type) {
  if (vehicle_state_ == nullptr) {
    return;
  }
  vehicle_state_->setModelType(model_type);
  if (model_type == static_cast<int>(qml_vehicle::VehicleModelType::kDroneF450)) {
    detectF450Meshes();
  }
  if (bridge_ != nullptr) {
    bridge_->resetOrigin();
  }
}

void Vehicle3DPanel::onResetOriginRequested() {
  if (bridge_ != nullptr) {
    bridge_->resetOrigin();
  }
}

void Vehicle3DPanel::detectF450Meshes() {
  const QString probe =
      QStringLiteral(":/qml/Autoviz/Vehicle3D/Djif450/DroneModel_arm_1/node.mesh");
  const bool has_mesh = QFile::exists(probe);
  if (vehicle_state_ != nullptr) {
    vehicle_state_->setUseF450Mesh(has_mesh);
    if (!has_mesh &&
        vehicle_state_->modelTypeEnum() ==
            qml_vehicle::VehicleModelType::kDroneF450) {
      vehicle_state_->setModelTypeEnum(qml_vehicle::VehicleModelType::kDroneSimple);
    }
  }
}

void Vehicle3DPanel::setTfBuffer(transform::Buffer* buffer) {
  tf_buffer_ = buffer;
}

void Vehicle3DPanel::setFixedFrame(const std::string& frame) {
  if (fixed_frame_ != frame) {
    fixed_frame_ = frame;
    if (bridge_ != nullptr) {
      bridge_->resetOrigin();
    }
  }
}

void Vehicle3DPanel::updateFromTf() {
  if (bridge_ == nullptr || vehicle_state_ == nullptr) {
    return;
  }
  bridge_->updateFromTf(fixed_frame_, tf_buffer_, vehicle_state_);
}

std::string Vehicle3DPanel::sourceFrame() const {
  return bridge_ != nullptr ? bridge_->sourceFrame() : std::string{};
}

void Vehicle3DPanel::setSourceFrame(const std::string& frame) {
  if (bridge_ == nullptr) {
    return;
  }
  bridge_->setSourceFrame(frame);
  syncRootProperties();
}

void Vehicle3DPanel::syncRootLinkFromDisplays(
    const std::vector<display::Display*>& displays) {
  if (bridge_ == nullptr) {
    return;
  }
  for (display::Display* display : displays) {
    if (display == nullptr || !display->enabled() ||
        display->typeId() != "RobotModel") {
      continue;
    }
    const std::string root_link =
        display->propertyValue("root_link", std::string{});
    if (!root_link.empty() && bridge_->sourceFrame() != root_link) {
      setSourceFrame(root_link);
    }
    return;
  }
}

}  // namespace autoviz
