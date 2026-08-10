/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/tools/select_tool.hpp"

#include <QMouseEvent>

#include "autoviz/common/selection_manager.hpp"
#include "autoviz/rendering/pick_utils.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

#include <QMatrix4x4>

namespace autoviz {
namespace tools {

void SelectTool::drawSelectionMarker(rendering::SceneOverlay& scene,
                                     const QVector3D& position) const {
  const QColor color(255, 120, 40);
  const float s = 0.12f;
  scene.addPoint(position, color);
  scene.addLine(position - QVector3D(s, 0.f, 0.f),
                position + QVector3D(s, 0.f, 0.f), color);
  scene.addLine(position - QVector3D(0.f, s, 0.f),
                position + QVector3D(0.f, s, 0.f), color);
  scene.addLine(position - QVector3D(0.f, 0.f, s),
                position + QVector3D(0.f, 0.f, s), color);
}

common::SelectionManager* SelectTool::selectionManager() const {
  if (context() == nullptr) {
    return nullptr;
  }
  if (context()->selection_manager != nullptr) {
    return context()->selection_manager;
  }
  return nullptr;
}

const std::vector<common::SelectionEntry>& SelectTool::selections() const {
  static const std::vector<common::SelectionEntry> kEmpty;
  if (common::SelectionManager* manager = selectionManager()) {
    return manager->selection();
  }
  return local_selections_;
}

bool SelectTool::mousePressEvent(QMouseEvent* event) {
  // Middle / right / shift+left are for camera navigation — do not consume.
  if (event->button() != Qt::LeftButton ||
      event->modifiers().testFlag(Qt::ShiftModifier)) {
    return false;
  }
  if (context() == nullptr || context()->view_controller == nullptr ||
      context()->scene_overlay == nullptr) {
    if (context() != nullptr && context()->set_status) {
      context()->set_status(QStringLiteral("Select: viewport not ready"));
    }
    return false;
  }
  const rendering::PickResult pick = rendering::pickAtToolContext(
      *context(), event->pos().x(), event->pos().y());

  const bool additive = event->modifiers().testFlag(Qt::ControlModifier);
  common::SelectionManager* manager = selectionManager();

  if (!pick.hit) {
    if (!additive) {
      if (manager != nullptr) {
        manager->clear();
      } else {
        local_selections_.clear();
        notifySelectionsChanged();
      }
    }
    if (context()->set_status) {
      context()->set_status(statusText());
    }
    if (context()->request_redraw) {
      context()->request_redraw();
    }
    // Empty space: clear selection, then let Orbit/Pan handle the drag.
    return false;
  }

  common::SelectionEntry entry;
  entry.position = pick.position;
  entry.display_name = pick.display_name;
  entry.display_type = pick.display_type;
  entry.pick_handle = pick.pick_handle;
  entry.point_index = pick.point_index;
  entry.properties = pick.properties;

  if (manager != nullptr) {
    manager->select(
        entry, additive ? common::SelectionManager::SelectMode::kAdd
                        : common::SelectionManager::SelectMode::kReplace);
  } else {
    if (additive) {
      local_selections_.push_back(std::move(entry));
    } else {
      local_selections_ = {std::move(entry)};
    }
    notifySelectionsChanged();
  }

  if (context()->request_redraw) {
    context()->request_redraw();
  }
  if (context()->set_status) {
    context()->set_status(statusText());
  }
  return true;
}

void SelectTool::notifySelectionsChanged() {
  if (context() != nullptr && context()->selections_changed) {
    context()->selections_changed(local_selections_);
  }
}

void SelectTool::onDraw(rendering::SceneOverlay& scene) {
  for (const auto& entry : selections()) {
    drawSelectionMarker(scene, entry.position);
    const float s = 0.08f;
    scene.addBoxWireframe(
        entry.position, QVector3D(s, s, s), QMatrix4x4(),
        QColor(255, 180, 60, 180));
  }
}

QString SelectTool::statusText() const {
  const auto& selected = selections();
  if (selected.empty()) {
    return QStringLiteral(
        "Select: click geometry (Ctrl+add) · empty drag orbits · "
        "Middle/Shift+Left pan");
  }
  if (selected.size() == 1) {
    const auto& entry = selected.front();
    const QVector3D p = entry.position;
    if (entry.display_name.empty()) {
      return QStringLiteral("Selected: (%1, %2, %3)")
          .arg(p.x(), 0, 'f', 3)
          .arg(p.y(), 0, 'f', 3)
          .arg(p.z(), 0, 'f', 3);
    }
    return QStringLiteral("Selected %1 [%2]: (%3, %4, %5)")
        .arg(QString::fromStdString(entry.display_name))
        .arg(QString::fromStdString(entry.display_type))
        .arg(p.x(), 0, 'f', 3)
        .arg(p.y(), 0, 'f', 3)
        .arg(p.z(), 0, 'f', 3);
  }
  return QStringLiteral("Selected %1 points (Ctrl+click to add more)")
      .arg(selected.size());
}

}  // namespace tools
}  // namespace autoviz
