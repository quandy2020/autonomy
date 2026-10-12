/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/tools/select/select_tool.hpp"

#include <QKeyEvent>
#include <QMouseEvent>

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <vector>

#include "autoviz/common/selection_manager.hpp"
#include "autoviz/plugins/displays/common/screen_projection.hpp"
#include "autoviz/rendering/pick_utils.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

#include <QMatrix4x4>

namespace autoviz {
namespace tools {
namespace {

constexpr float kSelectionRectEyeDepth = 2.f;
constexpr int kTinyRectPixels = 4;
constexpr int kMaxRectSamples = 200;

struct PickIdentity {
  common::PickHandle handle = common::kInvalidPickHandle;
  std::string display_name;
  int point_index = -1;

  bool operator==(const PickIdentity& other) const {
    if (handle != common::kInvalidPickHandle &&
        other.handle != common::kInvalidPickHandle) {
      return handle == other.handle && point_index == other.point_index;
    }
    if (!display_name.empty() && display_name == other.display_name) {
      return point_index == other.point_index;
    }
    return false;
  }
};

struct PickIdentityHash {
  size_t operator()(const PickIdentity& id) const {
    return std::hash<common::PickHandle>()(id.handle) ^
           (std::hash<std::string>()(id.display_name) << 1) ^
           (std::hash<int>()(id.point_index) << 2);
  }
};

PickIdentity IdentityFromEntry(const common::SelectionEntry& entry) {
  return {entry.pick_handle, entry.display_name, entry.point_index};
}

PickIdentity IdentityFromPick(const rendering::PickResult& pick) {
  return {pick.pick_handle, pick.display_name, pick.point_index};
}

common::SelectionEntry EntryFromPick(const rendering::PickResult& pick) {
  common::SelectionEntry entry;
  entry.position = pick.position;
  entry.display_name = pick.display_name;
  entry.display_type = pick.display_type;
  entry.pick_handle = pick.pick_handle;
  entry.point_index = pick.point_index;
  entry.properties = pick.properties;
  return entry;
}

}  // namespace

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

void SelectTool::drawSelectionRectangle(rendering::SceneOverlay& scene) const {
  if (!selecting_ || context() == nullptr ||
      context()->display_context == nullptr) {
    return;
  }
  const int min_x = std::min(sel_start_x_, sel_end_x_);
  const int max_x = std::max(sel_start_x_, sel_end_x_);
  const int min_y = std::min(sel_start_y_, sel_end_y_);
  const int max_y = std::max(sel_start_y_, sel_end_y_);
  const common::DisplayContext* display_context = context()->display_context;
  QVector3D corners[4];
  const float xs[2] = {static_cast<float>(min_x), static_cast<float>(max_x)};
  const float ys[2] = {static_cast<float>(min_y), static_cast<float>(max_y)};
  int idx = 0;
  for (float x : xs) {
    for (float y : ys) {
      if (!display::unprojectScreenToWorld(display_context, x, y,
                                           kSelectionRectEyeDepth,
                                           &corners[idx++])) {
        return;
      }
    }
  }
  const std::vector<QVector3D> loop = {corners[0], corners[1], corners[3],
                                         corners[2]};
  scene.addLineLoop(loop, QColor(255, 255, 0, 200), 1.5f);
}

common::SelectionManager* SelectTool::selectionManager() const {
  if (context() == nullptr) {
    return nullptr;
  }
  return context()->selection_manager;
}

const std::vector<common::SelectionEntry>& SelectTool::selections() const {
  static const std::vector<common::SelectionEntry> kEmpty;
  if (common::SelectionManager* manager = selectionManager()) {
    return manager->selection();
  }
  return local_selections_;
}

common::SelectionManager::SelectMode SelectTool::modeFromModifiers(
    Qt::KeyboardModifiers modifiers) {
  if (modifiers.testFlag(Qt::ShiftModifier)) {
    return common::SelectionManager::SelectMode::kAdd;
  }
  if (modifiers.testFlag(Qt::ControlModifier)) {
    return common::SelectionManager::SelectMode::kRemove;
  }
  return common::SelectionManager::SelectMode::kReplace;
}

void SelectTool::notifySelectionsChanged() {
  if (context() != nullptr && context()->selections_changed) {
    context()->selections_changed(local_selections_);
  }
}

void SelectTool::applyPickResult(const rendering::PickResult& pick,
                                 common::SelectionManager::SelectMode mode) {
  if (!pick.hit) {
    if (mode == common::SelectionManager::SelectMode::kReplace) {
      if (common::SelectionManager* manager = selectionManager()) {
        manager->clear();
      } else {
        local_selections_.clear();
        notifySelectionsChanged();
      }
    }
    return;
  }
  const common::SelectionEntry entry = EntryFromPick(pick);
  if (common::SelectionManager* manager = selectionManager()) {
    manager->select(entry, mode);
  } else {
    if (mode == common::SelectionManager::SelectMode::kReplace) {
      local_selections_ = {entry};
    } else if (mode == common::SelectionManager::SelectMode::kAdd) {
      local_selections_.push_back(entry);
    } else {
      const PickIdentity target = IdentityFromEntry(entry);
      const auto it = std::remove_if(
          local_selections_.begin(), local_selections_.end(),
          [&target](const common::SelectionEntry& item) {
            return IdentityFromEntry(item) == target;
          });
      local_selections_.erase(it, local_selections_.end());
    }
    notifySelectionsChanged();
  }
}

void SelectTool::selectInRectangle(int x0, int y0, int x1, int y1,
                                   common::SelectionManager::SelectMode mode) {
  if (context() == nullptr) {
    return;
  }
  const int min_x = std::min(x0, x1);
  const int max_x = std::max(x0, x1);
  const int min_y = std::min(y0, y1);
  const int max_y = std::max(y0, y1);
  const int width = max_x - min_x;
  const int height = max_y - min_y;

  if (width <= kTinyRectPixels && height <= kTinyRectPixels) {
    const rendering::PickResult pick = rendering::pickAtToolContext(
        *context(), x1, y1);
    applyPickResult(pick, mode);
    return;
  }

  const int span = std::max(width, height);
  int step = std::clamp(span / 32, 4, 8);
  std::vector<common::SelectionEntry> picked;
  picked.reserve(kMaxRectSamples);
  std::unordered_set<PickIdentity, PickIdentityHash> seen;
  int samples = 0;

  for (int y = min_y; y <= max_y && samples < kMaxRectSamples; y += step) {
    for (int x = min_x; x <= max_x && samples < kMaxRectSamples; x += step) {
      const rendering::PickResult pick =
          rendering::pickAtToolContext(*context(), x, y);
      ++samples;
      if (!pick.hit) {
        continue;
      }
      const PickIdentity id = IdentityFromPick(pick);
      if (seen.count(id) != 0) {
        continue;
      }
      seen.insert(id);
      picked.push_back(EntryFromPick(pick));
    }
  }

  common::SelectionManager* manager = selectionManager();
  if (mode == common::SelectionManager::SelectMode::kReplace) {
    if (manager != nullptr) {
      manager->setSelection(std::move(picked));
    } else {
      local_selections_ = std::move(picked);
      notifySelectionsChanged();
    }
    return;
  }

  for (const common::SelectionEntry& entry : picked) {
    if (manager != nullptr) {
      manager->select(entry, mode);
    } else if (mode == common::SelectionManager::SelectMode::kAdd) {
      local_selections_.push_back(entry);
    } else {
      const PickIdentity target = IdentityFromEntry(entry);
      const auto it = std::remove_if(
          local_selections_.begin(), local_selections_.end(),
          [&target](const common::SelectionEntry& item) {
            return IdentityFromEntry(item) == target;
          });
      local_selections_.erase(it, local_selections_.end());
    }
  }
  if (manager == nullptr) {
    notifySelectionsChanged();
  }
}

bool SelectTool::keyPressEvent(QKeyEvent* event) {
  if (event == nullptr || event->key() != Qt::Key_F) {
    return false;
  }
  if (common::SelectionManager* manager = selectionManager()) {
    manager->focusOnSelection();
    if (context() != nullptr && context()->set_status) {
      context()->set_status(QStringLiteral("Select: focus on selection"));
    }
    return true;
  }
  return false;
}

bool SelectTool::mousePressEvent(QMouseEvent* event) {
  if (event->modifiers().testFlag(Qt::AltModifier)) {
    return false;
  }
  if (event->button() != Qt::LeftButton) {
    return false;
  }
  if (context() == nullptr || context()->view_controller == nullptr) {
    if (context() != nullptr && context()->set_status) {
      context()->set_status(QStringLiteral("Select: viewport not ready"));
    }
    return false;
  }
  selecting_ = true;
  sel_start_x_ = event->pos().x();
  sel_start_y_ = event->pos().y();
  sel_end_x_ = sel_start_x_;
  sel_end_y_ = sel_start_y_;
  return true;
}

bool SelectTool::mouseMoveEvent(QMouseEvent* event) {
  if (event->modifiers().testFlag(Qt::AltModifier)) {
    return false;
  }
  if (!selecting_) {
    return false;
  }
  sel_end_x_ = event->pos().x();
  sel_end_y_ = event->pos().y();
  if (context() != nullptr && context()->request_redraw) {
    context()->request_redraw();
  }
  return true;
}

bool SelectTool::mouseReleaseEvent(QMouseEvent* event) {
  if (event->modifiers().testFlag(Qt::AltModifier)) {
    selecting_ = false;
    return false;
  }
  if (event->button() != Qt::LeftButton || !selecting_) {
    return false;
  }
  selecting_ = false;
  sel_end_x_ = event->pos().x();
  sel_end_y_ = event->pos().y();

  const common::SelectionManager::SelectMode mode =
      modeFromModifiers(event->modifiers());
  selectInRectangle(sel_start_x_, sel_start_y_, sel_end_x_, sel_end_y_, mode);

  if (context() != nullptr && context()->request_redraw) {
    context()->request_redraw();
  }
  if (context() != nullptr && context()->set_status) {
    context()->set_status(statusText());
  }
  return true;
}

void SelectTool::onDraw(rendering::SceneOverlay& scene) {
  if (selecting_) {
    drawSelectionRectangle(scene);
  }
  for (const auto& entry : selections()) {
    drawSelectionMarker(scene, entry.position);
    const float s = 0.08f;
    scene.addBoxWireframe(entry.position, QVector3D(s, s, s), QMatrix4x4(),
                          QColor(255, 180, 60, 180));
  }
}

QString SelectTool::statusText() const {
  const auto& selected = selections();
  if (selected.empty()) {
    return QStringLiteral("Click and drag to select objects on the screen.");
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
  return QStringLiteral("Selected %1 objects").arg(selected.size());
}

}  // namespace tools
}  // namespace autoviz
