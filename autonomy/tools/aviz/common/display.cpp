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

#include "autonomy/tools/aviz/common/display.hpp"

#include <QApplication>  // NOLINT: cpplint is unable to handle the include order here
#include <QColor>        // NOLINT: cpplint is unable to handle the include order here
#include <QDockWidget>   // NOLINT: cpplint is unable to handle the include order here
#include <QFont>         // NOLINT: cpplint is unable to handle the include order here
#include <QMetaObject>   // NOLINT: cpplint is unable to handle the include order here
#include <QWidget>       // NOLINT: cpplint is unable to handle the include order here
#include <cstdio>
#include <string>

#include "autonomy/tools/aviz/common/display_context.hpp"
#include "autonomy/tools/aviz/common/logging.hpp"
#include "autonomy/tools/aviz/scene_viewer.hpp"

namespace aviz {
namespace common {

Display::Display()
    : context_(nullptr),
      scene_viewer_(nullptr),
      scene_manager_(nullptr),
      scene_node_(nullptr),
      fixed_frame_(""),
      class_id_(""),
      name_(""),
      initialized_(false),
      enabled_(false),
      visibility_bits_(0xFFFFFFFF),
      associated_widget_(nullptr),
      associated_widget_panel_(nullptr) {}

Display::~Display() {
  if (associated_widget_panel_) {
    // Cleanup will be handled by parent widget
    associated_widget_panel_ = nullptr;
  }
  if (scene_node_ && scene_manager_) {
    scene_manager_->destroySceneNode(scene_node_);
    scene_node_ = nullptr;
  }
}

void Display::initialize(DisplayContext* context) {
  context_ = context;
  if (context_) {
    scene_viewer_ = context_->getSceneViewer();
    scene_manager_ = context_->getSceneManager();
    fixed_frame_ = context_->getFixedFrame();

    if (scene_manager_) {
      scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    }
  }

  onInitialize();

  initialized_ = true;
}

void Display::queueRender() {
  if (context_) {
    context_->queueRender();
  }
}

QVariant Display::getViewData(int column, int role) const {
  switch (role) {
    case Qt::ForegroundRole: {
      if (isEnabled()) {
        // Check if there are any error/warn statuses
        bool has_error = false;
        bool has_warn = false;
        for (const auto& entry : status_map_) {
          if (entry.level == 2) {  // Error
            has_error = true;
            break;
          } else if (entry.level == 1) {  // Warn
            has_warn = true;
          }
        }
        if (has_error) {
          return QColor(255, 0, 0);  // Red for error
        } else if (has_warn) {
          return QColor(255, 165, 0);  // Orange for warn
        } else {
          // Blue means that the enabled checkmark is set
          return QColor(40, 120, 197);
        }
      } else {
        return QApplication::palette().color(QPalette::Text);
      }
      break;
    }
    case Qt::FontRole: {
      QFont font;
      if (isEnabled()) {
        font.setBold(true);
      }
      return font;
    }
    case Qt::DecorationRole: {
      if (column == 0) {
        if (isEnabled()) {
          // Check status
          bool has_error = false;
          bool has_warn = false;
          for (const auto& entry : status_map_) {
            if (entry.level == 2) {  // Error
              has_error = true;
              break;
            } else if (entry.level == 1) {  // Warn
              has_warn = true;
            }
          }
          if (has_error) {
            // Return error icon (can be extended later)
            return icon_;
          } else if (has_warn) {
            // Return warn icon (can be extended later)
            return icon_;
          } else {
            return icon_;
          }
        } else {
          return icon_;
        }
      }
      break;
    }
  }
  return QVariant();
}

Qt::ItemFlags Display::getViewFlags(int column) const {
  (void)column;
  return Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDragEnabled;
}

QString Display::getClassId() const { return class_id_; }

void Display::setClassId(const QString& class_id) { class_id_ = class_id; }

void Display::setTopic(const QString& topic, const QString& datatype) {
  (void)topic;
  (void)datatype;
}

void Display::update(float wall_dt, float ros_dt) {
  (void)wall_dt;
  (void)ros_dt;
}

void Display::setStatus(int level, const QString& name, const QString& text) {
  QMetaObject::invokeMethod(this, "setStatusInternal", Qt::QueuedConnection, Q_ARG(int, level), Q_ARG(QString, name),
                            Q_ARG(QString, text));
}

void Display::setStatusStd(int level, const std::string& name, const std::string& text) {
  setStatus(level, QString::fromStdString(name), QString::fromStdString(text));
}

void Display::setMissingTransformToFixedFrame(const std::string& frame, const std::string& additional_info) {
  std::string error_string = "Could not transform " +
                             (additional_info.empty() ? "from [" : additional_info + " from [") + frame + "] to [" +
                             fixed_frame_.toStdString() + "]";
  setStatusStd(2, "Transform", error_string);  // 2 = Error
}

void Display::setTransformOk() {
  setStatusStd(0, "Transform", "Ok");  // 0 = Ok
}

void Display::deleteStatusStd(const std::string& name) { deleteStatus(QString::fromStdString(name)); }

void Display::setStatusInternal(int level, const QString& name, const QString& text) {
  StatusEntry entry;
  entry.level = level;
  entry.name = name;
  entry.text = text;
  status_map_[name] = entry;

  // Emit signal or update UI if needed
  // This can be extended later with StatusList
}

void Display::deleteStatus(const QString& name) {
  QMetaObject::invokeMethod(this, "deleteStatusInternal", Qt::QueuedConnection, Q_ARG(QString, name));
}

void Display::deleteStatusInternal(const QString& name) { status_map_.remove(name); }

void Display::clearStatuses() { QMetaObject::invokeMethod(this, "clearStatusesInternal", Qt::QueuedConnection); }

void Display::clearStatusesInternal() { status_map_.clear(); }

void Display::setVisibilityBits(uint32_t bits) { visibility_bits_ = bits; }

void Display::unsetVisibilityBits(uint32_t bits) { visibility_bits_ &= ~bits; }

uint32_t Display::getVisibilityBits() { return visibility_bits_; }

SceneViewer* Display::getSceneViewer() const { return scene_viewer_; }

void Display::setAssociatedWidget(QWidget* widget) {
  associated_widget_ = widget;
  if (associated_widget_) {
    associated_widget_->setVisible(isEnabled());
  }
}

QWidget* Display::getAssociatedWidget() const { return associated_widget_; }

PanelDockWidget* Display::getAssociatedWidgetPanel() { return associated_widget_panel_; }

void Display::setName(const QString& name) { name_ = name; }

QString Display::getName() const { return name_; }

void Display::setIcon(const QIcon& icon) { icon_ = icon; }

QIcon Display::getIcon() const { return icon_; }

void Display::setEnabled(bool enabled) {
  if (enabled_ == enabled) {
    return;
  }

  enabled_ = enabled;

  if (enabled_) {
    onEnable();
  } else {
    onDisable();
  }

  if (associated_widget_) {
    associated_widget_->setVisible(enabled_);
  }
}

bool Display::isEnabled() const { return enabled_; }

void Display::setFixedFrame(const QString& fixed_frame) {
  if (fixed_frame_ != fixed_frame) {
    fixed_frame_ = fixed_frame;
    fixedFrameChanged();
  }
}

void Display::reset() {
  // Override in derived classes
}

void Display::onInitialize() {
  // Override in derived classes
}

void Display::onEnable() {
  // Override in derived classes
}

void Display::onDisable() {
  // Override in derived classes
}

// clearStatuses() is already implemented above

void Display::fixedFrameChanged() {
  // Override in derived classes
}

bool Display::initialized() const { return initialized_; }

void Display::load(const Config& config) {
  QString name;
  if (config.mapGetString("Name", &name)) {
    setName(name);
  }

  bool enabled = false;
  if (config.mapGetBool("Enabled", &enabled)) {
    setEnabled(enabled);
  }
}

void Display::save(Config config) const {
  config.mapSetValue("Name", name_);
  config.mapSetValue("Enabled", enabled_);
  config.mapSetValue("Class", class_id_);
}

void Display::associatedPanelVisibilityChange(bool visible) {
  (void)visible;
  // Handle panel visibility changes if needed
}

Ogre::SceneNode* Display::getSceneNode() const { return scene_node_; }

void Display::disable() { setEnabled(false); }

void Display::onEnableChanged() {
  // Handle enable state changes if needed
}

}  // namespace common
}  // namespace aviz
