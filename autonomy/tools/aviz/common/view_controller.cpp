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

#include "autonomy/tools/aviz/common/view_controller.hpp"

#include <QApplication>
#include <QColor>
#include <QFont>
#include <QKeyEvent>

#include "autonomy/tools/aviz/common/display_context.hpp"
#include "autonomy/tools/aviz/common/viewport_mouse_event.hpp"

namespace aviz {
namespace common {

ViewController::ViewController()
    : Property("View", QVariant(), "", nullptr),
      context_(nullptr),
      is_active_(false),
      class_id_("") {
    // Initialize standard cursors
    standard_cursors_[Default] = QCursor(Qt::ArrowCursor);
    standard_cursors_[Rotate2D] = QCursor(Qt::OpenHandCursor);
    standard_cursors_[Rotate3D] = QCursor(Qt::OpenHandCursor);
    standard_cursors_[MoveXY] = QCursor(Qt::SizeAllCursor);
    standard_cursors_[MoveZ] = QCursor(Qt::SizeVerCursor);
    standard_cursors_[Zoom] = QCursor(Qt::SizeHorCursor);
    standard_cursors_[Crosshair] = QCursor(Qt::CrossCursor);

    cursor_ = standard_cursors_[Default];
}

ViewController::~ViewController() = default;

void ViewController::initialize(DisplayContext* context) {
    context_ = context;
    onInitialize();
}

QString ViewController::formatClassId(const QString& class_id) {
    // Remove "aviz/" prefix if present
    QString formatted = class_id;
    if (formatted.startsWith("aviz/")) {
        formatted = formatted.mid(5);
    }
    return formatted;
}

QVariant ViewController::getViewData(int column, int role) const {
    if (column == 0 && role == Qt::BackgroundRole && is_active_) {
        return QColor(40, 120, 197);  // Blue background for active view
    }
    if (column == 0 && role == Qt::FontRole && is_active_) {
        QFont font;
        font.setBold(true);
        return font;
    }
    return Property::getViewData(column, role);
}

Qt::ItemFlags ViewController::getViewFlags(int column) const {
    Qt::ItemFlags flags = Property::getViewFlags(column);
    if (!is_active_) {
        flags |= Qt::ItemIsDragEnabled;
    }
    return flags;
}

void ViewController::activate() {
    is_active_ = true;
    onActivate();
}

void ViewController::update(float dt, float ros_dt) {
    (void)dt;
    (void)ros_dt;
}

void ViewController::handleMouseEvent(ViewportMouseEvent& evt) {
    (void)evt;
}

void ViewController::handleKeyEvent(QKeyEvent* event, SceneViewer* viewer) {
    (void)event;
    (void)viewer;
    // Default implementation handles "F" (focus) and "Z" (reset) keys
    if (event->key() == Qt::Key_F) {
        // Focus on selection - can be implemented later
    } else if (event->key() == Qt::Key_Z) {
        reset();
    }
}

void ViewController::lookAt(float x, float y, float z) {
    lookAt(QVector3D(x, y, z));
}

void ViewController::mimic(ViewController* source_view) {
    (void)source_view;
    // Base implementation does nothing
}

void ViewController::transitionFrom(ViewController* previous_view) {
    (void)previous_view;
    // Base implementation does nothing
}

void ViewController::emitConfigChanged() {
    Q_EMIT configChanged();
}

QString ViewController::getClassId() const {
    return class_id_;
}

void ViewController::setClassId(const QString& class_id) {
    class_id_ = class_id;
}

void ViewController::load(const Config& config) {
    Property::load(config);
    QString name;
    if (config.mapGetString("Name", &name)) {
        setName(name);
    }
}

void ViewController::save(Config config) const {
    Property::save(config);
    config.mapSetValue("Name", getName());
    config.mapSetValue("Class", class_id_);
}

bool ViewController::isActive() const {
    return is_active_;
}

QCursor ViewController::getCursor() {
    return cursor_;
}

FocalPointStatus ViewController::getFocalPointStatus() {
    return FocalPointStatus(false, QVector3D(0, 0, 0));
}

void ViewController::onInitialize() {
    // Override in derived classes
}

void ViewController::onActivate() {
    // Override in derived classes
}

void ViewController::setCursor(CursorType cursor_type) {
    if (standard_cursors_.contains(cursor_type)) {
        cursor_ = standard_cursors_[cursor_type];
    }
}

void ViewController::setCursor(QCursor cursor) {
    cursor_ = cursor;
}

void ViewController::setStatus(const QString& message) {
    if (context_) {
        context_->setStatus(message);
    }
}

}  // namespace common
}  // namespace aviz
