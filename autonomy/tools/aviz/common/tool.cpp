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

#include "autonomy/tools/aviz/common/tool.hpp"

#include "autonomy/tools/aviz/common/display_context.hpp"
#include "autonomy/tools/aviz/common/properties/property.hpp"

namespace aviz {
namespace common {

Tool::Tool()
    : context_(nullptr),
      shortcut_key_(0),
      access_all_keys_(false),
      property_container_(
          new properties::Property("Tool", QVariant(), "", nullptr)),
      class_id_(""),
      name_(""),
      description_("") {}

Tool::~Tool() {
    if (property_container_) {
        delete property_container_;
    }
}

void Tool::initialize(DisplayContext* context) {
    context_ = context;
    onInitialize();
}

aviz::common::properties::Property* Tool::getPropertyContainer() const {
    return property_container_;
}

char Tool::getShortcutKey() const {
    return shortcut_key_;
}

bool Tool::accessAllKeys() const {
    return access_all_keys_;
}

void Tool::update(float wall_dt, float ros_dt) {
    (void)wall_dt;
    (void)ros_dt;
}

int Tool::processMouseEvent(ViewportMouseEvent& event) {
    (void)event;
    return 0;
}

int Tool::processKeyEvent(QKeyEvent* event, SceneViewer* viewer) {
    (void)event;
    (void)viewer;
    return 0;
}

QString Tool::getName() const {
    return name_;
}

void Tool::setName(const QString& name) {
    if (name_ != name) {
        name_ = name;
        Q_EMIT nameChanged(name);
    }
}

QString Tool::getDescription() const {
    return description_;
}

void Tool::setDescription(const QString& description) {
    description_ = description;
}

QString Tool::getClassId() const {
    return class_id_;
}

void Tool::setClassId(const QString& class_id) {
    class_id_ = class_id;
}

void Tool::load(const Config& config) {
    QString name;
    if (config.mapGetString("Name", &name)) {
        setName(name);
    }
    if (property_container_) {
        property_container_->load(config);
    }
}

void Tool::save(Config config) const {
    config.mapSetValue("Name", name_);
    config.mapSetValue("Class", class_id_);
    if (property_container_) {
        property_container_->save(config);
    }
}

void Tool::setIcon(const QIcon& icon) {
    icon_ = icon;
}

const QIcon& Tool::getIcon() {
    return icon_;
}

void Tool::setCursor(const QCursor& cursor) {
    cursor_ = cursor;
}

const QCursor& Tool::getCursor() {
    return cursor_;
}

void Tool::setStatus(const QString& message) {
    if (context_) {
        context_->setStatus(message);
    }
}

}  // namespace common
}  // namespace aviz
