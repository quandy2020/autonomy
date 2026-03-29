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

#include "autonomy/tools/aviz/common/panel.hpp"

#include "autonomy/tools/aviz/common/display_context.hpp"

namespace aviz {
namespace common {

Panel::Panel(QWidget* parent)
    : QWidget(parent), context_(nullptr), name_(""), class_id_("") {}

Panel::~Panel() = default;

void Panel::initialize(DisplayContext* context) {
    context_ = context;
    onInitialize();
}

void Panel::load(const Config& config) {
    QString name;
    if (config.mapGetString("Name", &name)) {
        setName(name);
    }
}

void Panel::save(Config config) const {
    config.mapSetValue("Name", name_);
    config.mapSetValue("Class", class_id_);
}

QString Panel::getName() const {
    return name_;
}

void Panel::setName(const QString& name) {
    name_ = name;
}

QString Panel::getClassId() const {
    return class_id_;
}

void Panel::setClassId(const QString& class_id) {
    class_id_ = class_id;
}

}  // namespace common
}  // namespace aviz
