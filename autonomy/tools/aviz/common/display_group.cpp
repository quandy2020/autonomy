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

#include "autonomy/tools/aviz/common/display_group.hpp"

namespace aviz {
namespace common {

DisplayGroup::DisplayGroup(const std::string& name) : aviz::common::Display() {
    setName(QString::fromStdString(name));
    setClassId("DisplayGroup");
}

DisplayGroup::~DisplayGroup() {
    clear();
}

void DisplayGroup::addDisplay(std::unique_ptr<aviz::common::Display> display) {
    if (!display) {
        return;
    }

    // Initialize display with our context if we have one
    if (context_) {
        display->initialize(context_);
    }

    displays_.push_back(std::move(display));
}

std::unique_ptr<aviz::common::Display> DisplayGroup::removeDisplay(const std::string& name) {
    for (auto it = displays_.begin(); it != displays_.end(); ++it) {
        if ((*it)->getName().toStdString() == name) {
            auto display = std::move(*it);
            displays_.erase(it);
            return display;
        }
    }
    return nullptr;
}

aviz::common::Display* DisplayGroup::getDisplay(const std::string& name) const {
    for (const auto& display : displays_) {
        if (display->getName().toStdString() == name) {
            return display.get();
        }
    }
    return nullptr;
}

void DisplayGroup::clear() {
    for (auto& display : displays_) {
        if (display->isEnabled()) {
            display->setEnabled(false);
        }
    }
    displays_.clear();
}

void DisplayGroup::onEnable() {
    for (auto& display : displays_) {
        if (!display->isEnabled()) {
            display->setEnabled(true);
        }
    }
}

void DisplayGroup::onDisable() {
    for (auto& display : displays_) {
        if (display->isEnabled()) {
            display->setEnabled(false);
        }
    }
}

void DisplayGroup::update(float wall_dt, float ros_dt) {
    (void)ros_dt;  // Unused for now
    for (auto& display : displays_) {
        if (display->isEnabled()) {
            display->update(wall_dt, ros_dt);
        }
    }
}

void DisplayGroup::reset() {
    for (auto& display : displays_) {
        display->reset();
    }
}

}  // namespace common
}  // namespace aviz
