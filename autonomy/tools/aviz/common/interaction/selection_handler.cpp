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

#include "autonomy/tools/aviz/common/interaction/selection_handler.hpp"

#include "autonomy/tools/aviz/common/display_context.hpp"
#include "autonomy/tools/aviz/common/interaction/handler_manager_iface.hpp"

// Forward declaration - RenderableObject is defined elsewhere
class RenderableObject;

namespace aviz {
namespace common {
namespace interaction {

SelectionHandler::SelectionHandler(DisplayContext* context) : context_(context), pick_handle_(0) {}

SelectionHandler::~SelectionHandler() {
    if (pick_handle_ != 0 && context_) {
        auto handler_manager = context_->getHandlerManager();
        if (handler_manager) {
            handler_manager->removeHandler(pick_handle_);
        }
    }
    // Delete all properties
    for (auto* prop : properties_) {
        delete prop;
    }
    properties_.clear();
}

void SelectionHandler::registerHandle() {
    if (context_) {
        auto handler_manager = context_->getHandlerManager();
        if (handler_manager) {
            pick_handle_ = handler_manager->createHandle();
            handler_manager->addHandler(pick_handle_, shared_from_this());
        }
    }
}

CollObjectHandle SelectionHandler::getHandle() const {
    return pick_handle_;
}

void SelectionHandler::addTrackedObject(RenderableObject* object) {
    if (object) {
        tracked_objects_.insert(object);
    }
}

void SelectionHandler::removeTrackedObject(RenderableObject* object) {
    tracked_objects_.erase(object);
}

void SelectionHandler::updateTrackedBoxes() {
    // Override in derived classes if needed
}

void SelectionHandler::createProperties(const Picked& obj, aviz::common::properties::Property* parent_property) {
    (void)obj;
    (void)parent_property;
    // Override in derived classes
}

void SelectionHandler::destroyProperties(const Picked& obj, aviz::common::properties::Property* parent_property) {
    (void)obj;
    (void)parent_property;
    // Delete all properties
    for (auto* prop : properties_) {
        delete prop;
    }
    properties_.clear();
}

void SelectionHandler::updateProperties() {
    // Override in derived classes
}

bool SelectionHandler::needsAdditionalRenderPass(uint32_t pass) {
    (void)pass;
    return false;
}

void SelectionHandler::preRenderPass(uint32_t pass) {
    (void)pass;
}

void SelectionHandler::postRenderPass(uint32_t pass) {
    (void)pass;
}

void SelectionHandler::onSelect(const Picked& obj) {
    (void)obj;
    // Override in derived classes
}

void SelectionHandler::onDeselect(const Picked& obj) {
    (void)obj;
    // Override in derived classes
}

}  // namespace interaction
}  // namespace common
}  // namespace aviz
