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

#include "autonomy/tools/aviz/common/view_manager.hpp"

namespace aviz {
namespace common {

void ViewManager::addViewController(ViewController* view_controller) {
    (void)view_controller;
    // TODO: Implement
}

void ViewManager::removeViewController(ViewController* view_controller) {
    (void)view_controller;
    // TODO: Implement
}

ViewController* ViewManager::getCurrentViewController() const {
    return current_view_controller_;
}

void ViewManager::setCurrentViewController(ViewController* view_controller) {
    current_view_controller_ = view_controller;
}

}  // namespace common
}  // namespace aviz
