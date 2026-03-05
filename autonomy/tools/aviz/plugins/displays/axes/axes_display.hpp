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

#pragma once

#include <memory>

#include "autonomy/tools/aviz/common/display.hpp"
#include "autonomy/tools/aviz/common/properties/float_property.hpp"

namespace aviz {
namespace rendering {
class Axes;
}  // namespace rendering
}  // namespace aviz

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief AxesDisplay
 * Axes display
 *
 * Displays a set of X/Y/Z axes at a reference frame
 */
class AxesDisplay : public aviz::common::Display {
  Q_OBJECT

 public:
  AxesDisplay();
  ~AxesDisplay() override;

  void onInitialize() override;

  // Overrides from Display
  void update(float wall_dt, float ros_dt) override;

 protected:
  void onEnable() override;

  void onDisable() override;

 private Q_SLOTS:
  void updateShape();

 private:
  std::shared_ptr<aviz::rendering::Axes> axes_;

  aviz::common::properties::FloatProperty* length_property_;
  aviz::common::properties::FloatProperty* radius_property_;
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
