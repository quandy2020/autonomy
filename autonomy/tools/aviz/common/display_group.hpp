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
#include <string>
#include <vector>

#include "autonomy/tools/aviz/common/display.hpp"

namespace aviz {
namespace common {

/**
 * @brief Display group - container for multiple displays
 * Similar to rviz_common::DisplayGroup
 *
 * A DisplayGroup is a Display that contains other Displays.
 * It can be used to organize displays hierarchically.
 */
class DisplayGroup : public aviz::common::Display
{
public:
    explicit DisplayGroup(const std::string& name = "DisplayGroup");
    ~DisplayGroup() override;

    /**
     * @brief Add a display to this group
     */
    void addDisplay(std::unique_ptr<aviz::common::Display> display);

    /**
     * @brief Remove a display from this group
     */
    std::unique_ptr<aviz::common::Display> removeDisplay(const std::string& name);

    /**
     * @brief Get a display by name
     */
    aviz::common::Display* getDisplay(const std::string& name) const;

    /**
     * @brief Get all displays in this group
     */
    const std::vector<std::unique_ptr<aviz::common::Display>>& getDisplays() const {
        return displays_;
    }

    /**
     * @brief Get number of displays in this group
     */
    size_t getDisplayCount() const {
        return displays_.size();
    }

    /**
     * @brief Clear all displays
     */
    void clear();

    // Overrides from Display
    void onEnable() override;
    void onDisable() override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;

private:
    std::vector<std::unique_ptr<aviz::common::Display>> displays_;
};

}  // namespace common
}  // namespace aviz
