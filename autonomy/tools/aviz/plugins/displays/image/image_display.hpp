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

#include "autonomy/commsgs/sensor_msgs.hpp"

#include "autonomy/tools/aviz/common/autolink_topic_display.hpp"

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief ImageDisplay
 * Image display
 *
 * Displays sensor_msgs::Image messages
 */
class ImageDisplay : public AutolinkTopicDisplay<autonomy::commsgs::sensor_msgs::Image>
{
    Q_OBJECT

public:
    explicit ImageDisplay(const QString& name = "ImageDisplay");
    ~ImageDisplay() override;

    void onInitialize() override;
    void reset() override;

protected:
    void processMessage(const std::shared_ptr<autonomy::commsgs::sensor_msgs::Image>& msg) override;
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
