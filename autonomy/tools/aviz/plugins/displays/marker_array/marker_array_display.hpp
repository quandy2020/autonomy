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

#include "autonomy/commsgs/visualization_msgs.hpp"

#include "autonomy/tools/aviz/common/autolink_topic_display.hpp"

namespace aviz {
namespace plugins {
namespace displays {

/**
 * @brief MarkerArrayDisplay
 * Similar to rviz_default_plugins::displays::MarkerArrayDisplay
 *
 * Displays visualization_msgs::MarkerArray messages
 * This is essentially a wrapper around MarkerDisplay that processes arrays
 */
class MarkerArrayDisplay : public AutolinkTopicDisplay<autonomy::commsgs::visualization_msgs::MarkerArray>
{
    Q_OBJECT

public:
    explicit MarkerArrayDisplay(const QString& name = "MarkerArrayDisplay");
    ~MarkerArrayDisplay() override;

    void onInitialize() override;
    void reset() override;

protected:
    void processMessage(const std::shared_ptr<autonomy::commsgs::visualization_msgs::MarkerArray>& msg) override;

private:
    // Use internal marker display to handle individual markers
    class MarkerDisplay* marker_display_;
};

}  // namespace displays
}  // namespace plugins
}  // namespace aviz
