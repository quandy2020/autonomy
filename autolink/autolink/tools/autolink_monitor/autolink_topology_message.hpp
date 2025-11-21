/**
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <map>
#include <string>

#include "autolink/tools/autolink_monitor/renderable_message.hpp"

namespace autolink {
namespace proto {
class ChangeMsg;
class RoleAttributes;
}  // namespace proto
}  // namespace autolink

class GeneralChannelMessage;
// class GeneralMessage;

class AutolinkTopologyMessage : public RenderableMessage
{
public:
    explicit AutolinkTopologyMessage(const std::string& channel);
    ~AutolinkTopologyMessage();

    int Render(const Screen* s, int key) override;
    RenderableMessage* Child(int index) const override;

    void TopologyChanged(const autolink::proto::ChangeMsg& change_msg);
    void AddReaderWriter(const autolink::proto::RoleAttributes& role,
                         bool isWriter);

private:
    AutolinkTopologyMessage(const AutolinkTopologyMessage&) = delete;
    AutolinkTopologyMessage& operator=(const AutolinkTopologyMessage&) = delete;

    void ChangeState(const Screen* s, int key);
    bool IsFromHere(const std::string& node_name);

    std::map<std::string, GeneralChannelMessage*>::const_iterator FindChild(
        int index) const;

    enum class SecondColumnType { MessageType, MessageFrameRatio };
    SecondColumnType second_column_;

    int pid_;
    int col1_width_;
    const std::string& specified_channel_;
    std::map<std::string, GeneralChannelMessage*> all_channels_map_;
};
