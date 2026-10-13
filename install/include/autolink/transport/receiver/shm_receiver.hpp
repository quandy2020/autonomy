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

#include <functional>
#include <unordered_map>

#include "autolink/common/log.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autolink/transport/dispatcher/shm_dispatcher.hpp"
#include "autolink/transport/receiver/receiver.hpp"
#include "autolink/transport/shm/protobuf_arena_manager.hpp"

namespace autolink {
namespace transport {

template <typename M>
class ShmReceiver : public Receiver<M>
{
public:
    ShmReceiver(const RoleAttributes& attr,
                const typename Receiver<M>::MessageListener& msg_listener);
    virtual ~ShmReceiver();

    void Enable() override;
    void Disable() override;

    void Enable(const RoleAttributes& opposite_attr) override;
    void Disable(const RoleAttributes& opposite_attr) override;

private:
    void DisableAllWriters();

    ShmDispatcherPtr dispatcher_;
    std::unordered_map<uint64_t, autolink::proto::RoleAttributes> enabled_writers_;
};

template <typename M>
ShmReceiver<M>::ShmReceiver(
    const RoleAttributes& attr,
    const typename Receiver<M>::MessageListener& msg_listener)
    : Receiver<M>(attr, msg_listener) {
    dispatcher_ = ShmDispatcher::Instance();
}

template <typename M>
ShmReceiver<M>::~ShmReceiver() {
    DisableAllWriters();
    Disable();
}

template <typename M>
void ShmReceiver<M>::Enable() {
    if (this->enabled_) {
        return;
    }

    if (autolink::common::GlobalData::Instance()->IsChannelEnableArenaShm(
            this->attr_.channel_id()) &&
        message::MessageType<M>() !=
            message::MessageType<message::RawMessage>()) {
        auto arena_manager = ProtobufArenaManager::Instance();
        if (!arena_manager->Enable() ||
            !arena_manager->EnableSegment(this->attr_.channel_id())) {
            AERROR << "arena manager enable failed.";
            return;
        }
    }

    dispatcher_->AddListener<M>(
        this->attr_, std::bind(&ShmReceiver<M>::OnNewMessage, this,
                               std::placeholders::_1, std::placeholders::_2));
    this->enabled_ = true;
}

template <typename M>
void ShmReceiver<M>::DisableAllWriters() {
    for (const auto& entry : enabled_writers_) {
        dispatcher_->RemoveListener<M>(this->attr_, entry.second);
    }
    enabled_writers_.clear();
}

template <typename M>
void ShmReceiver<M>::Disable() {
    DisableAllWriters();
    if (!this->enabled_) {
        return;
    }

    dispatcher_->RemoveListener<M>(this->attr_);
    this->enabled_ = false;
}

template <typename M>
void ShmReceiver<M>::Enable(const RoleAttributes& opposite_attr) {
    const uint64_t writer_id = opposite_attr.id();
    if (enabled_writers_.count(writer_id) > 0) {
        return;
    }

    if (autolink::common::GlobalData::Instance()->IsChannelEnableArenaShm(
            this->attr_.channel_id()) &&
        message::MessageType<M>() !=
            message::MessageType<message::RawMessage>()) {
        auto arena_manager = ProtobufArenaManager::Instance();
        if (!arena_manager->Enable() ||
            !arena_manager->EnableSegment(this->attr_.channel_id())) {
            AERROR << "arena manager enable failed.";
            return;
        }
    }
    AINFO << "SHM receiver enabled for channel [" << this->attr_.channel_name()
          << "] from writer (discovery ok)";
    dispatcher_->AddListener<M>(
        this->attr_, opposite_attr,
        std::bind(&ShmReceiver<M>::OnNewMessage, this, std::placeholders::_1,
                  std::placeholders::_2));
    enabled_writers_.emplace(writer_id, opposite_attr);
}

template <typename M>
void ShmReceiver<M>::Disable(const RoleAttributes& opposite_attr) {
    const uint64_t writer_id = opposite_attr.id();
    const auto iterator = enabled_writers_.find(writer_id);
    if (iterator == enabled_writers_.end()) {
        return;
    }
    dispatcher_->RemoveListener<M>(this->attr_, iterator->second);
    enabled_writers_.erase(iterator);
}

}  // namespace transport
}  // namespace autolink
