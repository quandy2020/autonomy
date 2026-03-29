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

#include "autolink/autolink.hpp"
#include "autolink/node/node.hpp"
#include "autolink/node/reader.hpp"
#include "autonomy/tools/aviz/common/display.hpp"
#include "autonomy/tools/aviz/common/logging.hpp"

/**
 * @brief Base class for displays that subscribe to Autolink topics
 * Topic-based display for Autolink channels
 *
 * This class handles subscribing and unsubscribing to Autolink channels
 * when the display is enabled or disabled.
 */
template <class MessageType>
class AutolinkTopicDisplay : public aviz::common::Display
{
public:
    explicit AutolinkTopicDisplay(const QString& class_id)
        : aviz::common::Display(),
          topic_(""),
          node_(nullptr),
          reader_(nullptr) {
        setClassId(class_id);
    }

    ~AutolinkTopicDisplay() override {
        unsubscribe();
    }

    /**
     * @brief Set the topic/channel name to subscribe to
     */
    void setTopic(const std::string& topic) {
        if (topic_ == topic) {
            return;
        }

        unsubscribe();
        topic_ = topic;
        if (isEnabled() && !topic_.empty()) {
            subscribe();
        }
    }

    /**
     * @brief Get the current topic/channel name
     */
    const std::string& getTopic() const {
        return topic_;
    }

protected:
    /**
     * @brief Called when display is enabled
     */
    void onEnable() override {
        if (!topic_.empty()) {
            subscribe();
        }
    }

    /**
     * @brief Called when display is disabled
     */
    void onDisable() override {
        unsubscribe();
    }

    /**
     * @brief Subscribe to the topic
     */
    virtual void subscribe() {
        if (topic_.empty() || !context_) {
            return;
        }

        if (!node_) {
            // Use a default name if getName() is not available
            QString display_name = "aviz_display";
            node_ = autolink::CreateNode("aviz_display_" +
                                         display_name.toStdString());
            if (!node_) {
                AVIZ_ERROR("Failed to create autolink node for display: " +
                           display_name.toStdString());
                return;
            }
        }

        // Use std::bind to avoid lambda type issues
        using CallbackType =
            std::function<void(const std::shared_ptr<MessageType>&)>;
        CallbackType callback =
            std::bind(&AutolinkTopicDisplay<MessageType>::processMessage, this,
                      std::placeholders::_1);

        reader_ = node_->CreateReader<MessageType>(topic_, callback);
        if (!reader_) {
            AVIZ_ERROR("Failed to create reader for topic: " + topic_);
        }
    }

    /**
     * @brief Unsubscribe from the topic
     */
    virtual void unsubscribe() {
        reader_.reset();
        // Note: We keep the node_ alive for potential reuse
    }

    /**
     * @brief Process a received message
     * Override this in derived classes to handle messages
     */
    virtual void processMessage(const std::shared_ptr<MessageType>& msg) = 0;

    std::string topic_;
    std::shared_ptr<autolink::Node> node_;
    std::shared_ptr<autolink::Reader<MessageType>> reader_;
};
