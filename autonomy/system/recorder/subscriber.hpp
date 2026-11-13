/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * @file subscriber.hpp
 * @brief Bypass RawMessage subscriber: BEST_EFFORT + KEEP_LAST, no publisher back-pressure.
 */

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autolink/node/node.hpp"
#include "autolink/node/reader_base.hpp"
#include "autonomy/system/recorder/proto/recorder_options.pb.h"
#include "autonomy/system/recorder/rolling_buffer.hpp"

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Bypass subscriber. Callbacks copy payload bytes; they do not steal the
 * middleware buffer.
 *
 * GetMsgType / GetProtoDesc run once per channel; schema rides on that channel's
 * first Sample.
 */
class BypassSubscriber {
public:
    /**
     * @brief Sample callback. Passed by value so the sink can move into the window.
     */
    using Callback = std::function<void(Sample)>;

    /**
     * @brief Create RawMessage readers from RecorderOptions.channels.
     * @param [in] node: Autolink node.
     * @param [in] options: Channel table and QoS depths.
     * @param [in] callback: Per-message callback (capture thread).
     * @return True if at least one reader attached.
     */
    bool Attach(const std::shared_ptr<autolink::Node>& node,
                const proto::RecorderOptions& options, Callback callback);

    /**
     * @brief Release every reader.
     */
    void Reset();

private:
    std::vector<std::shared_ptr<autolink::ReaderBase>> readers_;  ///< Keep-alive.
};

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
