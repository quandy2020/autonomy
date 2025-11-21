/*
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

#include "absl/strings/str_cat.h"
#include "autonomy/common/logging.hpp"
#include "autonomy/visualization/foxglove_bridge.hpp"
#include "autonomy/visualization/displays/image.hpp"
#include "autonomy/visualization/displays/point.hpp"
#include "autonomy/visualization/displays/path.hpp"
#include "autonomy/visualization/displays/map.hpp"
#include "autonomy/visualization/displays/tf.hpp"
#include "autonomy/visualization/displays/polygon.hpp"
#include "autonomy/visualization/displays/laser_scan.hpp"

namespace autonomy {
namespace visualization { 


FoxgloveBridge::FoxgloveBridge(const std::string& host, const uint32_t& port,
    const std::string& mcap_filename, bool write_mcap_data)
{
    server_handler_ = std::move(InitServerOptionsHandler(host, port, mcap_filename, write_mcap_data));
    channel_manager_ = std::make_unique<channel::ChannelManager>();
}

void FoxgloveBridge::WaitForShutdown()
{
    server_handler_->server->stop();
}

}   // namespace visualization
}   // namespace autonomy
