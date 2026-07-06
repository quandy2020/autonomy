/*
 * Copyright 2016 The Cartographer Authors
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

#include <chrono>
#include <memory>

#include "autolink/node/node.hpp"
#include "autonomy/localization/cartographer/io/submap_painter.hpp"
#include "autonomy/localization/cartographer/mapping/id.hpp"
#include "autonomy/localization/cartographer/proto/cartographer_services.pb.h"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

std::unique_ptr<::cartographer::io::SubmapTextures> FetchSubmapTextures(
    const ::cartographer::mapping::SubmapId& submap_id,
    const std::shared_ptr<autolink::Client<proto::SubmapQueryRequest,
                                           proto::SubmapQueryResponse>>&
        client,
    std::chrono::milliseconds timeout);

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
