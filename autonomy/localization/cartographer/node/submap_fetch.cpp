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

#include "autonomy/localization/cartographer/node/submap_fetch.hpp"

#include "autonomy/localization/cartographer/node/msg_conversion.hpp"
#include "glog/logging.h"

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

std::unique_ptr<::cartographer::io::SubmapTextures> ParseSubmapTexturesFromResponse(
    const proto::SubmapQueryResponse& response) {
    if (response.status().code() != proto::OK || response.textures().empty()) {
        return nullptr;
    }

    auto textures = std::make_unique<::cartographer::io::SubmapTextures>();
    textures->version = response.submap_version();
    for (const auto& texture : response.textures()) {
        const std::string compressed_cells(texture.cells().begin(),
                                           texture.cells().end());
        textures->textures.emplace_back(::cartographer::io::SubmapTexture{
            ::cartographer::io::UnpackTextureData(compressed_cells,
                                                  texture.width(),
                                                  texture.height()),
            texture.width(), texture.height(), texture.resolution(),
            ToRigid3d(FromProtoPose(texture.slice_pose()))});
    }
    return textures;
}

std::unique_ptr<::cartographer::io::SubmapTextures> FetchSubmapTextures(
    const ::cartographer::mapping::SubmapId& submap_id,
    const std::shared_ptr<autolink::Client<proto::SubmapQueryRequest,
                                           proto::SubmapQueryResponse>>&
        client,
    const std::chrono::milliseconds timeout) {
    auto request = std::make_shared<proto::SubmapQueryRequest>();
    request->set_trajectory_id(submap_id.trajectory_id);
    request->set_submap_index(submap_id.submap_index);

    const auto timeout_sec =
        std::chrono::duration_cast<std::chrono::seconds>(timeout);
    const auto response = client->SendRequest(request, timeout_sec);
    if (!response) {
        return nullptr;
    }
    return ParseSubmapTexturesFromResponse(*response);
}

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
