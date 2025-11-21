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

#include "autolink/autolink.hpp"

#include <memory>
#include <string>
#include <utility>

#include "autolink/proto/run_mode_conf.pb.h"

#include "autolink/common/global_data.hpp"

namespace autolink {

using autolink::common::GlobalData;
using autolink::proto::RunMode;

std::unique_ptr<Node> CreateNode(const std::string& node_name,
                                 const std::string& name_space) {
    bool is_reality_mode = GlobalData::Instance()->IsRealityMode();
    if (is_reality_mode && !OK()) {
        AERROR << "please initialize autolink firstly.";
        return nullptr;
    }

    return std::unique_ptr<Node>(new Node(node_name, name_space));
}

}  // namespace autolink