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

#include "autolink/transport/common/endpoint.hpp"
#include "autolink/common/global_data.hpp"

namespace autolink {
namespace transport {

Endpoint::Endpoint(const RoleAttributes& attr)
    : enabled_(false), id_(), attr_(attr) {
    if (attr_.host_name().empty()) {
        attr_.set_host_name(common::GlobalData::Instance()->HostName());
    }

    if (attr_.process_id() == 0) {
        attr_.set_process_id(common::GlobalData::Instance()->ProcessId());
    }

    if (attr_.id() == 0) {
        attr_.set_id(id_.HashValue());
    }
}

Endpoint::~Endpoint() {}

}  // namespace transport
}  // namespace autolink