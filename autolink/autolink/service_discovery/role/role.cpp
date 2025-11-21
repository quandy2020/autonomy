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

#include "autolink/service_discovery/role/role.hpp"

#include "autolink/common/log.hpp"

namespace autolink {
namespace service_discovery {

using proto::RoleAttributes;

RoleBase::RoleBase() : timestamp_ns_(0) {}

RoleBase::RoleBase(const RoleAttributes& attr, uint64_t timestamp_ns)
    : attributes_(attr), timestamp_ns_(timestamp_ns) {}

bool RoleBase::Match(const RoleAttributes& target_attr) const {
    if (target_attr.node_id() != 0 &&
        target_attr.node_id() != attributes_.node_id()) {
        return false;
    }

    if (target_attr.process_id() != 0 &&
        target_attr.process_id() != attributes_.process_id()) {
        return false;
    }

    if (!target_attr.host_name().empty() &&
        target_attr.host_name() != attributes_.host_name()) {
        return false;
    }

    return true;
}

bool RoleBase::IsEarlierThan(const RoleBase& other) const {
    return timestamp_ns_ < other.timestamp_ns();
}

RoleWriter::RoleWriter(const RoleAttributes& attr, uint64_t timestamp_ns)
    : RoleBase(attr, timestamp_ns) {}

bool RoleWriter::Match(const RoleAttributes& target_attr) const {
    if (target_attr.channel_id() != 0 &&
        target_attr.channel_id() != attributes_.channel_id()) {
        return false;
    }

    if (target_attr.id() != 0 && target_attr.id() != attributes_.id()) {
        return false;
    }

    return RoleBase::Match(target_attr);
}

RoleServer::RoleServer(const RoleAttributes& attr, uint64_t timestamp_ns)
    : RoleBase(attr, timestamp_ns) {}

bool RoleServer::Match(const RoleAttributes& target_attr) const {
    if (target_attr.service_id() != 0 &&
        target_attr.service_id() != attributes_.service_id()) {
        return false;
    }

    return RoleBase::Match(target_attr);
}

}  // namespace service_discovery
}  // namespace autolink
