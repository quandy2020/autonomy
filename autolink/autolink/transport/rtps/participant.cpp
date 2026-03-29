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

#include "autolink/transport/rtps/participant.hpp"

#include <fastrtps/utils/IPLocator.h>

#include <algorithm>

#include "autolink/common/global_data.hpp"
#include "autolink/common/log.hpp"
#include "autolink/proto/transport_conf.pb.h"

namespace autolink {
namespace transport {

Participant::Participant(const std::string& name, int send_port,
                         eprosima::fastrtps::ParticipantListener* listener)
    : shutdown_(false),
      name_(name),
      send_port_(send_port),
      listener_(listener),
      fastrtps_participant_(nullptr) {}

Participant::~Participant() {}

void Participant::Shutdown() {
    if (shutdown_.exchange(true)) {
        return;
    }

    std::lock_guard<std::mutex> lk(mutex_);
    if (fastrtps_participant_ != nullptr) {
        eprosima::fastrtps::Domain::removeParticipant(fastrtps_participant_);
        fastrtps_participant_ = nullptr;
        listener_ = nullptr;
    }
}

eprosima::fastrtps::Participant* Participant::fastrtps_participant() {
    if (shutdown_.load()) {
        return nullptr;
    }

    std::lock_guard<std::mutex> lk(mutex_);
    if (fastrtps_participant_ != nullptr) {
        return fastrtps_participant_;
    }

    CreateFastRtpsParticipant(name_, send_port_, listener_);
    return fastrtps_participant_;
}

void Participant::CreateFastRtpsParticipant(
    const std::string& name, int send_port,
    eprosima::fastrtps::ParticipantListener* listener) {
    uint32_t domain_id = 80;

    const char* val = ::getenv("AUTOLINK_DOMAIN_ID");
    if (val != nullptr) {
        try {
            domain_id = std::stoi(val);
        } catch (const std::exception& e) {
            AERROR << "convert domain_id error " << e.what();
            return;
        }
    }

    auto part_attr_conf = std::make_shared<proto::RtpsParticipantAttr>();
    auto& global_conf = common::GlobalData::Instance()->Config();
    if (global_conf.transport_conf().participant_attr().lease_duration() != 0 ||
        global_conf.transport_conf().participant_attr().announcement_period() !=
            0) {
        part_attr_conf->CopyFrom(
            global_conf.transport_conf().participant_attr());
    }

    eprosima::fastrtps::ParticipantAttributes attr;
    attr.domainId = domain_id;
    attr.rtps.port.domainIDGain =
        static_cast<uint16_t>(part_attr_conf->domain_id_gain());
    attr.rtps.port.portBase =
        static_cast<uint16_t>(part_attr_conf->port_base());
    attr.rtps.builtin.discovery_config.discoveryProtocol =
        eprosima::fastrtps::rtps::DiscoveryProtocol_t::SIMPLE;
    attr.rtps.builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol =
        true;
    attr.rtps.builtin.discovery_config.m_simpleEDP
        .use_PublicationReaderANDSubscriptionWriter = true;
    attr.rtps.builtin.discovery_config.m_simpleEDP
        .use_PublicationWriterANDSubscriptionReader = true;

    /**
     * FastDDS requires: LeaseDuration >= leaseDuration_announcementperiod.
     * The user should set the lease_duration and the announcement_period with
     * values that differ in at least 30%. Values too close to each other may
     * cause the failure of the writer liveliness assertion in networks with
     * high latency or with lots of communication errors.
     */
    int32_t lease_sec = part_attr_conf->lease_duration();
    int32_t announce_sec = part_attr_conf->announcement_period();
    if (lease_sec <= 0 && announce_sec <= 0) {
        lease_sec = 12;
        announce_sec = 3;
    } else if (lease_sec <= 0) {
        lease_sec = announce_sec * 4;
    } else if (announce_sec <= 0) {
        announce_sec = std::max(1, lease_sec / 4);
    }
    if (lease_sec < announce_sec) {
        lease_sec = announce_sec + 1;
    }
    attr.rtps.builtin.discovery_config.leaseDuration =
        eprosima::fastrtps::Duration_t(lease_sec, 0);
    attr.rtps.builtin.discovery_config.leaseDuration_announcementperiod =
        eprosima::fastrtps::Duration_t(announce_sec, 0);

    attr.rtps.setName(name.c_str());

    std::string ip_env("127.0.0.1");
    const char* ip_val = ::getenv("AUTOLINK_IP");
    if (ip_val != nullptr) {
        ip_env = ip_val;
        if (ip_env.empty()) {
            AERROR << "invalid AUTOLINK_IP (an empty string)";
            return;
        }
    }
    ADEBUG << "autolink ip: " << ip_env;

    eprosima::fastrtps::rtps::Locator_t locator;
    locator.kind = LOCATOR_KIND_UDPv4;
    locator.port = 0;
    RETURN_IF(!eprosima::fastrtps::rtps::IPLocator::setIPv4(locator, ip_env));

    attr.rtps.defaultUnicastLocatorList.push_back(locator);
    attr.rtps.builtin.metatrafficUnicastLocatorList.push_back(locator);

    eprosima::fastrtps::rtps::Locator_t mcast_locator;
    mcast_locator.kind = LOCATOR_KIND_UDPv4;
    mcast_locator.port = 0;
    RETURN_IF(!eprosima::fastrtps::rtps::IPLocator::setIPv4(mcast_locator,
                                                            "239.255.0.1"));
    attr.rtps.builtin.metatrafficMulticastLocatorList.push_back(mcast_locator);

    fastrtps_participant_ =
        eprosima::fastrtps::Domain::createParticipant(attr, listener);
    RETURN_IF_NULL(fastrtps_participant_);
    eprosima::fastrtps::Domain::registerType(fastrtps_participant_, &type_);
}

}  // namespace transport
}  // namespace autolink
