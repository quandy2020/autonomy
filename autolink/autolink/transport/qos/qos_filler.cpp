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

#include "autolink/transport/qos/qos_filler.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "autolink/common/log.hpp"
#include "autolink/transport/qos/qos_profile_conf.hpp"

#include "fastdds/dds/core/Time_t.hpp"
#include "fastdds/rtps/attributes/ResourceManagement.hpp"

namespace autolink {
namespace transport {

using proto::QosDurabilityPolicy;
using proto::QosHistoryPolicy;
using proto::QosProfile;
using proto::QosReliabilityPolicy;
using transport::QosProfileConf;

namespace {

using eprosima::fastdds::dds::Duration_t;
using eprosima::fastdds::dds::HistoryQosPolicy;

eprosima::fastdds::dds::HistoryQosPolicyKind ToHistoryKind(
    const QosProfile& qos) {
    switch (qos.history()) {
        case QosHistoryPolicy::HISTORY_KEEP_LAST:
            return eprosima::fastdds::dds::KEEP_LAST_HISTORY_QOS;
        case QosHistoryPolicy::HISTORY_KEEP_ALL:
            return eprosima::fastdds::dds::KEEP_ALL_HISTORY_QOS;
        default:
            return HistoryQosPolicy().kind;
    }
}

eprosima::fastdds::dds::DurabilityQosPolicyKind ToDurabilityKind(
    const QosProfile& qos) {
    switch (qos.durability()) {
        case QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL:
            return eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
        case QosDurabilityPolicy::DURABILITY_VOLATILE:
            return eprosima::fastdds::dds::VOLATILE_DURABILITY_QOS;
        default:
            return eprosima::fastdds::dds::TRANSIENT_LOCAL_DURABILITY_QOS;
    }
}

eprosima::fastdds::dds::ReliabilityQosPolicyKind ToReliabilityKind(
    const QosProfile& qos) {
    switch (qos.reliability()) {
        case QosReliabilityPolicy::RELIABILITY_BEST_EFFORT:
            return eprosima::fastdds::dds::BEST_EFFORT_RELIABILITY_QOS;
        case QosReliabilityPolicy::RELIABILITY_RELIABLE:
            return eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
        default:
            return eprosima::fastdds::dds::RELIABLE_RELIABILITY_QOS;
    }
}

bool ApplyHistoryDepth(const QosProfile& qos,
                       eprosima::fastdds::dds::HistoryQosPolicy* history) {
    RETURN_VAL_IF_NULL2(history, false);
    history->kind = ToHistoryKind(qos);
    if (qos.depth() != QosProfileConf::QOS_HISTORY_DEPTH_SYSTEM_DEFAULT) {
        history->depth = static_cast<int32_t>(qos.depth());
    }
    RETURN_VAL_IF2(history->depth < 0, false);
    return true;
}

void ConfigureHeartbeat(
    const QosProfile& qos,
    eprosima::fastdds::dds::RTPSReliableWriterQos* reliable) {
    if (reliable == nullptr) {
        return;
    }

    auto& heartbeat_period = reliable->times.heartbeat_period;

    if (qos.mps() == 0) {
        heartbeat_period = Duration_t(0, 300'000'000u);  // 300ms default
        return;
    }

    uint64_t clamped_mps = std::clamp<uint64_t>(qos.mps(), 64u, 1024u);
    long double period_seconds = 256.0L / static_cast<long double>(clamped_mps);
    long double integral_part = std::floor(period_seconds);
    int32_t seconds = static_cast<int32_t>(integral_part);
    long double fractional_part = period_seconds - integral_part;
    uint64_t nanos_ll =
        static_cast<uint64_t>(std::llround(fractional_part * 1'000'000'000.0L));
    if (nanos_ll >= 1'000'000'000ULL) {
        ++seconds;
        nanos_ll -= 1'000'000'000ULL;
    }
    heartbeat_period = Duration_t(seconds, static_cast<uint32_t>(nanos_ll));
}

std::string QoSLogMessage(
    const std::string& channel_name,
    const eprosima::fastdds::dds::HistoryQosPolicy& history,
    eprosima::fastdds::dds::DurabilityQosPolicyKind durability,
    eprosima::fastdds::dds::ReliabilityQosPolicyKind reliability) {
    std::ostringstream oss;
    oss << channel_name
        << " qos: [history_kind=" << static_cast<int>(history.kind)
        << "] [durability_kind=" << static_cast<int>(durability)
        << "] [reliability_kind=" << static_cast<int>(reliability)
        << "] [depth=" << history.depth << "]";
    return oss.str();
}

}  // namespace

bool QosFiller::FillInPubQos(const std::string& channel_name,
                             const QosProfile& qos,
                             eprosima::fastdds::dds::PublisherQos* pub_qos) {
    RETURN_VAL_IF_NULL2(pub_qos, false);
    *pub_qos = eprosima::fastdds::dds::PUBLISHER_QOS_DEFAULT;
    if (qos.depth() < 0 &&
        qos.depth() != static_cast<int32_t>(
                           QosProfileConf::QOS_HISTORY_DEPTH_SYSTEM_DEFAULT)) {
        AERROR << "Invalid history depth for channel " << channel_name << ": "
               << qos.depth();
        return false;
    }
    return true;
}

bool QosFiller::FillInWriterQos(
    const std::string& channel_name, const QosProfile& qos,
    eprosima::fastdds::dds::DataWriterQos* writer_qos) {
    RETURN_VAL_IF_NULL2(writer_qos, false);

    auto durability = ToDurabilityKind(qos);
    auto reliability = ToReliabilityKind(qos);

    RETURN_VAL_IF2(!ApplyHistoryDepth(qos, &writer_qos->history()), false);
    writer_qos->durability().kind = durability;
    writer_qos->reliability().kind = reliability;
    writer_qos->publish_mode().kind =
        eprosima::fastdds::dds::ASYNCHRONOUS_PUBLISH_MODE;
    writer_qos->endpoint().history_memory_policy =
        eprosima::fastdds::rtps::DYNAMIC_RESERVE_MEMORY_MODE;
    ConfigureHeartbeat(qos, &writer_qos->reliable_writer_qos());

    AINFO << QoSLogMessage(channel_name, writer_qos->history(), durability,
                           reliability);
    return true;
}

bool QosFiller::FillInSubQos(const std::string& channel_name,
                             const QosProfile& qos,
                             eprosima::fastdds::dds::SubscriberQos* sub_qos) {
    RETURN_VAL_IF_NULL2(sub_qos, false);
    *sub_qos = eprosima::fastdds::dds::SUBSCRIBER_QOS_DEFAULT;
    if (qos.depth() < 0 &&
        qos.depth() != static_cast<int32_t>(
                           QosProfileConf::QOS_HISTORY_DEPTH_SYSTEM_DEFAULT)) {
        AERROR << "Invalid history depth for channel " << channel_name << ": "
               << qos.depth();
        return false;
    }
    return true;
}

bool QosFiller::FillInReaderQos(
    const std::string& channel_name, const QosProfile& qos,
    eprosima::fastdds::dds::DataReaderQos* reader_qos) {
    RETURN_VAL_IF_NULL2(reader_qos, false);

    auto durability = ToDurabilityKind(qos);
    auto reliability = ToReliabilityKind(qos);

    RETURN_VAL_IF2(!ApplyHistoryDepth(qos, &reader_qos->history()), false);
    reader_qos->durability().kind = durability;
    reader_qos->reliability().kind = reliability;
    reader_qos->endpoint().history_memory_policy =
        eprosima::fastdds::rtps::DYNAMIC_RESERVE_MEMORY_MODE;
    reader_qos->expects_inline_qos(false);
    reader_qos->reliable_reader_qos().disable_positive_acks.enabled = false;

    AINFO << QoSLogMessage(channel_name, reader_qos->history(), durability,
                           reliability);
    return true;
}

bool QosFiller::FillInTopicQos(const std::string& /*channel_name*/,
                               const QosProfile& qos,
                               eprosima::fastdds::dds::TopicQos* topic_qos) {
    RETURN_VAL_IF_NULL2(topic_qos, false);
    RETURN_VAL_IF2(!ApplyHistoryDepth(qos, &topic_qos->history()), false);
    return true;
}

}  // namespace transport
}  // namespace autolink