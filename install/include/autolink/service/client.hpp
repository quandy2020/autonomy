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

#include <future>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>

#include "autolink/common/log.hpp"
#include "autolink/common/types.hpp"
#include "autolink/message/message_traits.hpp"
#include "autolink/node/node_channel_impl.hpp"
#include "autolink/proto/topology_change.pb.h"
#include "autolink/service/client_base.hpp"
#include "autolink/service_discovery/topology_manager.hpp"

namespace autolink {

/**
 * @class Client
 * @brief Client get `Response` from a responding `Service` by sending a Request
 *
 * @tparam Request the `Service` request type
 * @tparam Response the `Service` response type
 *
 * @warning One Client can only request one Service
 */
template <typename Request, typename Response>
class Client : public ClientBase
{
public:
    using SharedRequest = typename std::shared_ptr<Request>;
    using SharedResponse = typename std::shared_ptr<Response>;
    using Promise = std::promise<SharedResponse>;
    using SharedPromise = std::shared_ptr<Promise>;
    using SharedFuture = std::shared_future<SharedResponse>;
    using CallbackType = std::function<void(SharedFuture)>;
    using ChangeConnection =
        typename service_discovery::Manager::ChangeConnection;

    /**
     * @brief Construct a new Client object
     *
     * @param node_name used to fill RoleAttribute
     * @param service_name service name the Client can request
     */
    Client(const std::string& node_name, const std::string& service_name)
        : ClientBase(service_name),
          node_name_(node_name),
          request_channel_(service_name + SRV_CHANNEL_REQ_SUFFIX),
          response_channel_(service_name + SRV_CHANNEL_RES_SUFFIX),
          sequence_number_(0) {}

    /**
     * @brief forbid Constructing a new Client object with empty params
     */
    Client() = delete;

    virtual ~Client() {
        Destroy();
    }

    /**
     * @brief Init the Client
     *
     * @return true if init successfully
     * @return false if init failed
     */
    bool Init();

    /**
     * @brief Request the Service with a shared ptr Request type
     *
     * @param request shared ptr of Request type
     * @param timeout_s request timeout, if timeout, response will be empty
     * @return SharedResponse result of this request
     */
    SharedResponse SendRequest(
        SharedRequest request,
        const std::chrono::seconds& timeout_s = std::chrono::seconds(5));

    /**
     * @brief Request the Service with a Request object
     *
     * @param request Request object
     * @param timeout_s request timeout, if timeout, response will be empty
     * @return SharedResponse result of this request
     */
    SharedResponse SendRequest(
        const Request& request,
        const std::chrono::seconds& timeout_s = std::chrono::seconds(5));

    /**
     * @brief Send Request shared ptr asynchronously
     */
    SharedFuture AsyncSendRequest(SharedRequest request);

    /**
     * @brief Send Request object asynchronously
     */
    SharedFuture AsyncSendRequest(const Request& request);

    /**
     * @brief Send Request shared ptr asynchronously and invoke `cb` after we
     * get response
     *
     * @param request Request shared ptr
     * @param cb callback function after we get response
     * @return SharedFuture a `std::future` shared ptr
     */
    SharedFuture AsyncSendRequest(SharedRequest request, CallbackType&& cb);

    /**
     * @brief Is the Service is ready?
     */
    bool ServiceIsReady() const;

    /**
     * @brief destroy this Client
     */
    void Destroy();

    /**
     * @brief wait for the connection with the Service established
     *
     * @tparam RatioT timeout unit, default is std::milli
     * @param timeout wait time in unit of `RatioT`
     * @return true if the connection established
     * @return false if timeout
     */
    template <typename RatioT = std::milli>
    bool WaitForService(std::chrono::duration<int64_t, RatioT> timeout =
                            std::chrono::duration<int64_t, RatioT>(-1)) {
        return WaitForServiceNanoseconds(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
    }

private:
    void HandleResponse(const std::shared_ptr<Response>& response,
                        const transport::MessageInfo& request_info);

    bool IsInit(void) const {
        return response_receiver_ != nullptr;
    }

    void JoinTheTopology();
    void LeaveTheTopology();
    void OnChannelChange(const proto::ChangeMsg& change_msg);

    std::string node_name_;

    std::function<void(const std::shared_ptr<Response>&,
                       const transport::MessageInfo&)>
        response_callback_;

    std::unordered_map<uint64_t,
                       std::tuple<SharedPromise, CallbackType, SharedFuture>>
        pending_requests_;
    std::mutex pending_requests_mutex_;

    std::shared_ptr<transport::Transmitter<Request>> request_transmitter_;
    std::shared_ptr<transport::Receiver<Response>> response_receiver_;
    std::string request_channel_;
    std::string response_channel_;

    proto::RoleAttributes request_writer_attr_;
    proto::RoleAttributes response_reader_attr_;
    service_discovery::ChannelManagerPtr channel_manager_;
    ChangeConnection change_conn_;

    transport::Identity writer_id_;
    uint64_t sequence_number_;
};

template <typename Request, typename Response>
void Client<Request, Response>::Destroy() {
    if (!IsInit()) {
        return;
    }
    LeaveTheTopology();
    {
        std::lock_guard<std::mutex> lock(pending_requests_mutex_);
        pending_requests_.clear();
    }
    response_receiver_.reset();
    request_transmitter_.reset();
    channel_manager_ = nullptr;
}

template <typename Request, typename Response>
bool Client<Request, Response>::Init() {
    if (IsInit()) {
        return true;
    }
    proto::RoleAttributes role;
    role.set_host_name(common::GlobalData::Instance()->HostName());
    role.set_host_ip(common::GlobalData::Instance()->HostIp());
    role.set_process_id(common::GlobalData::Instance()->ProcessId());
    role.set_node_name(node_name_);
    role.set_channel_name(request_channel_);
    auto channel_id = common::GlobalData::RegisterChannel(request_channel_);
    role.set_channel_id(channel_id);
    role.set_message_type(message::MessageType<Request>());
    role.mutable_qos_profile()->CopyFrom(
        transport::QosProfileConf::QOS_PROFILE_SERVICES_DEFAULT);
    auto transport = transport::Transport::Instance();
    request_transmitter_ = transport->CreateTransmitter<Request>(
        role, proto::OptionalMode::HYBRID);
    if (request_transmitter_ == nullptr) {
        AERROR << "Create request pub failed.";
        return false;
    }
    writer_id_ = request_transmitter_->id();
    request_writer_attr_.CopyFrom(role);
    request_writer_attr_.set_id(writer_id_.HashValue());

    response_callback_ =
        std::bind(&Client<Request, Response>::HandleResponse, this,
                  std::placeholders::_1, std::placeholders::_2);

    role.set_channel_name(response_channel_);
    channel_id = common::GlobalData::RegisterChannel(response_channel_);
    role.set_channel_id(channel_id);
    role.set_message_type(message::MessageType<Response>());
    response_receiver_ = transport->CreateReceiver<Response>(
        role,
        [=](const std::shared_ptr<Response>& response,
            const transport::MessageInfo& message_info,
            const proto::RoleAttributes& reader_attr) {
            (void)reader_attr;
            response_callback_(response, message_info);
        },
        proto::OptionalMode::HYBRID);
    if (response_receiver_ == nullptr) {
        AERROR << "Create response sub failed.";
        request_transmitter_.reset();
        return false;
    }
    response_reader_attr_.CopyFrom(role);
    response_reader_attr_.set_id(response_receiver_->id().HashValue());

    channel_manager_ =
        service_discovery::TopologyManager::Instance()->channel_manager();
    JoinTheTopology();
    return true;
}

template <typename Request, typename Response>
void Client<Request, Response>::JoinTheTopology() {
    change_conn_ = channel_manager_->AddChangeListener(std::bind(
        &Client<Request, Response>::OnChannelChange, this,
        std::placeholders::_1));

    std::vector<proto::RoleAttributes> readers;
    channel_manager_->GetReadersOfChannel(request_channel_, &readers);
    for (auto& reader : readers) {
        request_transmitter_->Enable(reader);
    }
    channel_manager_->Join(request_writer_attr_, proto::RoleType::ROLE_WRITER,
                           message::HasSerializer<Request>::value);

    std::vector<proto::RoleAttributes> writers;
    channel_manager_->GetWritersOfChannel(response_channel_, &writers);
    for (auto& writer : writers) {
        response_receiver_->Enable(writer);
    }
    channel_manager_->Join(response_reader_attr_, proto::RoleType::ROLE_READER,
                           message::HasSerializer<Response>::value);
}

template <typename Request, typename Response>
void Client<Request, Response>::LeaveTheTopology() {
    if (channel_manager_ == nullptr) {
        return;
    }
    channel_manager_->RemoveChangeListener(change_conn_);
    channel_manager_->Leave(request_writer_attr_,
                            proto::RoleType::ROLE_WRITER);
    channel_manager_->Leave(response_reader_attr_,
                            proto::RoleType::ROLE_READER);
}

template <typename Request, typename Response>
void Client<Request, Response>::OnChannelChange(
    const proto::ChangeMsg& change_msg) {
    const auto& peer = change_msg.role_attr();
    const auto operate_type = change_msg.operate_type();

    if (peer.channel_name() == request_channel_ &&
        change_msg.role_type() == proto::RoleType::ROLE_READER) {
        if (operate_type == proto::OperateType::OPT_JOIN) {
            request_transmitter_->Enable(peer);
        } else {
            request_transmitter_->Disable(peer);
        }
        return;
    }

    if (peer.channel_name() == response_channel_ &&
        change_msg.role_type() == proto::RoleType::ROLE_WRITER) {
        if (operate_type == proto::OperateType::OPT_JOIN) {
            response_receiver_->Enable(peer);
        } else {
            response_receiver_->Disable(peer);
        }
    }
}

template <typename Request, typename Response>
typename Client<Request, Response>::SharedResponse
Client<Request, Response>::SendRequest(SharedRequest request,
                                       const std::chrono::seconds& timeout_s) {
    if (!IsInit()) {
        return nullptr;
    }
    auto future = AsyncSendRequest(request);
    if (!future.valid()) {
        return nullptr;
    }
    auto status = future.wait_for(timeout_s);
    if (status == std::future_status::ready) {
        return future.get();
    } else {
        return nullptr;
    }
}

template <typename Request, typename Response>
typename Client<Request, Response>::SharedResponse
Client<Request, Response>::SendRequest(const Request& request,
                                       const std::chrono::seconds& timeout_s) {
    if (!IsInit()) {
        return nullptr;
    }
    auto request_ptr = std::make_shared<const Request>(request);
    return SendRequest(request_ptr, timeout_s);
}

template <typename Request, typename Response>
typename Client<Request, Response>::SharedFuture
Client<Request, Response>::AsyncSendRequest(const Request& request) {
    auto request_ptr = std::make_shared<const Request>(request);
    return AsyncSendRequest(request_ptr);
}

template <typename Request, typename Response>
typename Client<Request, Response>::SharedFuture
Client<Request, Response>::AsyncSendRequest(SharedRequest request) {
    return AsyncSendRequest(request, [](SharedFuture) {});
}

template <typename Request, typename Response>
typename Client<Request, Response>::SharedFuture
Client<Request, Response>::AsyncSendRequest(SharedRequest request,
                                            CallbackType&& cb) {
    if (IsInit()) {
        std::lock_guard<std::mutex> lock(pending_requests_mutex_);
        sequence_number_++;
        transport::MessageInfo info(writer_id_, sequence_number_, writer_id_);
        if (!request_transmitter_->Transmit(request, info)) {
            AWARN << "Send request failed (no peer or transport error) channel="
                  << request_channel_;
            return std::shared_future<std::shared_ptr<Response>>();
        }
        SharedPromise call_promise = std::make_shared<Promise>();
        SharedFuture f(call_promise->get_future());
        pending_requests_[info.seq_num()] =
            std::make_tuple(call_promise, std::forward<CallbackType>(cb), f);
        return f;
    } else {
        return std::shared_future<std::shared_ptr<Response>>();
    }
}

template <typename Request, typename Response>
bool Client<Request, Response>::ServiceIsReady() const {
    auto* topology = service_discovery::TopologyManager::Instance();
    const bool has_request_reader =
        topology->channel_manager()->HasReader(request_channel_);
    const bool has_response_writer =
        topology->channel_manager()->HasWriter(response_channel_);
    // Channel endpoints are enough to RPC. ROLE_SERVER joins can lag behind
    // __SRV__ channel discovery (see tools/cli/discovery_wait.hpp).
    if (has_request_reader && has_response_writer) {
        return true;
    }
    return topology->service_manager()->HasService(service_name_) &&
           has_request_reader && has_response_writer;
}

template <typename Request, typename Response>
void Client<Request, Response>::HandleResponse(
    const std::shared_ptr<Response>& response,
    const transport::MessageInfo& request_header) {
    ADEBUG << "client recv response.";
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    if (request_header.spare_id() != writer_id_) {
        return;
    }
    uint64_t sequence_number = request_header.seq_num();
    if (this->pending_requests_.count(sequence_number) == 0) {
        return;
    }
    auto tuple = this->pending_requests_[sequence_number];
    auto call_promise = std::get<0>(tuple);
    auto callback = std::get<1>(tuple);
    auto future = std::get<2>(tuple);
    this->pending_requests_.erase(sequence_number);
    call_promise->set_value(response);
    callback(future);
}

}  // namespace autolink
