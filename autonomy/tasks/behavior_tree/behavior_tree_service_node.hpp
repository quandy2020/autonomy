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

#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp/json_export.h"
#include "behaviortree_cpp/action_node.h"

#include "autolink/autolink.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/common/client_wrapper.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/common/server_wrapper.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/tasks/behavior_tree/behavior_tree_utils.hpp"
#include "autonomy/tasks/behavior_tree/json_utils.hpp"

namespace autonomy {
namespace tasks {
namespace behavior_tree {

/**
 * @brief Abstract class representing a service based BT node
 * @tparam ServiceT Type of service
 * @note This is an Asynchronous (long-running) node which may return a RUNNING
 * state while executing. It will re-initialize when halted.
 */
template <class ServiceT>
class BtServiceNode : public BT::ActionNodeBase
{
public:
    /**
     * @brief A nav2_behavior_tree::BtServiceNode constructor
     * @param service_node_name BT node name
     * @param conf BT node configuration
     * @param service_name Optional service name this node creates a client for
     * instead of from input port
     */
    BtServiceNode(const std::string& service_node_name,
                  const BT::NodeConfiguration& conf,
                  const std::string& service_name = "")
        : BT::ActionNodeBase(service_node_name, conf),
          service_name_(service_name),
          service_node_name_(service_node_name) {
        initialize();

        // Make a request for the service without parameter
        request_ = std::make_shared<typename ServiceT::Request>();

        // Make sure the server is actually there before continuing
        ADEBUG << "Waiting for \"" << service_name_ << "\" service";
        if (!service_client_->WaitForService(wait_for_service_timeout_)) {
            AERROR << "Service server " << service_name_
                   << " not available after waiting for "
                   << wait_for_service_timeout_.count() / 1000.0 << " seconds";
            throw std::runtime_error(std::string(
                "Service server " + service_name_ + " not available"));
        }

        AINFO << "BtServiceNode " << service_node_name_ << " initialized";
    }

    BtServiceNode() = delete;

    virtual ~BtServiceNode() = default;

    /**
     * @brief Function to read parameters and initialize class variables
     */
    void initialize() {
        // Get the required items from the blackboard
        auto bt_loop_duration =
            config().blackboard->template get<std::chrono::milliseconds>(
                "bt_loop_duration");
        GetInputOrBlackboard("server_timeout", server_timeout_);
        wait_for_service_timeout_ =
            config().blackboard->template get<std::chrono::milliseconds>(
                "wait_for_service_timeout");

        // timeout should be less than bt_loop_duration to be able to finish the
        // current tick
        max_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            bt_loop_duration * 0.5);

        // Now that we have node_ to use, create the service client for this BT
        // service
        createROSInterfaces();
    }

    /**
     * @brief Function to create ROS interfaces
     */
    void createROSInterfaces() {
        std::string service_new;
        getInput("service_name", service_new);
        if (service_new != service_name_ || !service_client_) {
            service_name_ = service_new;
            node_ =
                config()
                    .blackboard
                    ->template get<std::shared_ptr<::autolink::Node>>("node");
            service_client_ =
                node_->CreateClient<typename ServiceT::Request,
                                    typename ServiceT::Response>(service_name_);
            if (!service_client_) {
                AERROR << "Failed to create service client for "
                       << service_name_;
                throw std::runtime_error("Failed to create service client");
            }
        }
    }

    /**
     * @brief Any subclass of BtServiceNode that accepts parameters must provide
     * a providedPorts method and call providedBasicPorts in it.
     * @param addition Additional ports to add to BT port list
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedBasicPorts(BT::PortsList addition) {
        BT::PortsList basic = {
            BT::InputPort<std::string>("service_name",
                                       "please_set_service_name_in_BT_Node"),
            BT::InputPort<std::chrono::milliseconds>("server_timeout")};
        basic.insert(addition.begin(), addition.end());

        return basic;
    }

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific
     * ports
     */
    static BT::PortsList providedPorts() {
        return providedBasicPorts({});
    }

    /**
     * @brief The main override required by a BT service
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override {
        if (!BT::isStatusActive(status())) {
            initialize();
        }

        if (!request_sent_) {
            // reset the flag to send the request or not,
            // allowing the user the option to set it in on_tick
            should_send_request_ = true;

            // Clear the input request to make sure we have no leftover from
            // previous calls
            request_ = std::make_shared<typename ServiceT::Request>();

            // user defined callback, may modify "should_send_request_".
            on_tick();

            if (!should_send_request_) {
                return BT::NodeStatus::FAILURE;
            }

            future_result_ = service_client_->AsyncSendRequest(request_);
            sent_time_ = commsgs::builtin_interfaces::Time::Now();
            request_sent_ = true;
        }
        return check_future();
    }

    /**
     * @brief The other (optional) override required by a BT service.
     */
    void halt() override {
        request_sent_ = false;
        resetStatus();
    }

    /**
     * @brief Function to perform some user-defined operation on tick
     * Fill in service request with information if necessary
     */
    virtual void on_tick() {}

    /**
     * @brief Function to perform some user-defined operation upon successful
     * completion of the service. Could put a value on the blackboard.
     * @param response can be used to get the result of the service call in the
     * BT Node.
     * @return BT::NodeStatus Returns SUCCESS by default, user may override to
     * return another value
     */
    virtual BT::NodeStatus on_completion(
        std::shared_ptr<typename ServiceT::Response> /*response*/) {
        return BT::NodeStatus::SUCCESS;
    }

    /**
     * @brief Check the future and decide the status of BT
     * @return BT::NodeStatus SUCCESS if future complete before timeout, FAILURE
     * otherwise
     */
    virtual BT::NodeStatus check_future() {
        // Calculate elapsed time in milliseconds
        auto now = commsgs::builtin_interfaces::Time::Now();
        int64_t sent_ns = static_cast<int64_t>(sent_time_.sec) * 1000000000LL +
                          sent_time_.nanosec;
        int64_t now_ns =
            static_cast<int64_t>(now.sec) * 1000000000LL + now.nanosec;
        int64_t elapsed_ns = now_ns - sent_ns;
        auto elapsed = std::chrono::milliseconds(elapsed_ns / 1000000);
        auto remaining = server_timeout_ - elapsed;

        if (remaining > std::chrono::milliseconds(0)) {
            auto timeout = remaining > max_timeout_ ? max_timeout_ : remaining;

            auto status = future_result_.wait_for(timeout);
            if (status == std::future_status::ready) {
                request_sent_ = false;
                auto response = future_result_.get();
                BT::NodeStatus bt_status = on_completion(response);
                return bt_status;
            }

            if (status == std::future_status::timeout) {
                on_wait_for_result();
                // Recalculate elapsed time
                now = commsgs::builtin_interfaces::Time::Now();
                now_ns =
                    static_cast<int64_t>(now.sec) * 1000000000LL + now.nanosec;
                elapsed_ns = now_ns - sent_ns;
                elapsed = std::chrono::milliseconds(elapsed_ns / 1000000);
                if (elapsed < server_timeout_) {
                    return BT::NodeStatus::RUNNING;
                }
            }
        }

        AWARN << "Node timed out while executing service call to "
              << service_name_ << ".";
        request_sent_ = false;
        return BT::NodeStatus::FAILURE;
    }

    /**
     * @brief Function to perform some user-defined operation after a timeout
     * waiting for a result that hasn't been received yet
     */
    virtual void on_wait_for_result() {}

protected:
    /**
     * @brief Function to increment recovery count on blackboard if this node
     * wraps a recovery
     */
    void increment_recovery_count() {
        int recovery_count = 0;
        [[maybe_unused]] auto res = config().blackboard->get(
            "number_recoveries", recovery_count);  // NOLINT
        recovery_count += 1;
        config().blackboard->set("number_recoveries",
                                 recovery_count);  // NOLINT
    }

    std::string service_name_, service_node_name_;
    std::shared_ptr<::autolink::Client<typename ServiceT::Request,
                                       typename ServiceT::Response>>
        service_client_;
    std::shared_ptr<typename ServiceT::Request> request_;

    // The node that will be used for any ROS operations
    std::shared_ptr<::autolink::Node> node_;

    // The timeout value while to use in the tick loop while waiting for a
    // result from the server
    std::chrono::milliseconds server_timeout_;

    // The timeout value for BT loop execution
    std::chrono::milliseconds max_timeout_;

    // The timeout value for waiting for a service to response
    std::chrono::milliseconds wait_for_service_timeout_;

    // To track the server response when a new request is sent
    std::shared_future<std::shared_ptr<typename ServiceT::Response>>
        future_result_;
    bool request_sent_{false};
    commsgs::builtin_interfaces::Time sent_time_;

    // Can be set in on_tick or on_wait_for_result to indicate if a request
    // should be sent.
    bool should_send_request_;
};

}  // namespace behavior_tree
}  // namespace tasks
}  // namespace autonomy