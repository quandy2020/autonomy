/*
 * Copyright 2026 The Openbot Authors
 *
 * Goal channel + feedback publisher for one task domain (teleop-style ingress).
 */

#ifndef AUTONOMY_TASK_INTERFACE_GOAL_INGRESS_HPP_
#define AUTONOMY_TASK_INTERFACE_GOAL_INGRESS_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "autolink/node/node.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace task {
namespace interface {

/**
 * Subscribes to Goal messages, calls Submit, and optionally publishes Feedback
 * on a background poll loop (same pattern as teleop ingress).
 */
template <typename GoalType, typename FeedbackType>
class GoalIngress
{
public:
    using SubmitFunction = std::function<bool(const GoalType&)>;
    using FeedbackFunction = std::function<bool(FeedbackType*)>;
    using IsActiveFunction = std::function<bool()>;
    using IsTerminalFunction = std::function<bool(const FeedbackType&)>;
    using MakeRejectedFunction = std::function<FeedbackType()>;

    GoalIngress() = default;
    ~GoalIngress() { Stop(); }

    GoalIngress(const GoalIngress&) = delete;
    GoalIngress& operator=(const GoalIngress&) = delete;

    bool Start(const std::shared_ptr<autolink::Node>& node,
               const std::string& goal_channel,
               const std::string& feedback_channel,
               std::chrono::milliseconds period, SubmitFunction submit,
               FeedbackFunction feedback, IsActiveFunction is_active,
               IsTerminalFunction is_terminal,
               MakeRejectedFunction make_rejected) {
        Stop();
        if (!node || !submit || goal_channel.empty()) {
            return false;
        }
        node_ = node;
        submit_ = std::move(submit);
        feedback_ = std::move(feedback);
        is_active_ = std::move(is_active);
        is_terminal_ = std::move(is_terminal);
        make_rejected_ = std::move(make_rejected);
        period_ = period.count() > 0 ? period : std::chrono::milliseconds(100);

        if (!feedback_channel.empty() && feedback_ && is_active_ &&
            is_terminal_) {
            feedback_writer_ =
                node_->CreateWriter<FeedbackType>(feedback_channel);
            if (feedback_writer_) {
                feedback_running_.store(true, std::memory_order_release);
                feedback_thread_ = std::thread([this] { FeedbackLoop(); });
            }
        }

        goal_reader_ = node_->CreateReader<GoalType>(
            goal_channel,
            [this](const std::shared_ptr<GoalType>& goal) {
                if (!goal || !submit_) {
                    return;
                }
                if (submit_(*goal)) {
                    return;
                }
                if (!feedback_writer_ || !make_rejected_) {
                    return;
                }
                FeedbackType rejected = make_rejected_();
                feedback_writer_->Write(rejected);
                last_status_ = StatusKey(rejected);
            });
        if (!goal_reader_) {
            AERROR << "GoalIngress: failed to subscribe " << goal_channel;
            Stop();
            return false;
        }
        AINFO << "GoalIngress listening on " << goal_channel;
        return true;
    }

    void Stop() {
        goal_reader_.reset();
        feedback_running_.store(false, std::memory_order_release);
        if (feedback_thread_.joinable()) {
            feedback_thread_.join();
        }
        feedback_writer_.reset();
        last_status_ = -1;
        submit_ = nullptr;
        feedback_ = nullptr;
        is_active_ = nullptr;
        is_terminal_ = nullptr;
        make_rejected_ = nullptr;
        node_.reset();
    }

private:
    static int StatusKey(const FeedbackType& feedback) {
        return static_cast<int>(feedback.status());
    }

    void FeedbackLoop() {
        while (feedback_running_.load(std::memory_order_acquire)) {
            if (feedback_ && feedback_writer_) {
                FeedbackType feedback;
                if (feedback_(&feedback)) {
                    const int status = StatusKey(feedback);
                    const bool terminal =
                        is_terminal_ ? is_terminal_(feedback) : false;
                    const bool active = is_active_ ? is_active_() : false;
                    if (status != last_status_ && (active || terminal)) {
                        feedback_writer_->Write(feedback);
                        last_status_ = status;
                    }
                }
            }
            std::this_thread::sleep_for(period_);
        }
    }

    std::shared_ptr<autolink::Node> node_;
    SubmitFunction submit_;
    FeedbackFunction feedback_;
    IsActiveFunction is_active_;
    IsTerminalFunction is_terminal_;
    MakeRejectedFunction make_rejected_;
    std::chrono::milliseconds period_{100};
    std::shared_ptr<autolink::Reader<GoalType>> goal_reader_;
    std::shared_ptr<autolink::Writer<FeedbackType>> feedback_writer_;
    std::thread feedback_thread_;
    std::atomic<bool> feedback_running_{false};
    int last_status_ = -1;
};

}  // namespace interface
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_INTERFACE_GOAL_INGRESS_HPP_
