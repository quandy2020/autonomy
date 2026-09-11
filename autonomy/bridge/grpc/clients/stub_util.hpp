/*
 * Copyright 2026 The Openbot Authors
 *
 * Shared helpers for bridge gRPC stubs: CommandAck fill and variadic command
 * dispatch.
 */

#pragma once

#include <array>
#include <string>
#include <type_traits>
#include <utility>

#include "autonomy/bridge/proto/external_command_service.pb.h"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Fill `CommandAck` fields on a bridge command response.
 *
 * @tparam Response Response type that exposes `mutable_ack()`.
 * @tparam Request Request type that may carry `header`.
 * @param[in,out] response Response message to populate.
 * @param[in] task_type Task type written into the CommandAck.
 * @param[in] request Source request (for `cmd_id`).
 * @param[in] success Whether the command was accepted / succeeded.
 * @param[in] final Whether this is the terminal stream frame.
 * @param[in] task_status Coarse task status for the muxer / UI.
 * @param[in] message Optional human-readable CommandAck message.
 */
template <typename Response, typename Request>
void FillCommandAck(Response& response, proto::TaskType task_type,
                    const Request& request, bool success, bool final,
                    proto::TaskStatus task_status,
                    const std::string& message = "") {
    auto* command_ack = response.mutable_ack();
    command_ack->set_success(success);
    command_ack->set_final(final);
    command_ack->set_task_type(task_type);
    command_ack->set_task_status(task_status);
    if (request.has_header()) {
        command_ack->set_cmd_id(request.header().cmd_id());
    }
    if (!message.empty()) {
        command_ack->set_message(message);
    }
}

/**
 * @brief Single-command dispatch rule.
 * @tparam Enum Command enum type.
 * @tparam Handler Functor invoked on match (optionally returns `bool`).
 */
template <typename Enum, typename Handler>
struct CommandRule {
    Enum command;
    Handler handler;
};

/**
 * @brief Build a single-command rule.
 * @param[in] command Command enum value to match.
 * @param[in] handler Handler invoked on match.
 */
template <typename Enum, typename Handler>
CommandRule<Enum, std::decay_t<Handler>> MakeCommandRule(Enum command,
                                                         Handler&& handler) {
    return CommandRule<Enum, std::decay_t<Handler>>{
        command, std::forward<Handler>(handler)};
}

/**
 * @brief Multi-command dispatch rule (one handler for several enum values).
 */
template <typename Enum, typename Handler, typename... MoreEnums>
struct CommandMultiRule {
    std::array<Enum, 1 + sizeof...(MoreEnums)> commands;
    Handler handler;

    /**
     * @brief Check whether @p command is listed in this rule.
     * @param[in] command Candidate command.
     * @return true if @p command matches any stored value.
     */
    bool CheckCommandMatch(Enum command) const {
        for (Enum candidate : commands) {
            if (candidate == command) {
                return true;
            }
        }
        return false;
    }
};

/**
 * @brief Build a multi-command rule.
 * @param[in] handler Handler invoked when any listed command matches.
 * @param[in] first First command enum value.
 * @param[in] rest Additional command enum values.
 */
template <typename Handler, typename Enum, typename... MoreEnums>
CommandMultiRule<Enum, std::decay_t<Handler>, MoreEnums...> MakeCommandRules(
    Handler&& handler, Enum first, MoreEnums... rest) {
    return CommandMultiRule<Enum, std::decay_t<Handler>, MoreEnums...>{
        {{first, rest...}}, std::forward<Handler>(handler)};
}

namespace detail {

template <typename Enum, typename Rule>
auto CheckRuleMatch(Enum command, const Rule& rule, int)
    -> decltype(rule.CheckCommandMatch(command), bool()) {
    return rule.CheckCommandMatch(command);
}

template <typename Enum, typename Rule>
bool CheckRuleMatch(Enum command, const Rule& rule, long) {
    return rule.command == command;
}

template <typename Rule>
auto InvokeRule(Rule& rule, int) -> decltype(rule.handler()) {
    return rule.handler();
}

template <typename Rule>
auto InvokeRule(const Rule& rule, int) -> decltype(rule.handler()) {
    return rule.handler();
}

}  // namespace detail

/**
 * @brief Dispatch @p command to the first matching variadic rule.
 *
 * Each rule is a `CommandRule` / `CommandMultiRule` (via `MakeCommandRule` /
 * `MakeCommandRules`). If the matched handler returns `bool`, that value is
 * propagated; otherwise a successful match yields `true`.
 *
 * @param[in] command Command enum to dispatch.
 * @param[in] rules Dispatch rules.
 * @return true if a rule matched and its handler succeeded (or returned void).
 */
template <typename Enum, typename... Rules>
bool DispatchCommands(Enum command, Rules&&... rules) {
    bool matched = false;
    bool result = false;
    auto try_one = [&](auto&& rule) {
        if (matched) {
            return;
        }
        if (!detail::CheckRuleMatch(command, rule, 0)) {
            return;
        }
        matched = true;
        using ReturnType = decltype(detail::InvokeRule(rule, 0));
        if constexpr (std::is_same_v<ReturnType, bool>) {
            result = detail::InvokeRule(rule, 0);
        } else {
            detail::InvokeRule(rule, 0);
            result = true;
        }
    };
    (try_one(rules), ...);
    return matched && result;
}

/**
 * @brief Reject when emergency stop is active; emit @p make_fail and return
 * true.
 * @return true if the caller should return immediately (rejected).
 */
template <typename MuxerPtr, typename MakeFailFn>
bool RejectIfEstopActive(MuxerPtr muxer, MakeFailFn&& make_fail) {
    if (muxer && muxer->CheckEstopActive()) {
        make_fail("emergency stop active");
        return true;
    }
    return false;
}

/**
 * @brief Try muxer acquire for @p type; on failure emit and return true.
 * @return true if the caller should return immediately (rejected).
 */
template <typename MuxerPtr, typename Request, typename MakeFailFn>
bool RejectIfAcquireFailed(MuxerPtr muxer, proto::TaskType type,
                           const Request& request, MakeFailFn&& make_fail) {
    if (!muxer) {
        return false;
    }
    const std::string cmd_id =
        request.has_header() ? request.header().cmd_id() : "";
    const std::string client_id =
        request.has_header() ? request.header().client_id() : "";
    if (!muxer->TryAcquire(type, cmd_id, client_id)) {
        make_fail("another task is active");
        return true;
    }
    return false;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
