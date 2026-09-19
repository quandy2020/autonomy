/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file command_dispatch.hpp
 * @brief Command dispatch helpers (DispatchCommands / RejectOn*).
 *
 * @details
 * Lightweight left-to-right command enum dispatch used by GoalChannel /
 * multi-command stubs. Also provides RejectOnEstop / RejectOnBusy helpers
 * that emit a failure frame and signal the caller to abort.
 *
 * @par Ownership
 * Header-only; rules own handlers by value. Muxer pointers are non-owning.
 *
 * @par Threading
 * Pure helpers; thread-safety depends on caller / muxer / handler bodies.
 *
 * @par Invariants
 * - DispatchCommands evaluates rules left-to-right; first match wins.
 * - Handlers returning bool propagate that value; void handlers yield true.
 * - RejectOnEstop / RejectOnBusy return true when the caller should abort.
 *
 * @see GoalChannelStub
 * @see TaskMuxer
 */

#pragma once

#include <array>
#include <string>
#include <type_traits>
#include <utility>

#include "autonomy/bridge/grpc/task_types.hpp"
#include "autonomy/common/function_traits.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Single-command dispatch rule.
 *
 * @tparam Enum    Command enum type.
 * @tparam Handler Functor invoked on match (optionally returns `bool`).
 */
template <typename Enum, typename Handler>
struct CommandRule {
    /**
     * @brief Shared / weak / unique pointer aliases for CommandRule.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(CommandRule<Enum, Handler>)

    /**
     * @brief Command enum value this rule matches.
     */
    Enum command;

    /**
     * @brief Handler invoked when @ref command matches (void or bool).
     */
    Handler handler;
};

/**
 * @brief Build a single-command rule.
 *
 * @tparam Enum    Command enum type.
 * @tparam Handler Handler callable type.
 * @param[in] command Command enum value to match.
 * @param[in] handler Handler invoked on match.
 * @return            CommandRule owning a decayed handler.
 */
template <typename Enum, typename Handler>
CommandRule<Enum, std::decay_t<Handler>> MakeCommandRule(Enum command,
                                                         Handler&& handler) {
    return CommandRule<Enum, std::decay_t<Handler>>{
        command, std::forward<Handler>(handler)};
}

/**
 * @brief Multi-command dispatch rule (one handler for several enum values).
 *
 * @tparam Enum      Command enum type.
 * @tparam Handler   Functor invoked on match.
 * @tparam MoreEnums Additional enum values packed into `commands`.
 */
template <typename Enum, typename Handler, typename... MoreEnums>
struct CommandMultiRule {
    /**
     * @brief Shared / weak / unique pointer aliases for CommandMultiRule.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(
        CommandMultiRule<Enum, Handler, MoreEnums...>)

    /**
     * @brief Command enum values this rule matches (any one).
     */
    std::array<Enum, 1 + sizeof...(MoreEnums)> commands;

    /**
     * @brief Handler invoked when any listed command matches.
     */
    Handler handler;

    /**
     * @brief Check whether @p command is listed in this rule.
     *
     * @param[in] command Candidate command.
     * @return            true if @p command matches any stored value.
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
 *
 * @tparam Handler   Handler callable type.
 * @tparam Enum      First command enum type.
 * @tparam MoreEnums Additional command enum types (same Enum in practice).
 * @param[in] handler   Handler invoked when any listed command matches.
 * @param[in] first     First command enum value.
 * @param[in] rest      Additional command enum values.
 * @return              CommandMultiRule owning a decayed handler.
 */
template <typename Handler, typename Enum, typename... MoreEnums>
CommandMultiRule<Enum, std::decay_t<Handler>, MoreEnums...> MakeCommandRules(
    Handler&& handler, Enum first, MoreEnums... rest) {
    return CommandMultiRule<Enum, std::decay_t<Handler>, MoreEnums...>{
        {{first, rest...}}, std::forward<Handler>(handler)};
}

namespace detail {

/**
 * @brief Prefer Rule::CheckCommandMatch when available (CommandMultiRule).
 *
 * @tparam Enum Command enum type.
 * @tparam Rule CommandRule or CommandMultiRule.
 */
template <typename Enum, typename Rule>
auto CheckRuleMatch(Enum command, const Rule& rule, int)
    -> decltype(rule.CheckCommandMatch(command), bool()) {
    return rule.CheckCommandMatch(command);
}

/**
 * @brief Fallback: compare Rule::command for single-command rules.
 *
 * @tparam Enum Command enum type.
 * @tparam Rule CommandRule-like type with `.command`.
 */
template <typename Enum, typename Rule>
bool CheckRuleMatch(Enum command, const Rule& rule, long) {
    return rule.command == command;
}

}  // namespace detail

/**
 * @brief Dispatch @p command to the first matching variadic rule.
 *
 * Each rule is a `CommandRule` / `CommandMultiRule` (via `MakeCommandRule` /
 * `MakeCommandRules`). If the matched handler returns `bool`, that value is
 * propagated; otherwise a successful match yields `true`.
 *
 * @tparam Enum    Command enum type.
 * @tparam Rules   Variadic CommandRule / CommandMultiRule types.
 * @param[in] command Command enum to dispatch.
 * @param[in] rules   Dispatch rules.
 * @return            true if a rule matched and its handler succeeded (or returned void).
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
        /** @brief Decayed type of the matched rule's handler callable. */
        using Handler = std::decay_t<decltype(rule.handler)>;
        /** @brief function_traits view of Handler (arity / return_type). */
        using Traits = ::autonomy::common::function_traits::function_traits<Handler>;
        /** @brief Handler return type; `bool` means success/fail, else void. */
        using ReturnType = typename Traits::return_type;
        if constexpr (std::is_same_v<ReturnType, bool>) {
            result = static_cast<bool>(rule.handler());
        } else {
            rule.handler();
            result = true;
        }
    };
    (try_one(rules), ...);
    return matched && result;
}

/**
 * @brief Reject when emergency stop is active; emit @p make_fail and return
 * true.
 *
 * @tparam MuxerPtr   Pointer-like type with IsEstop().
 * @tparam MakeFailFn Callable taking a detail string.
 * @param[in] muxer      Task muxer (null-safe).
 * @param[in] make_fail  Emitter invoked with "emergency stop active".
 * @return               true if the caller should return immediately (rejected).
 */
template <typename MuxerPtr, typename MakeFailFn>
bool RejectOnEstop(MuxerPtr muxer, MakeFailFn&& make_fail) {
    if (muxer && muxer->IsEstop()) {
        make_fail("emergency stop active");
        return true;
    }
    return false;
}

/**
 * @brief Try muxer acquire for @p type; on failure emit and return true.
 *
 * @tparam MuxerPtr   Pointer-like type with TryAcquire().
 * @tparam Request    Request type with goal_id().
 * @tparam MakeFailFn Callable taking a detail string.
 * @param[in] muxer      Task muxer (null → never busy).
 * @param[in] type       Task type to acquire.
 * @param[in] request    Request supplying goal_id.
 * @param[in] make_fail  Emitter invoked with "another task is active".
 * @return               true if the caller should return immediately (rejected).
 */
template <typename MuxerPtr, typename Request, typename MakeFailFn>
bool RejectOnBusy(MuxerPtr muxer, TaskType type,
                           const Request& request, MakeFailFn&& make_fail) {
    if (!muxer) {
        return false;
    }
    if (!muxer->TryAcquire(type, request.goal_id(), "")) {
        make_fail("another task is active");
        return true;
    }
    return false;
}

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
