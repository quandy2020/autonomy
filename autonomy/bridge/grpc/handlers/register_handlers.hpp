/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file register_handlers.hpp
 * @brief Variadic handler registration (fold over HandlerTs...).
 *
 * @details
 * Single helper used at BridgeServer / async_grpc ServerBuilder setup time to
 * register every RpcHandler type with one call. Expands a C++17 fold
 * expression so registration order matches the template argument pack order
 * (left-to-right). Does not construct handler instances itself — async_grpc
 * creates them per incoming RPC.
 *
 * Typical call site (conceptual):
 * @code
 * RegisterHandlers<Builder,
 *     handlers::RpcNavigateHandler,
 *     handlers::RpcNavigationCancelHandler,
 *     ...>(builder);
 * @endcode
 *
 * Invariants:
 * - Registration order is left-to-right in the HandlerTs pack.
 * - Builder must expose `template <typename H> void RegisterHandler()`.
 * - Does not own handlers; async_grpc instantiates them per RPC.
 * - No Autolink / GoalChannel topics involved here — pure type registration.
 *
 * Threading: call during single-threaded server construction before Serve().
 * Ownership: none; Builder retains registration metadata only.
 *
 * @see handler_templates.hpp
 * @see BridgeServer
 */

#pragma once

namespace autonomy {
namespace bridge {
namespace grpc {
namespace handlers {

/**
 * @brief Register a pack of async_grpc handlers onto @p builder.
 *
 * Expands to `(builder.RegisterHandler<HandlerTs>(), ...)` so every handler
 * type in the pack is registered exactly once at server build time.
 *
 * @tparam HandlerTs Handler types accepted by Builder::RegisterHandler
 *         (typically RpcHandler specializations from rpc_*_handlers.hpp).
 * @tparam BuilderT Server builder with `template <typename H> void RegisterHandler()`.
 * @param[in,out] builder Server builder receiving the registrations.
 *
 * @note HandlerTs is the explicit template pack; BuilderT is deduced from
 *       `builder` so calls like `RegisterHandlers<H1, H2>(builder)` do not bind
 *       H1 to BuilderT.
 * @note Empty HandlerTs packs are a no-op fold.
 * @warning Duplicate HandlerTs in the pack may register twice depending on
 *          Builder semantics — keep the pack unique.
 */
template <typename... HandlerTs, typename BuilderT>
void RegisterHandlers(BuilderT& builder) {
    (builder.template RegisterHandler<HandlerTs>(), ...);
}

}  // namespace handlers
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
