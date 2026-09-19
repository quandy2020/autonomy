/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file proto_pool.hpp
 * @brief ObjectPool adapter for protobuf messages (Clear on recycle).
 *
 * @details
 * Wraps `autolink::base::ObjectPool<MessageT>` so hot stream paths can
 * reuse cleared protobuf instances instead of allocating every frame.
 * Acquire always Clear()s before returning; empty pool falls back to
 * `std::make_shared<MessageT>()`.
 *
 * @par Ownership
 * ProtoPool owns the shared ObjectPool; acquired `shared_ptr`s keep
 * recycled objects alive until released back to the pool deleter.
 *
 * @par Threading
 * Follows the underlying ObjectPool thread-safety. Callers that share one
 * ProtoPool across threads must serialize Acquire if the pool is not
 * internally synchronized.
 *
 * @par Invariants
 * - Acquire always returns a non-null shared_ptr (pool or fresh).
 * - Recycled messages are Clear()'d before reuse.
 * - Prefer for hot stream frames (e.g. Sensor Record); not for unique_ptr
 *   gRPC Send ownership.
 *
 * @see SensorStub
 */

#pragma once

#include <cstdint>
#include <memory>

#include "autolink/base/object_pool.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Fixed-size pool of MessageT; falls back to make_shared when empty.
 *
 * @tparam MessageT Default-constructible protobuf (or Clear()-able) type.
 *
 * @par Ownership
 * Sole owner of the shared ObjectPool instance held in @ref pool_.
 *
 * @par Threading
 * Thread-safety follows the underlying ObjectPool implementation.
 *
 * @note Suitable for RecordResponse / similar high-rate stream frames.
 */
template <typename MessageT>
class ProtoPool
{
public:
    /**
     * @brief Shared / weak / unique pointer aliases for ProtoPool.
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(ProtoPool<MessageT>)

    /**
     * @brief Construct a pool with @p size recycled slots.
     *
     * @details
     * The ObjectPool recycle callback Clear()s each MessageT before it is
     * returned to the free list.
     *
     * @param[in] size Initial ObjectPool capacity (default 32).
     */
    explicit ProtoPool(std::uint32_t size = 32)
        : pool_(std::make_shared<::autolink::base::ObjectPool<MessageT>>(
              size, [](MessageT* message) {
                  if (message) {
                      message->Clear();
                  }
              })) {}

    /**
     * @brief Borrow a cleared message; never returns null.
     *
     * @return shared_ptr to a Clear()'d MessageT (pooled or heap).
     *
     * @note When GetObject() fails, allocates a fresh MessageT on the heap.
     */
    std::shared_ptr<MessageT> Acquire() {
        if (auto object = pool_->GetObject()) {
            object->Clear();
            return object;
        }
        return std::make_shared<MessageT>();
    }

private:
    /**
     * @brief Underlying fixed-capacity ObjectPool of MessageT.
     *
     * @details Shared ownership so acquired objects can outlive temporary
     * ProtoPool references while still recycling into the same pool.
     */
    std::shared_ptr<::autolink::base::ObjectPool<MessageT>> pool_{nullptr};
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
