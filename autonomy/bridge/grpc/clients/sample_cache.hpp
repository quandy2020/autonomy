/*
 * Copyright 2026 The Openbot Authors
 */

/**
 * @file sample_cache.hpp
 * @brief GenericSampleCache + SensorSampleCache + SampleFieldTraits.
 *
 * @details
 * Compile-time typed latest-sample slots selected by SampleVariable tags.
 * SensorStub stores per-sensor caches; SampleFieldTraits bridges message
 * types into the matching Variable slot via message_variable_t.
 *
 * @par Ownership
 * Value-owned message slots; no heap ownership beyond the Messages tuple.
 *
 * @par Threading
 * Not mutex-protected; callers (e.g. SensorStub) must serialize access.
 *
 * @note Adding a sensor type requires a SampleVariable tag, sample_message /
 * message_variable specializations, and a SensorStub catalogue entry.
 *
 * @see variable_tags.hpp
 * @see SensorStub
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <utility>
#include "autonomy/bridge/grpc/clients/variable_tags.hpp"
#include "autonomy/bridge/grpc/clients/variable_tag.hpp"
#include "autonomy/common/helper_functions/type_traits.hpp"
#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace bridge {
namespace grpc {
namespace clients {

/**
 * @brief Typed latest-sample slots selected by VariableTs pack.
 *
 * @tparam VariableTs SampleVariable tags (must inherit SampleVariable).
 */
template <typename... VariableTs>
class GenericSampleCache
{
    static_assert(
        common::type_traits::conjunction<is_sample_variable<VariableTs>...>::
            value,
        "\n\nGenericSampleCache can only be composed of SampleVariable tags.\n\n");
    static_assert(sizeof...(VariableTs) > 0,
                  "\n\nCannot create sample cache without variables.\n\n");

    constexpr static std::int32_t kSize =
        static_cast<std::int32_t>(sizeof...(VariableTs));

public:
    AUTONOMY_SMART_PTR_DEFINITIONS(GenericSampleCache<VariableTs...>)

    /**
     * @brief Tuple of SampleVariable tags composing this cache.
     */
    using Variables = std::tuple<VariableTs...>;

    /**
     * @brief Tuple of protobuf message types parallel to Variables.
     */
    using Messages = std::tuple<sample_message_t<VariableTs>...>;

    /**
     * @brief Construct an empty cache with all slots unmarked.
     */
    GenericSampleCache() = default;

    /**
     * @brief Access sample message for VariableT.
     *
     * @tparam VariableT Sample variable tag present in this cache.
     * @return          Mutable reference to the stored message slot.
     */
    template <typename VariableT>
    sample_message_t<VariableT>& GetSample() noexcept {
        static_assert(is_sample_variable<VariableT>::value,
                      "Type is not a SampleVariable.");
        static_assert(
            common::type_traits::has_type<VariableT, Variables>::value,
            "VariableT is not part of this cache.");
        return std::get<common::type_traits::index<VariableT, Variables>::value>(
            messages_);
    }

    /**
     * @brief Const access sample message for VariableT.
     *
     * @tparam VariableT Sample variable tag present in this cache.
     * @return          Const reference to the stored message slot.
     */
    template <typename VariableT>
    const sample_message_t<VariableT>& GetSample() const noexcept {
        static_assert(is_sample_variable<VariableT>::value,
                      "Type is not a SampleVariable.");
        static_assert(
            common::type_traits::has_type<VariableT, Variables>::value,
            "VariableT is not part of this cache.");
        return std::get<common::type_traits::index<VariableT, Variables>::value>(
            messages_);
    }

    /**
     * @brief Mutable presence flag for VariableT.
     *
     * @tparam VariableT Sample variable tag present in this cache.
     * @return          Mutable bool reference (true once StoreSample succeeded).
     */
    template <typename VariableT>
    bool& HasSample() noexcept {
        static_assert(is_sample_variable<VariableT>::value,
                      "Type is not a SampleVariable.");
        static_assert(
            common::type_traits::has_type<VariableT, Variables>::value,
            "VariableT is not part of this cache.");
        return has_[common::type_traits::index<VariableT, Variables>::value];
    }

    /**
     * @brief Whether a sample has been stored for VariableT.
     *
     * @tparam VariableT Sample variable tag present in this cache.
     * @return          true if the slot has been written at least once.
     */
    template <typename VariableT>
    bool HasSample() const noexcept {
        static_assert(is_sample_variable<VariableT>::value,
                      "Type is not a SampleVariable.");
        static_assert(
            common::type_traits::has_type<VariableT, Variables>::value,
            "VariableT is not part of this cache.");
        return has_[common::type_traits::index<VariableT, Variables>::value];
    }

    /**
     * @brief Store by Variable tag.
     *
     * @tparam VariableT Sample variable tag present in this cache.
     * @param[in] message   Sample payload to store.
     */
    template <typename VariableT>
    void StoreSample(const sample_message_t<VariableT>& message) {
        GetSample<VariableT>() = message;
        HasSample<VariableT>() = true;
    }

    /**
     * @brief Store by protobuf message type (reverse-mapped to Variable).
     *
     * @tparam MessageT Sensor protobuf message.
     * @param[in] message  Sample payload to store.
     */
    template <typename MessageT>
    void StoreMessage(const MessageT& message) {
        using VariableT = message_variable_t<MessageT>;
        StoreSample<VariableT>(message);
    }

    /**
     * @brief Number of Variable slots in this cache.
     *
     * @return Compile-time slot count.
     */
    constexpr static std::int32_t GetSize() noexcept { return kSize; }

private:
    /**
     * @brief Stored latest message per Variable slot (parallel to Variables).
     */
    Messages messages_{};

    /**
     * @brief Presence flags per slot (true after StoreSample / StoreMessage).
     */
    std::array<bool, static_cast<std::size_t>(kSize)> has_{};
};

/**
 * @brief Default onboard sensor sample composition.
 */
using SensorSampleCache =
    GenericSampleCache<variable::Image, variable::CompressedImage,
                       variable::LaserScan, variable::PointCloud,
                       variable::Imu>;

/**
 * @brief Store helper used by SensorStub::SubscribeSample.
 *
 * @tparam MessageT Sensor protobuf message.
 */
template <typename MessageT>
struct SampleFieldTraits {
    AUTONOMY_SMART_PTR_DEFINITIONS(SampleFieldTraits<MessageT>)

    /**
     * @brief Forward @p message into @p cache via StoreMessage.
     *
     * @param[in,out] cache   Per-sensor sample cache.
     * @param[in]     message Incoming sensor frame.
     */
    static void StoreSample(SensorSampleCache& cache, const MessageT& message) {
        cache.StoreMessage(message);
    }
};

}  // namespace clients
}  // namespace grpc
}  // namespace bridge
}  // namespace autonomy
