/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file depth_statistics_encoder.hpp
 * @brief Model-free depth encoder for simulation fallback.
 */

#pragma once

#include <cstdint>

#include "navigation/perception/observation_encoder.hpp"

namespace navigation
{
namespace perception
{

/**
 * @brief Encodes coarse depth statistics without a neural network.
 *
 * @details Used by the simulation composition when no DeFM weights are
 * available. Produces a fixed 8-D token/pooled feature from centre, minimum,
 * mean and valid-pixel ratio.
 */
class DepthStatisticsEncoder final : public ObservationEncoder
{
public:
    /** @brief Encode the observation's depth image. */
    common::Status Encode(const observation::Observation& observation,
                          observation::DepthFeature* feature) override;

    /** @brief Reset the sequence counter. */
    void Reset() override;

private:
    /** @brief Monotonic feature sequence. */
    std::uint64_t sequence_{0};
};

}  // namespace perception
}  // namespace navigation
