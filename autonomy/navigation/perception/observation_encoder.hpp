/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file observation_encoder.hpp
 * @brief Encoder boundary between raw observations and DeFM features.
 */

#pragma once

#include "navigation/common/status.hpp"
#include "navigation/proto/observation.pb.h"

namespace navigation
{
namespace perception
{

/**
 * @brief Encodes one observation into a DepthFeature.
 *
 * @par Threading
 * Called from the navigation tick thread; implementations must not block on
 * network or disk after Configure().
 */
class ObservationEncoder
{
public:
    virtual ~ObservationEncoder() = default;

    /** @brief Encode @p observation into @p feature. */
    virtual common::Status Encode(const observation::Observation& observation,
                                  observation::DepthFeature* feature) = 0;

    /** @brief Drop cached state. */
    virtual void Reset() = 0;
};

}  // namespace perception
}  // namespace navigation
