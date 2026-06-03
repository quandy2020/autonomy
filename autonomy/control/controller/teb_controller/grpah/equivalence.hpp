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

#include <memory>

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

/**
 * @brief Homotopy equivalence interface for trajectory comparison
 */
class Equivalence
{
public:
    /**
     * Define Equivalence::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(Equivalence);

    /**
     * @brief Destructor for Equivalence
     */
    virtual ~Equivalence() = default;

    /**
     * @brief Check if two trajectories belong to the same homotopy class
     * @param other The other equivalence instance to compare with
     */
    virtual bool IsEqual(const Equivalence& other) const = 0;

    /**
     * @brief Check if the homotopy signature was computed successfully
     * @return false if detection failed (e.g. NaN/Inf)
     */
    virtual bool IsValid() const = 0;

    /**
     * @brief Check if trajectory does not loop around obstacles
     * @return false if the trajectory loops around an obstacle
     */
    virtual bool IsReasonable() const = 0;
};

using EquivalencePtr = Equivalence::SharedPtr;

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
