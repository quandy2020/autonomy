/*
 * Copyright 2026 The Openbot Authors
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

#include <stdexcept>
#include <string>

namespace autonomy {
namespace exploration {
namespace common {

/**
 * @file exploration_exceptions.hpp
 * @brief Exception types for exploration failures.
 */

/**
 * @class ExplorationException
 * @brief Base runtime_error for exploration.
 */
class ExplorationException : public std::runtime_error
{
public:
    /**
     * @brief Constructor with description.
     * @param description Error description
     */
    explicit ExplorationException(const std::string& description)
        : std::runtime_error(description)
    {
    }
};

/**
 * @class NotInitialized
 * @brief Thrown when exploration components are used before initialization.
 */
class NotInitialized : public ExplorationException
{
public:
    /**
     * @brief Constructor with description.
     * @param description Error description
     */
    explicit NotInitialized(const std::string& description)
        : ExplorationException(description)
    {
    }
};

/**
 * @class NoTarget
 * @brief Thrown when no exploration target is available.
 */
class NoTarget : public ExplorationException
{
public:
    /**
     * @brief Constructor with description.
     * @param description Error description
     */
    explicit NoTarget(const std::string& description)
        : ExplorationException(description)
    {
    }
};

/**
 * @class Finished
 * @brief Thrown / used to signal that exploration is finished.
 */
class Finished : public ExplorationException
{
public:
    /**
     * @brief Constructor with description.
     * @param description Error description
     */
    explicit Finished(const std::string& description)
        : ExplorationException(description)
    {
    }
};

}  // namespace common
}  // namespace exploration
}  // namespace autonomy
