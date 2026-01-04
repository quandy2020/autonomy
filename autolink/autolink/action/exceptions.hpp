/**
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

#include <stdexcept>
#include <string>

namespace autolink {
namespace action {

/**
 * @class ActionException
 * @brief Base exception class for action-related errors.
 */
class ActionException : public std::runtime_error
{
public:
    explicit ActionException(const std::string& message)
        : std::runtime_error(message) {}
};

/**
 * @class UnknownGoalHandleError
 * @brief Exception thrown when a goal handle is unknown or invalid.
 */
class UnknownGoalHandleError : public ActionException
{
public:
    explicit UnknownGoalHandleError(
        const std::string& message = "Unknown goal handle")
        : ActionException(message) {}
};

/**
 * @class UnawareGoalHandleError
 * @brief Exception thrown when a goal handle is not aware of the result.
 */
class UnawareGoalHandleError : public ActionException
{
public:
    explicit UnawareGoalHandleError(
        const std::string& message = "Goal handle is not aware of the result")
        : ActionException(message) {}
};

}  // namespace action
}  // namespace autolink
