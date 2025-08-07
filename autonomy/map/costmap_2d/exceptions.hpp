/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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
#include <memory>


namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class CollisionCheckerException
 * @brief Exceptions thrown if collision checker determines a pose is in
 * collision with the environment costmap
 */
class CollisionCheckerException : public std::runtime_error
{
public:
  explicit CollisionCheckerException(const std::string description)
  : std::runtime_error(description) {}
};

/**
 * @class IllegalPoseException
 * @brief Thrown when CollisionChecker encounters a fatal error
 */
class IllegalPoseException : public CollisionCheckerException
{
public:
  IllegalPoseException(const std::string name, const std::string description)
  : CollisionCheckerException(description), name_(name) {}
  std::string getCriticName() const {return name_;}

protected:
  std::string name_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy