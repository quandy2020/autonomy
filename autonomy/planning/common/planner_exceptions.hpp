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

#include <stdexcept>
#include <string>
#include <memory>

namespace autonomy {
namespace planning {
namespace common {

class PlannerException : public std::runtime_error
{
public:
  explicit PlannerException(const std::string & description)
  : std::runtime_error(description) {}
};

class InvalidPlanner : public PlannerException
{
public:
  explicit InvalidPlanner(const std::string & description)
  : PlannerException(description) {}
};

class StartOccupied : public PlannerException
{
public:
  explicit StartOccupied(const std::string & description)
  : PlannerException(description) {}
};

class GoalOccupied : public PlannerException
{
public:
  explicit GoalOccupied(const std::string & description)
  : PlannerException(description) {}
};

class StartOutsideMapBounds : public PlannerException
{
public:
  explicit StartOutsideMapBounds(const std::string & description)
  : PlannerException(description) {}
};

class GoalOutsideMapBounds : public PlannerException
{
public:
  explicit GoalOutsideMapBounds(const std::string & description)
  : PlannerException(description) {}
};

class NoValidPathCouldBeFound : public PlannerException
{
public:
  explicit NoValidPathCouldBeFound(const std::string & description)
  : PlannerException(description) {}
};

class PlannerTimedOut : public PlannerException
{
public:
  explicit PlannerTimedOut(const std::string & description)
  : PlannerException(description) {}
};

class PlannerTFError : public PlannerException
{
public:
  explicit PlannerTFError(const std::string & description)
  : PlannerException(description) {}
};

class NoViapointsGiven : public PlannerException
{
public:
  explicit NoViapointsGiven(const std::string & description)
  : PlannerException(description) {}
};

class PlannerCancelled : public PlannerException
{
public:
  explicit PlannerCancelled(const std::string & description)
  : PlannerException(description) {}
};

}  // namespace common
}  // namespace planning
}  // namespace autonomy
