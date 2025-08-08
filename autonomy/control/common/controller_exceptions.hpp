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

#include "autonomy/common/macros.hpp"

namespace autonomy {
namespace control {
namespace common {

class ControllerException : public std::runtime_error
{
public:
  explicit ControllerException(const std::string & description)
  : std::runtime_error(description) {}
};

class InvalidController : public ControllerException
{
public:
  explicit InvalidController(const std::string & description)
  : ControllerException(description) {}
};

class ControllerTFError : public ControllerException
{
public:
  explicit ControllerTFError(const std::string & description)
  : ControllerException(description) {}
};

class FailedToMakeProgress : public ControllerException
{
public:
  explicit FailedToMakeProgress(const std::string & description)
  : ControllerException(description) {}
};

class PatienceExceeded : public ControllerException
{
public:
  explicit PatienceExceeded(const std::string & description)
  : ControllerException(description) {}
};

class InvalidPath : public ControllerException
{
public:
  explicit InvalidPath(const std::string & description)
  : ControllerException(description) {}
};

class NoValidControl : public ControllerException
{
public:
  explicit NoValidControl(const std::string & description)
  : ControllerException(description) {}
};

class ControllerTimedOut : public ControllerException
{
public:
  explicit ControllerTimedOut(const std::string & description)
  : ControllerException(description) {}
};

}  // namespace common
}  // namespace control
}  // namespace autonomy