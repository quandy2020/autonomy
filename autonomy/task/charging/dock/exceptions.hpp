/*
 * Copyright 2026 The Openbot Authors
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
 * @file exceptions.hpp
 * @brief Docking exceptions (port of opennav_docking_core).
 */

#pragma once

#include <stdexcept>
#include <string>

namespace autonomy {
namespace task {
namespace charging {

/**
 * @class autonomy::task::charging::DockingException
 * @brief Base docking error (port of opennav_docking_core::DockingException).
 */
class DockingException : public std::runtime_error {
public:
    explicit DockingException(const std::string& description)
        : std::runtime_error(description) {}
};

class DockNotInDB : public DockingException {
public:
    explicit DockNotInDB(const std::string& description)
        : DockingException(description) {}
};

class DockNotValid : public DockingException {
public:
    explicit DockNotValid(const std::string& description)
        : DockingException(description) {}
};

class FailedToStage : public DockingException {
public:
    explicit FailedToStage(const std::string& description)
        : DockingException(description) {}
};

class FailedToDetectDock : public DockingException {
public:
    explicit FailedToDetectDock(const std::string& description)
        : DockingException(description) {}
};

class FailedToControl : public DockingException {
public:
    explicit FailedToControl(const std::string& description)
        : DockingException(description) {}
};

class FailedToCharge : public DockingException {
public:
    explicit FailedToCharge(const std::string& description)
        : DockingException(description) {}
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy
