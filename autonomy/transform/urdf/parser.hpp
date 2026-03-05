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

#include <string>

#include "urdf_world/types.h"

namespace urdf {

/** \brief Base class for URDF parsers */
class URDFParser {
 public:
  URDFParser() = default;
  virtual ~URDFParser() = default;

  /// \brief Load Model from string
  /// \return nullptr and write to stderr if the given string is invalid
  virtual ModelInterfaceSharedPtr parse(const std::string& data) = 0;

  /// \brief Indicate if data is meant to be parsed by this parser
  /// \return The position in the string that the plugin became confident the
  ///         data is intended to be parsed by it.
  ///         For example, the plugin parsing COLLADA files might return the
  ///         position in the string that the '<COLLADA>' xml tag was found.
  ///         Smaller values are interpretted as more confidence, and the
  ///         plugin with the smallest value is used to parse the data.
  ///         If a plugin believes data is not meant for it, then it should
  ///         return a value greater than or equal to data.size().
  virtual std::size_t might_handle(const std::string& data) = 0;
};

}  // namespace urdf
