/*
 * Copyright 2026 Autodriver contributors
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

/**
 * @file
 * @brief Implements RealSense parsing helpers.
 */

#include "autodriver/drivers/hardware/realsense_types.hpp"

#include <algorithm>
#include <cctype>

namespace autodriver {
namespace hardware {
namespace realsense {
namespace {

std::string ToLower(std::string text)
{
  std::transform(
    text.begin(), text.end(), text.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

}  // namespace

StreamKind ParseStreamKind(
  const std::string & text,
  const StreamKind default_kind)
{
  const std::string value = ToLower(text);
  if (value.empty() || value == "color" || value == "rgb") {
    return StreamKind::kColor;
  }
  if (value == "depth" || value == "z16") {
    return StreamKind::kDepth;
  }
  if (value == "infrared" || value == "ir" || value == "ir1" ||
    value == "infrared1")
  {
    return StreamKind::kInfrared1;
  }
  if (value == "ir2" || value == "infrared2") {
    return StreamKind::kInfrared2;
  }
  return default_kind;
}

bool MatchesModelFilter(
  const std::string & product_name,
  const std::string & model_filter)
{
  if (model_filter.empty()) {
    return true;
  }
  const std::string name = ToLower(product_name);
  const std::string model = ToLower(model_filter);
  return name.find(model) != std::string::npos;
}

std::string EncodingForStreamKind(const StreamKind kind)
{
  switch (kind) {
    case StreamKind::kColor:
      return "rgb8";
    case StreamKind::kDepth:
      return "16UC1";
    case StreamKind::kInfrared1:
    case StreamKind::kInfrared2:
      return "mono8";
  }
  return "rgb8";
}

}  // namespace realsense
}  // namespace hardware
}  // namespace autodriver
