/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace autonomy {
namespace visualization {

enum class VisualizationPanel {
  kRawPlot,
  kThreeD,
  kImage,
  kMap,
};

enum class VisualizationStrategy {
  kSchemaPassThrough,
  kPayloadAdaptation,
  kRawPlotOnly,
};

struct VisualizationSchemaRule {
  std::string_view source_message_type;
  std::string_view target_schema_name;
  VisualizationPanel panel;
  VisualizationStrategy strategy;
};

class VisualizationSchemaRegistry {
 public:
  static const VisualizationSchemaRule* FindRule(
      const std::string& source_message_type);
  static std::vector<std::string> ListSourceMessageTypes(
      VisualizationPanel panel, bool include_raw_plot_only = false);
  static bool IsBridgeable(const std::string& source_message_type);
  static bool IsThreeDRenderable(const std::string& source_message_type);
  static bool RequiresPayloadAdaptation(const std::string& source_message_type);
  static std::string ResolveTargetSchemaName(
      const std::string& source_message_type);
};

}  // namespace visualization
}  // namespace autonomy
