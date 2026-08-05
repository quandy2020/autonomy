/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QColor>
#include <QString>
#include <QVector>

#include <cstdint>
#include <deque>
#include <string>

#include "autoviz/integration/message_queue.hpp"
#include "autoviz/ui/plot/plot_types.hpp"

namespace autoviz {
namespace state_transitions {

using StateSubscriptionId = std::uint64_t;

enum class StateXAxisMode {
  kSlidingWindow = 0,
  kFixedRange = 1,
};

enum class StateMappingKind {
  kExact = 0,
  kNumericRange = 1,
};

struct StateMappingRule {
  StateMappingKind kind = StateMappingKind::kExact;
  QString match_value;
  double range_min = 0.0;
  double range_max = 0.0;
  QString label;
  QColor color = QColor(120, 120, 130);
};

struct StateSegment {
  double start_time = 0.0;
  double end_time = 0.0;
  QString state_key;
  QString label;
  QColor color;
};

struct StateTransitionSeriesConfig {
  QString channel;
  QString field_path;
  QString custom_timestamp_path;
  QString label;
  plot::PlotTimestampMode timestamp_mode = plot::PlotTimestampMode::kLogTime;
  QVector<StateMappingRule> mappings;
  bool enabled = true;
};

struct StateTransitionSeriesRuntime {
  StateTransitionSeriesConfig config;
  integration::MessageQueue queue;
  StateSubscriptionId subscription_id = 0;
  std::deque<StateSegment> segments;
  QString last_state_key;
  bool has_last_state = false;
};

struct StateTransitionPanelConfig {
  QString title = QStringLiteral("State Transitions");
  StateXAxisMode x_axis_mode = StateXAxisMode::kSlidingWindow;
  double x_window_sec = 30.0;
  double fixed_min_time = 0.0;
  double fixed_max_time = 60.0;
  bool settings_visible = false;
  QVector<StateTransitionSeriesConfig> series;
};

StateTransitionPanelConfig DefaultStateTransitionPanelConfig();

}  // namespace state_transitions
}  // namespace autoviz
