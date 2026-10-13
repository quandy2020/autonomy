/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/core/plot_series_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "autoviz/ui/plot/core/plot_time_join.hpp"
#include "autoviz/ui/plot/core/plot_xy_align.hpp"
#include "autoviz/ui/plot/field/plot_field_extractor.hpp"
#include "autoviz/ui/plot/field/plot_field_path.hpp"

namespace autoviz {
namespace plot {
namespace {

int EffectiveMaxPoints(const SeriesPipelineContext& ctx) {
  return std::max(100, ctx.max_points_per_series);
}

std::optional<double> ApplyBinaryOp(PlotBinaryOp op, double a, double b) {
  switch (op) {
    case PlotBinaryOp::kAdd:
      return a + b;
    case PlotBinaryOp::kSub:
      return a - b;
    case PlotBinaryOp::kMul:
      return a * b;
    case PlotBinaryOp::kDiv:
      if (std::abs(b) <= 1e-15) {
        return std::nullopt;
      }
      return a / b;
    case PlotBinaryOp::kNone:
    default:
      return a;
  }
}

std::optional<double> ExtractSeriesRawY(const std::string& message_type,
                                        const std::string& payload,
                                        const ParsedFieldPath& path,
                                        bool has_norm) {
  if (has_norm) {
    return PlotFieldExtractor::instance().extractVectorNorm(
        message_type, payload, path.base_path.toStdString());
  }
  return PlotFieldExtractor::instance().extractNumeric(
      message_type, payload, path.base_path.toStdString());
}

std::vector<double> ExtractPrimaryYs(const std::string& message_type,
                                     const std::string& payload,
                                     const ParsedFieldPath& y_path,
                                     bool y_has_norm, bool y_expand) {
  std::vector<double> raw_ys;
  if (y_has_norm) {
    const std::optional<double> norm =
        PlotFieldExtractor::instance().extractVectorNorm(
            message_type, payload, y_path.base_path.toStdString());
    if (norm.has_value()) {
      raw_ys.push_back(*norm);
    }
    return raw_ys;
  }
  if (y_expand) {
    const std::optional<std::vector<double>> values =
        PlotFieldExtractor::instance().extractNumericAll(
            message_type, payload, y_path.base_path.toStdString());
    if (values.has_value()) {
      return *values;
    }
    return raw_ys;
  }
  const std::optional<double> raw_y = PlotFieldExtractor::instance().extractNumeric(
      message_type, payload, y_path.base_path.toStdString());
  if (raw_y.has_value()) {
    raw_ys.push_back(*raw_y);
  }
  return raw_ys;
}

double ApplyYModifiers(PlotSeriesRuntime& runtime, double raw,
                       double timestamp_sec, const QStringList& modifiers,
                       bool multi) {
  return ApplyPlotModifiers(
             raw, timestamp_sec, modifiers,
             multi ? nullptr : &runtime.last_raw_value,
             multi ? nullptr : &runtime.last_timestamp_sec,
             multi ? nullptr : &runtime.has_last_sample,
             multi ? nullptr : &runtime.modifier_window,
             multi ? nullptr : &runtime.integral_accum) *
             runtime.config.y_scale +
         runtime.config.y_offset;
}

bool DrainXySeries(PlotSeriesRuntime& runtime,
                   const SeriesPipelineContext& ctx) {
  bool changed = false;
  const int max_points = EffectiveMaxPoints(ctx);
  if (!ctx.message_type_for_channel ||
      runtime.config.x_field_path.trimmed().isEmpty()) {
    while (runtime.queue.pop()) {
    }
    while (runtime.x_queue.pop()) {
    }
    return false;
  }

  const ParsedFieldPath y_path = ParseFieldPath(runtime.config.field_path);
  const ParsedFieldPath x_path = ParseFieldPath(runtime.config.x_field_path);
  const bool y_has_norm = FieldPathHasNormModifier(y_path.modifiers);
  const QStringList y_modifiers =
      y_has_norm ? StripNormModifier(y_path.modifiers) : y_path.modifiers;
  const bool x_has_norm = FieldPathHasNormModifier(x_path.modifiers);
  const bool cross = UsesCrossChannelXy(runtime.config, ctx.x_axis_mode);

  if (cross) {
    const std::string x_channel =
        EffectiveXChannel(runtime.config).toStdString();
    const std::string x_type = ctx.message_type_for_channel(x_channel);
    while (auto x_payload = runtime.x_queue.pop()) {
      const double timestamp_sec = ResolveTimestampSec(
          runtime, x_type, *x_payload, ctx.receive_time, ctx.sim_time);
      const std::optional<double> raw_x =
          ExtractSeriesRawY(x_type, *x_payload, x_path, x_has_norm);
      if (!raw_x.has_value()) {
        continue;
      }
      const double x_val = ApplyPlotModifiers(
          *raw_x, timestamp_sec, x_path.modifiers, nullptr, nullptr, nullptr);
      PushTimedSample(runtime.x_samples, timestamp_sec, x_val, max_points);
    }
  }

  const std::string y_channel = runtime.config.channel.toStdString();
  const std::string y_type = ctx.message_type_for_channel(y_channel);
  while (auto payload = runtime.queue.pop()) {
    const double timestamp_sec = ResolveTimestampSec(
        runtime, y_type, *payload, ctx.receive_time, ctx.sim_time);

    if (!cross) {
      const std::optional<double> raw_x =
          ExtractSeriesRawY(y_type, *payload, x_path, x_has_norm);
      if (!raw_x.has_value()) {
        continue;
      }
      const double x_val = ApplyPlotModifiers(
          *raw_x, timestamp_sec, x_path.modifiers, nullptr, nullptr, nullptr);
      PushTimedSample(runtime.x_samples, timestamp_sec, x_val, max_points);
    }

    const std::vector<double> raw_ys =
        ExtractPrimaryYs(y_type, *payload, y_path, y_has_norm,
                         FieldPathHasArrayExpand(y_path.base_path));
    if (raw_ys.empty()) {
      continue;
    }

    const double y =
        ApplyYModifiers(runtime, raw_ys.front(), timestamp_sec, y_modifiers,
                        /*multi=*/false);
    PushTimedSample(runtime.secondary_samples, timestamp_sec, y, max_points);

    const std::vector<PlotPoint> aligned = AlignXyNearest(
        runtime.x_samples, runtime.secondary_samples,
        runtime.config.binary_max_dt_sec);
    for (const PlotPoint& point : aligned) {
      runtime.points.push_back(point);
      TrimSeriesPoints(runtime, ctx.x_axis_mode, ctx.x_window_sec, max_points,
                       point.t);
      changed = true;
    }
  }
  return changed;
}

bool DrainTimeSeries(PlotSeriesRuntime& runtime,
                     const SeriesPipelineContext& ctx) {
  bool changed = false;
  const int max_points = EffectiveMaxPoints(ctx);
  const std::string channel = runtime.config.channel.toStdString();
  const std::string message_type = ctx.message_type_for_channel
                                       ? ctx.message_type_for_channel(channel)
                                       : std::string();
  const ParsedFieldPath y_path = ParseFieldPath(runtime.config.field_path);
  const ParsedFieldPath x_path = ParseFieldPath(runtime.config.x_field_path);
  const bool y_has_norm = FieldPathHasNormModifier(y_path.modifiers);
  const QStringList y_modifiers =
      y_has_norm ? StripNormModifier(y_path.modifiers) : y_path.modifiers;
  const bool y_expand = FieldPathHasArrayExpand(y_path.base_path);
  const bool binary_enabled = BinaryOpEnabled(runtime.config);
  const bool cross_channel = UsesCrossChannelBinary(runtime.config);
  const ParsedFieldPath secondary_path =
      ParseFieldPath(runtime.config.secondary_field_path);
  const bool secondary_has_norm =
      FieldPathHasNormModifier(secondary_path.modifiers);

  if (cross_channel) {
    const std::string secondary_channel =
        EffectiveSecondaryChannel(runtime.config).toStdString();
    const std::string secondary_type =
        ctx.message_type_for_channel
            ? ctx.message_type_for_channel(secondary_channel)
            : std::string();
    while (auto secondary_payload = runtime.secondary_queue.pop()) {
      const double timestamp_sec =
          ResolveTimestampSec(runtime, secondary_type, *secondary_payload,
                              ctx.receive_time, ctx.sim_time);
      const std::optional<double> raw_b = ExtractSeriesRawY(
          secondary_type, *secondary_payload, secondary_path,
          secondary_has_norm);
      if (!raw_b.has_value()) {
        continue;
      }
      PlotPoint sample;
      sample.x = timestamp_sec;
      sample.y = *raw_b;
      sample.t = timestamp_sec;
      runtime.secondary_samples.push_back(sample);
      TrimSeriesPoints(runtime, ctx.x_axis_mode, ctx.x_window_sec, max_points,
                       sample.x);
    }
  }

  while (auto payload = runtime.queue.pop()) {
    const double timestamp_sec = ResolveTimestampSec(
        runtime, message_type, *payload, ctx.receive_time, ctx.sim_time);

    const std::vector<double> raw_ys = ExtractPrimaryYs(
        message_type, *payload, y_path, y_has_norm, y_expand);
    if (raw_ys.empty()) {
      continue;
    }

    std::optional<double> same_message_secondary;
    if (binary_enabled && !cross_channel) {
      same_message_secondary = ExtractSeriesRawY(
          message_type, *payload, secondary_path, secondary_has_norm);
      if (!same_message_secondary.has_value()) {
        continue;
      }
    }

    if (ctx.x_axis_mode == PlotXAxisMode::kIndex && y_expand) {
      runtime.points.clear();
      runtime.index_counter = 0;
    }

    const bool multi = raw_ys.size() > 1;
    for (std::size_t i = 0; i < raw_ys.size(); ++i) {
      double raw_combined = raw_ys[i];
      if (binary_enabled) {
        std::optional<double> raw_b;
        if (cross_channel) {
          raw_b = NearestYByTime(runtime.secondary_samples, timestamp_sec,
                                 runtime.config.binary_max_dt_sec);
        } else {
          raw_b = same_message_secondary;
        }
        if (!raw_b.has_value()) {
          continue;
        }
        const std::optional<double> combined =
            ApplyBinaryOp(runtime.config.binary_op, raw_ys[i], *raw_b);
        if (!combined.has_value()) {
          continue;
        }
        raw_combined = *combined;
      }

      const double y = ApplyYModifiers(runtime, raw_combined, timestamp_sec,
                                       y_modifiers, multi);

      PlotPoint point;
      point.t = timestamp_sec;
      if (ctx.x_axis_mode == PlotXAxisMode::kIndex) {
        point.x = runtime.index_counter++;
        point.y = y;
      } else if (ctx.x_axis_mode == PlotXAxisMode::kMessagePath) {
        if (x_path.base_path.isEmpty()) {
          continue;
        }
        const std::optional<double> raw_x =
            PlotFieldExtractor::instance().extractNumeric(
                message_type, *payload, x_path.base_path.toStdString());
        if (!raw_x.has_value()) {
          continue;
        }
        point.x = ApplyPlotModifiers(*raw_x, timestamp_sec, x_path.modifiers,
                                     nullptr, nullptr, nullptr);
        point.y = y;
        if (ctx.message_path_mode == PlotMessagePathMode::kCurrent && i == 0) {
          runtime.points.clear();
        }
      } else {
        point.x = timestamp_sec;
        point.y = y;
      }
      runtime.points.push_back(point);
      TrimSeriesPoints(runtime, ctx.x_axis_mode, ctx.x_window_sec, max_points,
                       point.x);
      changed = true;
    }
  }
  return changed;
}

}  // namespace

bool BinaryOpEnabled(const PlotSeriesConfig& config) {
  return config.binary_op != PlotBinaryOp::kNone &&
         !config.secondary_field_path.trimmed().isEmpty();
}

QString EffectiveSecondaryChannel(const PlotSeriesConfig& config) {
  const QString secondary = config.secondary_channel.trimmed();
  return secondary.isEmpty() ? config.channel : secondary;
}

bool UsesCrossChannelBinary(const PlotSeriesConfig& config) {
  if (!BinaryOpEnabled(config)) {
    return false;
  }
  return EffectiveSecondaryChannel(config) != config.channel;
}

QString EffectiveXChannel(const PlotSeriesConfig& config) {
  const QString x_channel = config.x_channel.trimmed();
  return x_channel.isEmpty() ? config.channel : x_channel;
}

bool UsesCrossChannelXy(const PlotSeriesConfig& config,
                        PlotXAxisMode x_axis_mode) {
  if (x_axis_mode != PlotXAxisMode::kXY) {
    return false;
  }
  return EffectiveXChannel(config) != config.channel;
}

double ResolveTimestampSec(const PlotSeriesRuntime& runtime,
                           const std::string& message_type,
                           const std::string& payload, double receive_time,
                           double sim_time) {
  if (runtime.config.timestamp_mode == PlotTimestampMode::kReceiveTime) {
    return receive_time;
  }
  const double fallback =
      runtime.config.timestamp_mode == PlotTimestampMode::kLogTime ? sim_time
                                                                   : receive_time;
  if (runtime.config.timestamp_mode == PlotTimestampMode::kCustomField) {
    const QString resolved_path = runtime.config.custom_timestamp_path;
    const std::optional<double> custom =
        PlotFieldExtractor::instance().extractTimestamp(
            message_type, payload, resolved_path.toStdString(), fallback);
    return custom.value_or(fallback);
  }
  const std::optional<double> header_time =
      PlotFieldExtractor::instance().extractTimestamp(message_type, payload, "",
                                                      fallback);
  return header_time.value_or(fallback);
}

void TrimSeriesPoints(PlotSeriesRuntime& runtime, PlotXAxisMode x_axis_mode,
                      double x_window_sec, int max_points, double latest_x) {
  const int limit = std::max(100, max_points);
  while (static_cast<int>(runtime.points.size()) > limit) {
    runtime.points.pop_front();
  }
  while (static_cast<int>(runtime.secondary_samples.size()) > limit) {
    runtime.secondary_samples.pop_front();
  }
  while (static_cast<int>(runtime.x_samples.size()) > limit) {
    runtime.x_samples.pop_front();
  }
  if (x_axis_mode != PlotXAxisMode::kTimestamp) {
    return;
  }
  const double min_x = latest_x - x_window_sec;
  while (!runtime.points.empty() && runtime.points.front().x < min_x) {
    runtime.points.pop_front();
  }
  while (!runtime.secondary_samples.empty() &&
         runtime.secondary_samples.front().x < min_x) {
    runtime.secondary_samples.pop_front();
  }
  while (!runtime.x_samples.empty() && runtime.x_samples.front().x < min_x) {
    runtime.x_samples.pop_front();
  }
}

bool DrainAndSampleSeries(PlotSeriesRuntime& runtime,
                          const SeriesPipelineContext& ctx, bool paused) {
  if (!runtime.config.enabled || runtime.config.channel.isEmpty() ||
      runtime.config.field_path.isEmpty()) {
    return false;
  }
  if (paused) {
    while (runtime.queue.pop()) {
    }
    while (runtime.secondary_queue.pop()) {
    }
    while (runtime.x_queue.pop()) {
    }
    return false;
  }
  if (ctx.x_axis_mode == PlotXAxisMode::kXY) {
    return DrainXySeries(runtime, ctx);
  }
  return DrainTimeSeries(runtime, ctx);
}

}  // namespace plot
}  // namespace autoviz
