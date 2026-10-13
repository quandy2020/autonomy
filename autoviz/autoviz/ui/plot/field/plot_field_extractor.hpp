/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file plot_field_extractor.hpp
 * @brief Extract numeric Y samples (and timestamps) from serialized messages.
 *
 * Singleton used by @ref PlotPanel when draining series queues: deserializes
 * by message type and reads the configured field path.
 *
 * @see PlotSample
 * @see message_path_navigation.hpp
 * @see plot_field_path.hpp
 */

#pragma once

#include <optional>
#include <string>
#include <vector>

namespace autoviz {
namespace plot {

/**
 * @struct PlotSample
 * @brief One extracted (timestamp, value) pair for a plot series.
 */
struct PlotSample {
  double timestamp_sec = 0.0;  /**< Sample time in seconds. */
  double value = 0.0;          /**< Numeric Y (pre-modifier). */
};

/**
 * @class PlotFieldExtractor
 * @brief Process-wide helper for protobuf field extraction into plot samples.
 *
 * Access via @ref instance(). Thread-safety depends on the underlying
 * deserializer; call from the UI / drain thread that owns the panel.
 */
class PlotFieldExtractor {
 public:
  /**
   * @brief Returns the process singleton.
   *
   * @return Reference to the shared extractor.
   */
  static PlotFieldExtractor& instance();

  /**
   * @brief Extracts a timestamped numeric sample from a payload.
   *
   * @param message_type Schema type name.
   * @param payload Serialized message bytes.
   * @param field_path Y field path (may include modifiers stripped elsewhere).
   * @param fallback_timestamp_sec Used when the message has no usable stamp.
   * @return Sample, or @c std::nullopt on parse / path failure.
   */
  std::optional<PlotSample> extract(const std::string& message_type,
                                    const std::string& payload,
                                    const std::string& field_path,
                                    double fallback_timestamp_sec) const;

  /**
   * @brief Extracts only the numeric field (no timestamp packaging).
   *
   * @param message_type Schema type name.
   * @param payload Serialized message bytes.
   * @param field_path Numeric leaf path.
   * @return Value, or @c std::nullopt on failure.
   */
  std::optional<double> extractNumeric(const std::string& message_type,
                                       const std::string& payload,
                                       const std::string& field_path) const;

  /**
   * @brief Extracts all numeric leaves matched by @c [:] expansion.
   *
   * @param message_type Schema type name.
   * @param payload Serialized message bytes.
   * @param field_path Path that may contain @c [:].
   * @return Values, or empty optional on failure / no matches.
   */
  std::optional<std::vector<double>> extractNumericAll(
      const std::string& message_type, const std::string& payload,
      const std::string& field_path) const;

  /**
   * @brief Extracts L2 norm of a vector message at @p field_path.
   *
   * @param message_type Schema type name.
   * @param payload Serialized message bytes.
   * @param field_path Path to a message with @c x/@c y/@c z children.
   * @return Norm, or @c std::nullopt on failure.
   */
  std::optional<double> extractVectorNorm(const std::string& message_type,
                                          const std::string& payload,
                                          const std::string& field_path) const;

  /**
   * @brief Extracts a timestamp from a custom path or falls back.
   *
   * @param message_type Schema type name.
   * @param payload Serialized message bytes.
   * @param timestamp_path Custom stamp field path (may be empty).
   * @param fallback_timestamp_sec Used when path missing / invalid.
   * @return Timestamp in seconds, or @c std::nullopt only on hard failure
   *         (implementation may still return the fallback).
   */
  std::optional<double> extractTimestamp(
      const std::string& message_type, const std::string& payload,
      const std::string& timestamp_path, double fallback_timestamp_sec) const;

 private:
  /** Private default constructor for the singleton. */
  PlotFieldExtractor() = default;
};

}  // namespace plot
}  // namespace autoviz
