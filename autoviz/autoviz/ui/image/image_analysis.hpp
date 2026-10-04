/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_analysis.hpp
 * @brief Pixel-space analysis helpers for Image panel tools (ROI / hist / measure).
 */

#pragma once

#include <array>
#include <cmath>
#include <optional>
#include <string>

#include <QImage>
#include <QPoint>
#include <QRect>
#include <QString>
#include <QVector>

#include "autoviz/ui/image/image_calibration_utils.hpp"

namespace autoviz {
namespace image {

/**
 * @enum ImageViewTool
 * @brief Active interaction mode for @ref ImageViewWidget.
 */
enum class ImageViewTool {
  kProbe = 0,   /**< Hover probe + click publish (default). */
  kMeasure = 1, /**< Two-click distance measurement. */
  kRoi = 2,     /**< Drag rectangle for ROI statistics. */
  kProfile = 3, /**< Click two points for a luminance line profile. */
};

/**
 * @struct ImageRoiStats
 * @brief Summary statistics over an axis-aligned ROI.
 */
struct ImageRoiStats {
  int pixel_count = 0;     /**< Sampled pixels. */
  double mean_luma = 0.0;  /**< Mean luminance (0–255). */
  double std_luma = 0.0;   /**< Std-dev of luminance. */
  double min_luma = 0.0;   /**< Min luminance. */
  double max_luma = 0.0;   /**< Max luminance. */
  double mean_r = 0.0;     /**< Mean red. */
  double mean_g = 0.0;     /**< Mean green. */
  double mean_b = 0.0;     /**< Mean blue. */
};

/**
 * @struct ImageHistogram
 * @brief 256-bin luminance histogram.
 */
struct ImageHistogram {
  std::array<int, 256> bins{};  /**< Counts per intensity. */
  int total = 0;                /**< Sum of bins. */
  int peak_bin = 0;             /**< Bin with max count. */
};

/**
 * @brief Euclidean distance in image pixels.
 */
inline double pixelDistance(const QPoint& a, const QPoint& b) {
  const double dx = static_cast<double>(a.x() - b.x());
  const double dy = static_cast<double>(a.y() - b.y());
  return std::hypot(dx, dy);
}

/**
 * @brief Metric distance assuming both pixels lie on a plane at optical depth
 *        @p depth_m (pinhole unprojection).
 *
 * @param intrinsics Valid camera intrinsics.
 * @param a First pixel.
 * @param b Second pixel.
 * @param depth_m Plane depth along optical Z (meters); must be > 0.
 * @return Distance in meters, or nullopt when calibration / depth invalid.
 */
std::optional<double> metricDistance(const CameraIntrinsics& intrinsics,
                                     const QPoint& a, const QPoint& b,
                                     double depth_m);

/**
 * @brief Computes luminance histogram over @p image, optionally clipped to @p roi.
 *
 * @param image Source display frame.
 * @param roi When non-null and valid, only that rectangle is sampled.
 */
ImageHistogram computeLumaHistogram(const QImage& image, const QRect& roi = {});

/**
 * @brief Computes ROI statistics over @p roi (intersected with image bounds).
 */
ImageRoiStats computeRoiStats(const QImage& image, const QRect& roi);

/**
 * @brief Formats ROI stats for the analysis status line.
 */
QString formatRoiStats(const ImageRoiStats& stats);

/**
 * @brief Formats a measure readout (pixels [+ meters]).
 */
QString formatMeasureReadout(double pixels,
                             const std::optional<double>& meters);

/**
 * @brief Samples luminance along the segment @p a → @p b (inclusive).
 *
 * @param image Source display frame.
 * @param a Start pixel.
 * @param b End pixel.
 * @param max_samples Cap on returned samples (evenly subsampled when longer).
 * @return Luminance samples in \[0, 255\]; empty when inputs invalid.
 */
QVector<double> sampleLineProfile(const QImage& image, const QPoint& a,
                                  const QPoint& b, int max_samples = 512);

/**
 * @brief Tints pixels whose luminance is ≥ @p threshold (mask preview).
 *
 * Below-threshold pixels are dimmed; above-threshold pixels keep luma and get a
 * green highlight. Source data is not modified for export of the raw frame.
 *
 * @param image Display frame.
 * @param threshold Luma cutoff in \[0, 255\].
 * @param highlight_alpha Tint alpha for the mask (0–255).
 */
QImage applyLumaThresholdPreview(const QImage& image, int threshold,
                                 int highlight_alpha = 140);

/**
 * @brief Absolute per-pixel difference |current − previous|, gain-scaled.
 *
 * @param current Latest display frame.
 * @param previous Previous display frame (same size required).
 * @param gain Multiplier applied to |Δluma| before clamping to 255.
 * @return Grayscale diff preview, or null image when sizes mismatch / empty.
 */
QImage computeFrameDiffPreview(const QImage& current, const QImage& previous,
                               double gain = 4.0);

/**
 * @struct DetectionClassCount
 * @brief Per-class detection tally.
 */
struct DetectionClassCount {
  QString class_id;  /**< Hypothesis class_id (or "(none)"). */
  int count = 0;     /**< Detections whose top hypothesis is this class. */
};

/**
 * @struct Detection2DStats
 * @brief Aggregate stats for a Detection2D / Detection2DArray payload.
 */
struct Detection2DStats {
  int total = 0;                        /**< Detection count. */
  double mean_score = 0.0;              /**< Mean top-hypothesis score. */
  double min_score = 0.0;               /**< Min top-hypothesis score. */
  double max_score = 0.0;               /**< Max top-hypothesis score. */
  QVector<DetectionClassCount> by_class; /**< Class histogram (count desc). */
};

/**
 * @brief Summarizes Detection2D / Detection2DArray payloads; empty otherwise.
 */
Detection2DStats summarizeDetection2D(const std::string& message_type,
                                      const std::string& payload);

/**
 * @brief Formats detection stats for the analysis status line.
 */
QString formatDetectionStats(const Detection2DStats& stats);

}  // namespace image
}  // namespace autoviz
