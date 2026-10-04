/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/image/image_analysis.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include <QtGlobal>

#include <automsgs/msgs/vision_msgs/detection2d.pb.h>
#include <automsgs/msgs/vision_msgs/detection2d_array.pb.h>

namespace autoviz {
namespace image {
namespace {

double LumaOf(QRgb rgba) {
  return static_cast<double>(qGray(rgba));
}

QRect ClampRoi(const QImage& image, const QRect& roi) {
  if (image.isNull()) {
    return {};
  }
  const QRect bounds(0, 0, image.width(), image.height());
  if (!roi.isValid() || roi.isEmpty()) {
    return bounds;
  }
  return roi.intersected(bounds);
}

}  // namespace

std::optional<double> metricDistance(const CameraIntrinsics& intrinsics,
                                     const QPoint& a, const QPoint& b,
                                     double depth_m) {
  if (!intrinsics.valid || depth_m <= 1e-6 || intrinsics.fx <= 1e-9 ||
      intrinsics.fy <= 1e-9) {
    return std::nullopt;
  }
  const double z = depth_m;
  const double x0 = (static_cast<double>(a.x()) - intrinsics.cx) * z /
                    intrinsics.fx;
  const double y0 = (static_cast<double>(a.y()) - intrinsics.cy) * z /
                    intrinsics.fy;
  const double x1 = (static_cast<double>(b.x()) - intrinsics.cx) * z /
                    intrinsics.fx;
  const double y1 = (static_cast<double>(b.y()) - intrinsics.cy) * z /
                    intrinsics.fy;
  return std::hypot(x1 - x0, y1 - y0);
}

ImageHistogram computeLumaHistogram(const QImage& image, const QRect& roi) {
  ImageHistogram hist;
  if (image.isNull()) {
    return hist;
  }
  const QRect area = ClampRoi(image, roi);
  if (area.isEmpty()) {
    return hist;
  }
  const QImage src =
      (image.format() == QImage::Format_ARGB32 ||
       image.format() == QImage::Format_ARGB32_Premultiplied ||
       image.format() == QImage::Format_RGB32)
          ? image
          : image.convertToFormat(QImage::Format_ARGB32);
  for (int y = area.top(); y <= area.bottom(); ++y) {
    const uchar* line = src.constScanLine(y);
    for (int x = area.left(); x <= area.right(); ++x) {
      const QRgb* pixel = reinterpret_cast<const QRgb*>(line) + x;
      const int bin = std::clamp(qGray(*pixel), 0, 255);
      ++hist.bins[static_cast<size_t>(bin)];
      ++hist.total;
    }
  }
  int peak = 0;
  for (int i = 0; i < 256; ++i) {
    if (hist.bins[static_cast<size_t>(i)] > hist.bins[static_cast<size_t>(peak)]) {
      peak = i;
    }
  }
  hist.peak_bin = peak;
  return hist;
}

ImageRoiStats computeRoiStats(const QImage& image, const QRect& roi) {
  ImageRoiStats stats;
  if (image.isNull()) {
    return stats;
  }
  const QRect area = ClampRoi(image, roi);
  if (area.isEmpty()) {
    return stats;
  }
  const QImage src =
      (image.format() == QImage::Format_ARGB32 ||
       image.format() == QImage::Format_ARGB32_Premultiplied ||
       image.format() == QImage::Format_RGB32)
          ? image
          : image.convertToFormat(QImage::Format_ARGB32);

  double sum_l = 0.0;
  double sum_l2 = 0.0;
  double sum_r = 0.0;
  double sum_g = 0.0;
  double sum_b = 0.0;
  double min_l = 255.0;
  double max_l = 0.0;
  int count = 0;

  for (int y = area.top(); y <= area.bottom(); ++y) {
    const uchar* line = src.constScanLine(y);
    for (int x = area.left(); x <= area.right(); ++x) {
      const QRgb rgba = *(reinterpret_cast<const QRgb*>(line) + x);
      const double l = LumaOf(rgba);
      sum_l += l;
      sum_l2 += l * l;
      sum_r += qRed(rgba);
      sum_g += qGreen(rgba);
      sum_b += qBlue(rgba);
      min_l = std::min(min_l, l);
      max_l = std::max(max_l, l);
      ++count;
    }
  }
  if (count <= 0) {
    return stats;
  }
  stats.pixel_count = count;
  stats.mean_luma = sum_l / count;
  stats.std_luma = std::sqrt(std::max(0.0, sum_l2 / count - stats.mean_luma * stats.mean_luma));
  stats.min_luma = min_l;
  stats.max_luma = max_l;
  stats.mean_r = sum_r / count;
  stats.mean_g = sum_g / count;
  stats.mean_b = sum_b / count;
  return stats;
}

QString formatRoiStats(const ImageRoiStats& stats) {
  if (stats.pixel_count <= 0) {
    return QStringLiteral("ROI: empty");
  }
  return QStringLiteral(
             "ROI %1px  I μ=%2 σ=%3 [%4–%5]  RGB(%6,%7,%8)")
      .arg(stats.pixel_count)
      .arg(stats.mean_luma, 0, 'f', 1)
      .arg(stats.std_luma, 0, 'f', 1)
      .arg(stats.min_luma, 0, 'f', 0)
      .arg(stats.max_luma, 0, 'f', 0)
      .arg(stats.mean_r, 0, 'f', 0)
      .arg(stats.mean_g, 0, 'f', 0)
      .arg(stats.mean_b, 0, 'f', 0);
}

QString formatMeasureReadout(double pixels,
                             const std::optional<double>& meters) {
  if (meters.has_value()) {
    return QStringLiteral("Measure: %1 px  (%2 m)")
        .arg(pixels, 0, 'f', 1)
        .arg(*meters, 0, 'f', 3);
  }
  return QStringLiteral("Measure: %1 px").arg(pixels, 0, 'f', 1);
}

QVector<double> sampleLineProfile(const QImage& image, const QPoint& a,
                                  const QPoint& b, int max_samples) {
  QVector<double> samples;
  if (image.isNull() || max_samples <= 0) {
    return samples;
  }
  const QImage src =
      (image.format() == QImage::Format_ARGB32 ||
       image.format() == QImage::Format_ARGB32_Premultiplied ||
       image.format() == QImage::Format_RGB32)
          ? image
          : image.convertToFormat(QImage::Format_ARGB32);
  const int dx = b.x() - a.x();
  const int dy = b.y() - a.y();
  const int steps = std::max(std::abs(dx), std::abs(dy));
  if (steps <= 0) {
    if (a.x() >= 0 && a.y() >= 0 && a.x() < src.width() && a.y() < src.height()) {
      samples.push_back(LumaOf(src.pixel(a)));
    }
    return samples;
  }
  const int count = std::min(steps + 1, max_samples);
  samples.reserve(count);
  for (int i = 0; i < count; ++i) {
    const double t = (count == 1) ? 0.0 : static_cast<double>(i) / (count - 1);
    const int x = static_cast<int>(std::lround(a.x() + dx * t));
    const int y = static_cast<int>(std::lround(a.y() + dy * t));
    if (x < 0 || y < 0 || x >= src.width() || y >= src.height()) {
      continue;
    }
    samples.push_back(LumaOf(src.pixel(x, y)));
  }
  return samples;
}

QImage applyLumaThresholdPreview(const QImage& image, int threshold,
                                 int highlight_alpha) {
  if (image.isNull()) {
    return {};
  }
  const int thr = std::clamp(threshold, 0, 255);
  const int alpha = std::clamp(highlight_alpha, 0, 255);
  QImage out = image.convertToFormat(QImage::Format_ARGB32);
  for (int y = 0; y < out.height(); ++y) {
    QRgb* line = reinterpret_cast<QRgb*>(out.scanLine(y));
    for (int x = 0; x < out.width(); ++x) {
      const QRgb rgba = line[x];
      const int luma = qGray(rgba);
      if (luma >= thr) {
        const int r = (qRed(rgba) * (255 - alpha) + 0 * alpha) / 255;
        const int g = (qGreen(rgba) * (255 - alpha) + 220 * alpha) / 255;
        const int b = (qBlue(rgba) * (255 - alpha) + 80 * alpha) / 255;
        line[x] = qRgba(r, g, b, qAlpha(rgba));
      } else {
        line[x] = qRgba(qRed(rgba) / 3, qGreen(rgba) / 3, qBlue(rgba) / 3,
                        qAlpha(rgba));
      }
    }
  }
  return out;
}

QImage computeFrameDiffPreview(const QImage& current, const QImage& previous,
                               double gain) {
  if (current.isNull() || previous.isNull() ||
      current.width() != previous.width() ||
      current.height() != previous.height()) {
    return {};
  }
  const double g = std::max(0.0, gain);
  const QImage a = current.convertToFormat(QImage::Format_ARGB32);
  const QImage b = previous.convertToFormat(QImage::Format_ARGB32);
  QImage out(a.size(), QImage::Format_ARGB32);
  for (int y = 0; y < a.height(); ++y) {
    const QRgb* la = reinterpret_cast<const QRgb*>(a.constScanLine(y));
    const QRgb* lb = reinterpret_cast<const QRgb*>(b.constScanLine(y));
    QRgb* lo = reinterpret_cast<QRgb*>(out.scanLine(y));
    for (int x = 0; x < a.width(); ++x) {
      const int d = static_cast<int>(std::lround(
          std::abs(static_cast<double>(qGray(la[x]) - qGray(lb[x]))) * g));
      const int v = std::clamp(d, 0, 255);
      lo[x] = qRgba(v, v, v, 255);
    }
  }
  return out;
}

namespace {

void AccumulateDetection(
    Detection2DStats* stats, std::map<QString, int>* class_counts,
    const automsgs::msgs::vision_msgs::Detection2D& det) {
  if (stats == nullptr || class_counts == nullptr) {
    return;
  }
  ++stats->total;
  double score = 0.0;
  QString class_id = QStringLiteral("(none)");
  if (det.results_size() > 0) {
    // Top hypothesis = highest score.
    int best = 0;
    for (int i = 1; i < det.results_size(); ++i) {
      if (det.results(i).hypothesis().score() >
          det.results(best).hypothesis().score()) {
        best = i;
      }
    }
    score = det.results(best).hypothesis().score();
    class_id = QString::fromStdString(det.results(best).hypothesis().class_id());
    if (class_id.isEmpty()) {
      class_id = QStringLiteral("(none)");
    }
  }
  stats->mean_score += score;
  if (stats->total == 1) {
    stats->min_score = score;
    stats->max_score = score;
  } else {
    stats->min_score = std::min(stats->min_score, score);
    stats->max_score = std::max(stats->max_score, score);
  }
  ++(*class_counts)[class_id];
}

}  // namespace

Detection2DStats summarizeDetection2D(const std::string& message_type,
                                      const std::string& payload) {
  Detection2DStats stats;
  std::map<QString, int> class_counts;
  if (message_type == "automsgs.msgs.vision_msgs.Detection2DArray" ||
      message_type == "vision_msgs/Detection2DArray") {
    automsgs::msgs::vision_msgs::Detection2DArray message;
    if (!message.ParseFromString(payload)) {
      return stats;
    }
    for (int i = 0; i < message.detections_size(); ++i) {
      AccumulateDetection(&stats, &class_counts, message.detections(i));
    }
  } else if (message_type == "automsgs.msgs.vision_msgs.Detection2D" ||
             message_type == "vision_msgs/Detection2D") {
    automsgs::msgs::vision_msgs::Detection2D message;
    if (!message.ParseFromString(payload)) {
      return stats;
    }
    AccumulateDetection(&stats, &class_counts, message);
  } else {
    return stats;
  }
  if (stats.total > 0) {
    stats.mean_score /= static_cast<double>(stats.total);
  }
  std::vector<DetectionClassCount> sorted;
  sorted.reserve(class_counts.size());
  for (const auto& entry : class_counts) {
    sorted.push_back(DetectionClassCount{entry.first, entry.second});
  }
  std::sort(sorted.begin(), sorted.end(),
            [](const DetectionClassCount& a, const DetectionClassCount& b) {
              if (a.count != b.count) {
                return a.count > b.count;
              }
              return a.class_id < b.class_id;
            });
  stats.by_class = QVector<DetectionClassCount>(sorted.begin(), sorted.end());
  return stats;
}

QString formatDetectionStats(const Detection2DStats& stats) {
  if (stats.total <= 0) {
    return QStringLiteral("Detection: —");
  }
  QStringList classes;
  const int show = std::min(4, static_cast<int>(stats.by_class.size()));
  for (int i = 0; i < show; ++i) {
    classes << QStringLiteral("%1×%2")
                   .arg(stats.by_class[i].class_id)
                   .arg(stats.by_class[i].count);
  }
  if (stats.by_class.size() > show) {
    classes << QStringLiteral("+%1").arg(stats.by_class.size() - show);
  }
  return QStringLiteral("Detection %1  score μ=%2 [%3–%4]  %5")
      .arg(stats.total)
      .arg(stats.mean_score, 0, 'f', 2)
      .arg(stats.min_score, 0, 'f', 2)
      .arg(stats.max_score, 0, 'f', 2)
      .arg(classes.join(QStringLiteral(" ")));
}

}  // namespace image
}  // namespace autoviz
