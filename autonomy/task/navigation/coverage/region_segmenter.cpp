// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file region_segmenter.cpp
 * @brief Grid region segmentation implementation. */
#include "autonomy/task/navigation/coverage/region_segmenter.hpp"

#include <algorithm>
#include <limits>

#include "autonomy/task/navigation/coverage/generator_utils.hpp"

#include <opencv2/imgproc.hpp>

namespace autonomy::task::coverage {
namespace {

}  // namespace

double RegionSegmenter::Area(const F2CCells& cells) {
  double area = 0.0;
  for (const auto& cell : cells) {
    area += cell.Area();
  }
  return area;
}

std::vector<F2CCells> RegionSegmenter::MapSplit(
    const proto::OccupancyMap& map, const MapOptions& options) {
  const int width = static_cast<int>(map.width());
  const int height = static_cast<int>(map.height());
  if (width <= 0 || height <= 0 || map.resolution() <= 0.f ||
      map.data_size() != width * height) {
    throw RequestException("Map segmentation requires a full occupancy grid");
  }
  cv::Mat free_space(height, width, CV_8UC1, cv::Scalar(0));
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int value = map.data(y * width + x);
      if (value >= 0 && value < options.occupied_threshold) {
        free_space.at<unsigned char>(y, x) = 255;
      }
    }
  }
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(free_space, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  const double resolution = static_cast<double>(map.resolution());
  const double c = std::cos(map.origin_yaw());
  const double s = std::sin(map.origin_yaw());
  struct Room {
    double area;
    F2CLinearRing ring;
  };
  std::vector<Room> rooms;
  for (const auto& contour : contours) {
    if (contour.size() < 3) {
      continue;
    }
    double pixel_area = 0.0;
    for (std::size_t index = 0; index < contour.size(); ++index) {
      const auto& current = contour[index];
      const auto& next = contour[(index + 1) % contour.size()];
      pixel_area += static_cast<double>(current.x) * next.y -
                    static_cast<double>(next.x) * current.y;
    }
    const double area =
        std::abs(pixel_area) * 0.5 * resolution * resolution;
    if (area < options.min_area) {
      continue;
    }
    F2CLinearRing ring;
    for (const auto& point : contour) {
      const double x = static_cast<double>(point.x) * resolution;
      const double y = static_cast<double>(point.y) * resolution;
      ring.AddPoint(map.origin_x() + c * x - s * y,
                    map.origin_y() + s * x + c * y);
    }
    ring.CloseRing();
    rooms.push_back(Room{area, ring});
  }
  std::sort(rooms.begin(), rooms.end(),
            [](const Room& a, const Room& b) { return a.area > b.area; });
  if (rooms.size() > static_cast<std::size_t>(options.max_regions)) {
    rooms.resize(static_cast<std::size_t>(options.max_regions));
  }
  std::vector<F2CCells> result;
  result.reserve(rooms.size());
  for (auto& room : rooms) {
    result.push_back(F2CCells(F2CCell(room.ring)));
  }
  if (result.empty()) {
    throw RequestException("Map segmentation found no free-space rooms");
  }
  return result;
}

std::vector<F2CCells> RegionSegmenter::GridSplit(const F2CCells& field,
                                                 const Options& options) {
  if (options.columns < 1 || options.columns > 8 || options.rows < 1 ||
      options.rows > 8) {
    throw RequestException("Region grid must be 1..8 by 1..8");
  }
  if (field.IsEmpty() || Area(field) <= 1e-6) {
    throw RequestException("Region split requires a non-empty field");
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (std::size_t index = 0; index < field.size(); ++index) {
    for (const auto& point : field.GetCellBorder(index)) {
      min_x = std::min(min_x, point.GetX());
      min_y = std::min(min_y, point.GetY());
      max_x = std::max(max_x, point.GetX());
      max_y = std::max(max_y, point.GetY());
    }
  }
  const double width = (max_x - min_x) / static_cast<double>(options.columns);
  const double height = (max_y - min_y) / static_cast<double>(options.rows);
  if (width <= 1e-6 || height <= 1e-6) {
    throw RequestException("Field is too small for the requested region grid");
  }

  std::vector<F2CCells> regions;
  regions.reserve(static_cast<std::size_t>(options.columns * options.rows));
  for (int row = 0; row < options.rows; ++row) {
    for (int column = 0; column < options.columns; ++column) {
      const double left = min_x + width * static_cast<double>(column);
      const double bottom = min_y + height * static_cast<double>(row);
      F2CLinearRing ring;
      ring.AddPoint(left, bottom);
      ring.AddPoint(left + width, bottom);
      ring.AddPoint(left + width, bottom + height);
      ring.AddPoint(left, bottom + height);
      ring.CloseRing();
      regions.push_back(F2CCells(F2CCell(ring)));
    }
  }
  if (regions.empty()) {
    throw RequestException("Region split produced no regions");
  }
  return regions;
}

}  // namespace autonomy::task::coverage
