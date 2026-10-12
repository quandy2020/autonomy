/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
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

#include "examples/legged_robot/terrain/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace automanip {
namespace legged_robot {

Terrain Terrain::flat() {
  return grid(-1.0, -1.0, 1.0, 2, 2, std::vector<double>(4, 0.0), 0.575, 0.4, 0.25, "flat");
}

Terrain Terrain::grid(double origin_x, double origin_y, double resolution, int cols, int rows, std::vector<double> raw,
                      double com_height, double max_slope, double smooth_radius, std::string description) {
  Terrain terrain;
  terrain.origin_x_ = origin_x;
  terrain.origin_y_ = origin_y;
  terrain.resolution_ = resolution > 0.0 ? resolution : 0.05;
  terrain.cols_ = std::max(2, cols);
  terrain.rows_ = std::max(2, rows);
  terrain.com_height_ = com_height;
  terrain.max_slope_ = max_slope;
  terrain.description_ = std::move(description);
  const std::size_t count = static_cast<std::size_t>(terrain.cols_ * terrain.rows_);
  if (raw.size() != count) {
    raw.assign(count, 0.0);
  }
  terrain.raw_ = std::move(raw);
  terrain.finish(smooth_radius);
  return terrain;
}

double Terrain::cell(int ix, int iy) const {
  if (ix < 0 || iy < 0 || ix >= cols_ || iy >= rows_ || raw_.empty()) {
    return 0.0;
  }
  return raw_[static_cast<std::size_t>(iy * cols_ + ix)];
}

double Terrain::sample(const std::vector<double>& field, double x, double y) const {
  if (cols_ < 1 || rows_ < 1 || !(resolution_ > 0.0) || field.empty()) {
    return 0.0;
  }
  const double fx = (x - origin_x_) / resolution_;
  const double fy = (y - origin_y_) / resolution_;
  if (fx < 0.0 || fy < 0.0 || fx > static_cast<double>(cols_ - 1) || fy > static_cast<double>(rows_ - 1)) {
    return 0.0;
  }
  const int x0 = std::min(static_cast<int>(std::floor(fx)), cols_ - 1);
  const int y0 = std::min(static_cast<int>(std::floor(fy)), rows_ - 1);
  const int x1 = std::min(x0 + 1, cols_ - 1);
  const int y1 = std::min(y0 + 1, rows_ - 1);
  const double tx = fx - static_cast<double>(x0);
  const double ty = fy - static_cast<double>(y0);
  const auto at = [&](int ix, int iy) { return field[static_cast<std::size_t>(iy * cols_ + ix)]; };
  const double z0 = at(x0, y0) * (1.0 - tx) + at(x1, y0) * tx;
  const double z1 = at(x0, y1) * (1.0 - tx) + at(x1, y1) * tx;
  return z0 * (1.0 - ty) + z1 * ty;
}

double Terrain::height(double x, double y) const { return sample(raw_, x, y); }

double Terrain::smoothHeight(double x, double y) const { return sample(smooth_, x, y); }

vector3_t Terrain::normal(double x, double y) const {
  const double step = std::max(resolution_, 0.05);
  const double gx = (smoothHeight(x + step, y) - smoothHeight(x - step, y)) / (2.0 * step);
  const double gy = (smoothHeight(x, y + step) - smoothHeight(x, y - step)) / (2.0 * step);
  const double slope = std::hypot(gx, gy);
  const double limit = std::tan(std::max(0.0, max_slope_));
  double sx = gx;
  double sy = gy;
  if (slope > limit && slope > 1e-9) {
    sx *= limit / slope;
    sy *= limit / slope;
  }
  vector3_t n(-sx, -sy, 1.0);
  const double norm = n.norm();
  if (norm > 1e-9) {
    n /= norm;
  } else {
    n = vector3_t::UnitZ();
  }
  return n;
}

void Terrain::finish(double smooth_radius) {
  smooth_ = raw_;
  double lowest = 0.0;
  double highest = 0.0;
  for (double value : raw_) {
    lowest = std::min(lowest, value);
    highest = std::max(highest, value);
  }
  relief_ = (highest - lowest) > 1e-4;
  if (!relief_ || cols_ < 1 || rows_ < 1 || !(resolution_ > 0.0)) {
    return;
  }
  const int radius = std::max(1, static_cast<int>(std::ceil(smooth_radius / resolution_)));
  std::vector<double> blurred(raw_.size(), 0.0);
  for (int iy = 0; iy < rows_; ++iy) {
    for (int ix = 0; ix < cols_; ++ix) {
      double sum = 0.0;
      int count = 0;
      for (int dy = -radius; dy <= radius; ++dy) {
        const int y = std::min(rows_ - 1, std::max(0, iy + dy));
        for (int dx = -radius; dx <= radius; ++dx) {
          const int x = std::min(cols_ - 1, std::max(0, ix + dx));
          sum += raw_[static_cast<std::size_t>(y * cols_ + x)];
          ++count;
        }
      }
      blurred[static_cast<std::size_t>(iy * cols_ + ix)] = count > 0 ? sum / static_cast<double>(count) : 0.0;
    }
  }
  smooth_.swap(blurred);
}

namespace {

double stairHeight(double x, double x0, double tread, double rise, int up_count, double landing, int down_count) {
  if (!(tread > 0.0) || up_count < 1) {
    return 0.0;
  }
  const double up_end = x0 + static_cast<double>(up_count) * tread;
  const double land_end = up_end + std::max(0.0, landing);
  const double down_end = land_end + static_cast<double>(std::max(0, down_count)) * tread;
  if (x < x0 || x >= down_end) {
    return 0.0;
  }
  if (x < up_end) {
    int step = static_cast<int>(std::floor((x - x0) / tread));
    step = std::max(0, std::min(up_count - 1, step));
    return static_cast<double>(step + 1) * rise;
  }
  if (x < land_end || down_count < 1) {
    return static_cast<double>(up_count) * rise;
  }
  int step = static_cast<int>(std::floor((x - land_end) / tread));
  step = std::max(0, std::min(down_count - 1, step));
  return static_cast<double>(down_count - step - 1) * rise;
}

Terrain stairsGrid(const boost::property_tree::ptree& pt) {
  const double origin_x = pt.get<double>("terrain.originX", -1.5);
  const double origin_y = pt.get<double>("terrain.originY", -1.5);
  const double resolution = pt.get<double>("terrain.resolution", 0.05);
  const double size_x = pt.get<double>("terrain.sizeX", 8.0);
  const double size_y = pt.get<double>("terrain.sizeY", 3.0);
  const double com_height = pt.get<double>("terrain.comHeight", 0.575);
  const double max_slope = pt.get<double>("terrain.maxSlope", 0.4);
  const double smooth_radius = pt.get<double>("terrain.smoothRadius", 0.25);
  const double x0 = pt.get<double>("terrain.stairs.x0", 1.0);
  const double y0 = pt.get<double>("terrain.stairs.y0", -1.0);
  const double width = pt.get<double>("terrain.stairs.width", 2.0);
  const double tread = pt.get<double>("terrain.stairs.tread", 0.32);
  const double rise = pt.get<double>("terrain.stairs.rise", 0.07);
  const int up_count = pt.get<int>("terrain.stairs.upCount", 4);
  const double landing = pt.get<double>("terrain.stairs.landing", 0.6);
  const int down_count = pt.get<int>("terrain.stairs.downCount", 4);

  const int cols = std::max(2, static_cast<int>(std::ceil(size_x / std::max(resolution, 1e-3))));
  const int rows = std::max(2, static_cast<int>(std::ceil(size_y / std::max(resolution, 1e-3))));
  std::vector<double> raw(static_cast<std::size_t>(cols * rows), 0.0);
  for (int iy = 0; iy < rows; ++iy) {
    const double y = origin_y + (static_cast<double>(iy) + 0.5) * resolution;
    for (int ix = 0; ix < cols; ++ix) {
      const double x = origin_x + (static_cast<double>(ix) + 0.5) * resolution;
      if (y < y0 || y > y0 + width) {
        continue;
      }
      raw[static_cast<std::size_t>(iy * cols + ix)] = stairHeight(x, x0, tread, rise, up_count, landing, down_count);
    }
  }
  std::ostringstream label;
  label << "stairs up " << up_count << " down " << down_count << " rise " << rise;
  return Terrain::grid(origin_x, origin_y, resolution, cols, rows, std::move(raw), com_height, max_slope, smooth_radius,
                       label.str());
}

}  // namespace

Terrain Terrain::loadHeightFile(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    std::cerr << "[Terrain] height file not found: " << path << std::endl;
    return flat();
  }
  double origin_x = 0.0;
  double origin_y = 0.0;
  double resolution = 0.05;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream header(line);
    if (!(header >> origin_x >> origin_y >> resolution) || !(resolution > 0.0)) {
      std::cerr << "[Terrain] height file header must be: origin_x origin_y resolution" << std::endl;
      return flat();
    }
    break;
  }
  std::vector<std::vector<double>> rows;
  int cols = 0;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream row(line);
    std::vector<double> values;
    double value = 0.0;
    while (row >> value) {
      values.push_back(value);
    }
    if (values.empty()) {
      continue;
    }
    if (cols == 0) {
      cols = static_cast<int>(values.size());
    }
    values.resize(static_cast<std::size_t>(cols), values.back());
    rows.push_back(std::move(values));
  }
  if (rows.empty() || cols < 2) {
    std::cerr << "[Terrain] height file has no grid: " << path << std::endl;
    return flat();
  }
  std::vector<double> raw;
  raw.reserve(rows.size() * static_cast<std::size_t>(cols));
  for (const auto& row : rows) {
    raw.insert(raw.end(), row.begin(), row.end());
  }
  return grid(origin_x, origin_y, resolution, cols, static_cast<int>(rows.size()), std::move(raw), 0.575, 0.4, 0.25,
              "grid " + path);
}

Terrain Terrain::load(const std::string& info_file) {
  boost::property_tree::ptree pt;
  try {
    boost::property_tree::read_info(info_file, pt);
  } catch (const std::exception& ex) {
    std::cerr << "[Terrain] " << ex.what() << std::endl;
    return flat();
  }
  const std::string type = pt.get<std::string>("terrain.type", "flat");
  const double com_height = pt.get<double>("terrain.comHeight", 0.575);
  if (type == "grid") {
    const std::string height_file = pt.get<std::string>("terrain.heightFile", "");
    boost::filesystem::path path(height_file);
    if (!path.is_absolute()) {
      path = boost::filesystem::path(info_file).parent_path() / path;
    }
    Terrain terrain = loadHeightFile(path.string());
    terrain.com_height_ = com_height;
    terrain.max_slope_ = pt.get<double>("terrain.maxSlope", terrain.max_slope_);
    return terrain;
  }
  if (type == "stairs") {
    try {
      return stairsGrid(pt);
    } catch (const std::exception& ex) {
      std::cerr << "[Terrain] stairs: " << ex.what() << std::endl;
      return flat();
    }
  }
  Terrain terrain = flat();
  terrain.com_height_ = com_height;
  return terrain;
}

}  // namespace legged_robot
}  // namespace automanip
