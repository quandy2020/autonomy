/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

#pragma once

/**
 * @file visualizer.hpp
 * @brief Fields2Cover visualizer.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include <vector>
#include <string>
#include <type_traits>
#include "autonomy/planning/planner/fields2cover/types.hpp"

namespace autonomy {
namespace planning {
namespace planner {
namespace fields2cover {


/**
 * @class autonomy::planning::planner::fields2cover::Visualizer
 * @brief Class to plot Fields2Cover data structures.
 */
class Visualizer {
 public:
  static double GetLineWidth() {return 1.0;}

  static void plot(double x, double y, const std::vector<double>& color = {});
  static void plot(const F2CPoint& p, const std::vector<double>& color = {});
  static void plot(
      const F2CSwath& s, const std::vector<double>& color = {});
  static void plot(const F2CSwaths& s,
      const std::vector<std::vector<double>>& color = {});
  static void plot(const F2CSwathsByCells& s,
      const std::vector<std::vector<double>>& color = {});
  static void plot(const std::vector<F2CSwathsByCells>& vs,
      const std::vector<std::vector<double>>& color = {});
  static void plot(const F2CLineString& line,
      const std::vector<double>& color = {});
  static void plot(const F2CLinearRing& ring,
      const std::vector<double>& color = {});
  static void plot(const F2CMultiPoint& multipoint,
      const std::vector<double>& color = {});
  static void plot(const F2CMultiLineString& multiline,
      const std::vector<double>& color = {});
  static void plot(const F2CCell& cell,
      const std::vector<double>& color = {});
  static void plot(const F2CCells& cells,
      const std::vector<double>& color = {});
  static void plot(const F2CRoute& route,
      const std::vector<std::vector<double>>& color = {});
  static void plot(const std::vector<F2CRoute>& route,
      const std::vector<std::vector<double>>& color = {});
  static void plot(const F2CPath& path,
      const std::vector<std::vector<double>>& color = {});
  static void plot(const F2CField& field,
      const std::vector<double>& color = {});

  static void PlotFilled(const F2CField& field,
      const std::vector<double>& color = {});
  static void PlotFilled(
      const F2CLinearRing& ring, const std::vector<double>& color);
  static void PlotFilled(const F2CCell& cell,
      const std::vector<double>& poly_color,
      const std::vector<double>& holes_color);
  static void PlotFilled(const F2CCells& cells,
      const std::vector<double>& poly_color,
      const std::vector<double>& holes_color);
  static void PlotFilled(const F2CCell& cell,
      const std::vector<double>& color);
  static void PlotFilled(const F2CCells& cells,
      const std::vector<double>& poly_color);

  static void plot(const std::vector<double>& t,
      const std::vector<double>& d,  const std::vector<double>& color);

  /**
   * @brief Plot vector of type T.
   */
  template<class T>
  static void plot(const std::vector<T>& v_t, const std::vector<double>& color);
  template<class T>
  static void plot(
      const std::vector<T>& v_t,
      const std::vector<std::vector<double>>& color = {});

  /**
   * @brief Create figure to plot on.
   */
  static void figure();

  /**
   * @brief Change size of current figure.
   */
  static void figure_size(unsigned int width, unsigned int height);

  static void axis_equal();

  /**
   * @brief Show the figure on screen.
   * @param[in] block if true the program will wait until the screen is closed
   * to continue
   */
  static void show();

  /**
   * @brief Save figure to a file.
   * @param[in] file Name of the file
   */
  static void save(const std::string& file);

  /**
   * @brief Add title to the figure.
   * @param[in] text Title text
   */
  static void title(const std::string& text);

  static void xlim(double min, double max);
  static void ylim(double min, double max);

 private:
  template<class T>
  static std::vector<F2CPoint> data2vector(const T& t);

  static std::vector<std::vector<double>> GetComponents(
      const std::vector<F2CPoint>& points);

  static std::vector<double> linspace(double min, double max, size_t N);
  static std::vector<std::vector<double>> color_linspace(
      const std::vector<int>& min,
      const std::vector<int>& max,
      size_t N);
};

template<class T>
std::vector<F2CPoint> Visualizer::data2vector(const T& t) {
  if constexpr (std::is_same<T, F2CPoint>::value) {
    return std::vector<F2CPoint>{t};
  } else {
    std::vector<F2CPoint> res;
    for (auto&& i : t) {
      auto v = data2vector(i);
      res.insert(res.end(), v.begin(), v.end());
    }
    return res;
  }
}

template<class T>
void Visualizer::plot(
    const std::vector<T>& v_t,
    const std::vector<double>& color) {
  if constexpr (std::is_same<T, F2CPoint>::value) {
    auto comp = GetComponents(v_t);
    plot(comp[0], comp[1], color);
  } else if constexpr (std::is_same<T, double>::value) {
    std::vector<double> t(v_t.size());
    std::iota(std::begin(t), std::end(t), 0.0);
    plot(t, v_t, color);
  } else {
    for (auto&& t : v_t) {
      plot(t, color);
    }
  }
}

}  // namespace fields2cover
}  // namespace planner
}  // namespace planning
}  // namespace autonomy


