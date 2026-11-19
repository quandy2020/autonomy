// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file row_swath_generator.cpp
 * @brief Native complete-coverage row swath generator implementation. */
#include "autonomy/task/navigation/coverage/row_swath_generator.hpp"

#include "autonomy/task/navigation/coverage/generator_utils.hpp"
namespace autonomy::task::coverage {
F2CSwaths GenerateRowSwaths(const proto::CoverageRequest& request,
                            const F2CPoint& reference) {
  if (request.rows_size() < 2)
    throw RequestException("Row coverage needs at least two rows");
  auto mode = Upper(request.row_swath_mode().mode());
  if (Unknown(mode)) mode = "CENTER";
  if (mode != "CENTER" && mode != "OFFSET" && mode != "ROWSARESWATHS")
    throw CoverageException("Row mode must be CENTER, OFFSET or ROWSARESWATHS");
  const double offset = request.row_swath_mode().offset();
  if (!std::isfinite(offset) || offset < 0)
    throw RequestException("Row offset must be finite and nonnegative");
  std::set<int> skip(request.row_swath_mode().skip_ids().begin(),
                     request.row_swath_mode().skip_ids().end());
  std::vector<std::vector<F2CPoint>> rows;
  for (const auto& row : request.rows()) {
    if (row.coordinates_size() < 2)
      throw RequestException("Row needs at least two points");
    std::vector<F2CPoint> points;
    for (const auto& p : row.coordinates()) {
      Angle(p.axis1());
      Angle(p.axis2());
      points.emplace_back(F2CPoint(p.axis1(), p.axis2()) - reference);
    }
    if (!rows.empty() && points.size() != rows[0].size())
      throw RequestException("Rows must have matching sample counts");
    if (!rows.empty() && points.front().Distance(rows.back().back()) <
                             points.front().Distance(rows.back().front()))
      std::reverse(points.begin(), points.end());
    rows.push_back(std::move(points));
  }
  F2CSwaths output;
  const size_t count = mode == "ROWSARESWATHS" ? rows.size() : rows.size() - 1;
  for (size_t i = 0; i < count; ++i) {
    const int id = request.rows(static_cast<int>(i)).id();
    if (skip.count(id)) continue;
    const size_t other =
        mode == "ROWSARESWATHS" ? (i + 1 < rows.size() ? i + 1 : i - 1) : i + 1;
    F2CLineString line;
    double width = std::numeric_limits<double>::infinity();
    for (size_t j = 0; j < rows[i].size(); ++j) {
      const auto a = rows[i][j], b = rows[other][j];
      const double d = a.Distance(b);
      Positive(d, "row separation");
      width = std::min(width, d);
      if (mode == "OFFSET" && offset > d)
        throw RequestException("Offset leaves the row corridor");
      line.AddPoint(mode == "CENTER"   ? (a + b) * 0.5
                    : mode == "OFFSET" ? a + (b - a) * (offset / d)
                                       : a);
    }
    output.emplace_back(line, width, id);
  }
  return output;
}

}  // namespace autonomy::task::coverage
