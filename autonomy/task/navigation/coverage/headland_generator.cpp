// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file headland_generator.cpp
 * @brief Native complete-coverage headland generator implementation. */
#include "autonomy/task/navigation/coverage/headland_generator.hpp"

#include "autonomy/task/navigation/coverage/generator_utils.hpp"
namespace autonomy::task::coverage {
double HeadlandGenerator::Width(const proto::HeadlandMode& m) const {
  const auto mode =
      Unknown(m.mode()) ? Upper(config_.headland_type) : Upper(m.mode());
  if (mode != "CONSTANT")
    throw CoverageException("Headland mode must be CONSTANT");
  const double width = Unknown(m.mode()) ? config_.headland_width
                                         : (m.has_width() ? m.width() : 2.0);
  Positive(width, "headland width");
  return width;
}
F2CCells HeadlandGenerator::GenerateHeadlands(
    const F2CCells& cells, const proto::HeadlandMode& m) const {
  const auto result = f2c::hg::ConstHL().GenerateHeadlands(cells, Width(m));
  if (result.IsEmpty() || result.Area() < 1e-6)
    throw RequestException("Headland removed the entire field");
  return result;
}
F2CCells HeadlandGenerator::GenerateHeadlandsBetweenCells(const F2CCells& cells,
                                                          double width) const {
  Positive(width, "route_headland_width");
  F2CCells result;
  for (size_t i = 0; i < cells.size(); ++i) {
    F2CCells part(cells.GetGeometry(i));
    const auto a_ring = cells.GetCellBorder(i);
    for (size_t k = 0; k < cells.size(); ++k) {
      if (i == k) continue;
      const auto b_ring = cells.GetCellBorder(k);
      for (size_t e = 0; e + 1 < a_ring.size(); ++e) {
        const auto a = a_ring[e], b = a_ring[e + 1], mid = (a + b) * 0.5;
        if (b_ring.ClosestPointTo(a).Distance(a) < 1e-3 &&
            b_ring.ClosestPointTo(b).Distance(b) < 1e-3 &&
            b_ring.ClosestPointTo(mid).Distance(mid) < 1e-3)
          part = part.difference(F2CCells::buffer(F2CLineString(a, b), width));
      }
    }
    for (const auto& p : part)
      if (p.Area() > 1e-3) result.AddGeometry(p);
  }
  if (result.IsEmpty())
    throw RequestException("Inter-cell corridors removed all swath cells");
  return result;
}
std::vector<F2CCells> HeadlandGenerator::GenerateHeadlandSwaths(
    const F2CCells& field, double width, const proto::HeadlandMode& m) const {
  Positive(width, "operation_width");
  const int count = std::max(1, static_cast<int>(std::round(Width(m) / width)));
  if (count > 1024) throw RequestException("Too many headland passes");
  return f2c::hg::ConstHL().GenerateHeadlandSwaths(field, width, count, true);
}

}  // namespace autonomy::task::coverage
