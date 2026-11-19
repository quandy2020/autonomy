#ifndef OPT_NAVIGATION_GRID_MAP_H_
#define OPT_NAVIGATION_GRID_MAP_H_

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "opt/navigation/types.h"

namespace opt::navigation {

enum class CellState { kUnknown, kFree, kOccupied };
struct GridCell {
  CellState state = CellState::kUnknown;
  double floor_height = 0.0;
  // +infinity means explicitly known unbounded overhead space, not missing data.
  double ceiling_height = std::numeric_limits<double>::infinity();
};
struct CellIndex { int x = 0; int y = 0; };

/** @brief Bounded start-only support inferred from a measured surrounding ring.
 * The region may close only unknown cells; observed occupied or conflicting
 * surfaces are never overridden. It is never serialized into the map.
 */
struct StartupSupportRegion {
  double center_x = 0.0;  ///< Startup pose X in the map frame, metres.
  double center_y = 0.0;  ///< Startup pose Y in the map frame, metres.
  double radius = 0.0;  ///< Maximum closed-cell radius around the startup pose, metres.
  double floor_height = 0.0;  ///< Measured surrounding-ring floor estimate, metres.
  double ceiling_height = 0.0;  ///< Measured surrounding-ring ceiling estimate, metres.
  /** @brief Test one world-coordinate position against the closed startup disc. */
  bool Contains(double x, double y) const {
    return std::hypot(x - center_x, y - center_y) <= radius + 1e-12;
  }
};

class GridMap {
 public:
  GridMap(int width, int height, double resolution, double origin_x = 0.0,
          double origin_y = 0.0, std::string frame_id = "map");
  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }
  const std::string& frame_id() const { return frame_id_; }
  std::size_t size() const { return cells_.size(); }
  bool Contains(int x, int y) const;
  std::size_t Index(int x, int y) const;
  CellIndex FromIndex(std::size_t index) const;
  std::optional<CellIndex> WorldToCell(const Pose2d& pose) const;
  Pose2d CellCenter(int x, int y) const;
  const GridCell& At(int x, int y) const;
  void Set(int x, int y, const GridCell& cell);
  void Fill(const GridCell& cell);

 private:
  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::string frame_id_;
  std::vector<GridCell> cells_;
};

}  // namespace opt::navigation
#endif  // OPT_NAVIGATION_GRID_MAP_H_
