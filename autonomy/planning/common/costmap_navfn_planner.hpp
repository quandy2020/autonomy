/*
 * Copyright 2026 The Openbot Authors
 *
 * NavFn-based grid planning on a Costmap2D (shared by exploration and planners).
 */

#pragma once

#include <functional>
#include <vector>

#include <automsgs/msgs/geometry_msgs/point.pb.h>

namespace autonomy::map::costmap_2d {
class Costmap2D;
}

namespace autonomy::planning::common {

struct CostmapNavfnPlannerOptions {
  bool allow_unknown = true;
  bool use_astar = true;
  double tolerance_m = 0.0;
};

class CostmapNavfnPlanner {
 public:
  // Returns path length in meters, or +inf if no path. Fills world-frame points.
  static double Plan(const map::costmap_2d::Costmap2D& costmap, double from_x,
                     double from_y, double to_x, double to_y,
                     std::vector<automsgs::msgs::geometry_msgs::Point>* path,
                     const CostmapNavfnPlannerOptions& options =
                         CostmapNavfnPlannerOptions(),
                     std::function<bool()> cancel_checker = {});
};

}  // namespace autonomy::planning::common
