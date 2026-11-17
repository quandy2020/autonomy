#ifndef OPT_NAVIGATION_SAFETY_H_
#define OPT_NAVIGATION_SAFETY_H_

#include <memory>
#include <string>
#include <vector>

#include "opt/navigation/grid_map.h"

namespace opt::navigation {

struct SafetyOptions {
  double robot_radius = 0.20;
  double horizontal_margin = 0.05;
  double robot_height = 0.60;
  double vertical_margin = 0.05;
  double max_floor_step = 0.08;
};
struct SafetyResult {
  bool safe = false;
  double clearance = 0.0;  // Lower bound on center-to-occupied/unknown/boundary distance.
  std::string reason;
};

// Conservative 2.5D checker. Uses a linear-time 2D Euclidean distance transform
// and checks all grid cells overlapped by a circumscribed circular footprint.
// It does not claim a 3D ESDF or certify unseen/dynamic obstacles.
class GeometrySafety {
 public:
  GeometrySafety(std::shared_ptr<const GridMap> map, SafetyOptions options = {});
  const GridMap& map() const { return *map_; }
  const SafetyOptions& options() const { return options_; }
  SafetyResult VerifyPose(const Pose2d& pose, double extra_padding = 0.0) const;
  SafetyResult VerifySegment(const Pose2d& from, const Pose2d& to) const;
  SafetyResult VerifyPath(const Path& path) const;
  double Clearance(const Pose2d& pose) const;

 private:
  void BuildDistanceField();
  std::shared_ptr<const GridMap> map_;
  SafetyOptions options_;
  std::vector<double> distances_;
};

}  // namespace opt::navigation
#endif  // OPT_NAVIGATION_SAFETY_H_
