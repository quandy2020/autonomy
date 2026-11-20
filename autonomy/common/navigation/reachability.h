#ifndef OPT_NAVIGATION_REACHABILITY_H_
#define OPT_NAVIGATION_REACHABILITY_H_

#include <memory>
#include <functional>
#include <vector>

#include "opt/navigation/safety.h"

namespace opt::navigation {

// A single four-connected Dijkstra field serves every candidate. Edges and
// endpoint connections are checked with the same swept-footprint safety rules.
class ReachabilityField {
 public:
  ReachabilityField(std::shared_ptr<const GeometrySafety> safety, Pose2d start,
                    std::size_t max_expansions = 0,
                    std::function<bool()> cancel_checker = {});
  double DistanceTo(const Pose2d& target) const;
  Path PathTo(const Pose2d& target) const;
  bool budget_exhausted() const { return budget_exhausted_; }
  std::size_t expansions() const { return expansions_; }
  bool canceled() const { return canceled_; }

 private:
  std::shared_ptr<const GeometrySafety> safety_;
  Pose2d start_;
  std::vector<double> distance_;
  std::vector<std::size_t> parent_;
  std::vector<bool> settled_;
  bool budget_exhausted_ = false;
  bool canceled_ = false;
  std::size_t expansions_ = 0;
};

class IPathPlanner {
 public:
  virtual ~IPathPlanner() = default;
  virtual PlanningResult Plan(const PlanningRequest& request) const = 0;
};
class GridPlanner final : public IPathPlanner {
 public:
  explicit GridPlanner(SafetyOptions options = {}) : options_(options) {}
  PlanningResult Plan(const PlanningRequest& request) const override;

 private:
  SafetyOptions options_;
};

}  // namespace opt::navigation
#endif  // OPT_NAVIGATION_REACHABILITY_H_
