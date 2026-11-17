/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/viewpoint/viewpoint.hpp"
namespace autonomy::task::exploration {
struct ExplorationRegion {
    std::pair<int,int> id;
    std::vector<CoverageId> pending_units;
    double remaining_area=0;
};
class RegionManager {
public:
    explicit RegionManager(double region_size=4.0);
    void Update(const CoverageState& coverage);
    std::pair<int,int> RegionFor(const Pose2d& pose) const;
    const std::map<std::pair<int,int>,ExplorationRegion>& regions() const{return regions_;}
private:
    double region_size_;
    std::map<std::pair<int,int>,ExplorationRegion> regions_;
};
}  // namespace autonomy::task::exploration
