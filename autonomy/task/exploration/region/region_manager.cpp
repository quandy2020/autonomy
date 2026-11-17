/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/region/region_manager.hpp"
#include <stdexcept>
namespace autonomy::task::exploration {
RegionManager::RegionManager(double size):region_size_(size){if(!std::isfinite(size)||size<=0)throw std::invalid_argument("invalid region size");}
std::pair<int,int> RegionManager::RegionFor(const Pose2d& p) const{return {static_cast<int>(std::floor(p.x/region_size_)),static_cast<int>(std::floor(p.y/region_size_))};}
void RegionManager::Update(const CoverageState& coverage){regions_.clear();
    for(const auto& unit:coverage.units()){if(!unit.second.pending)continue;
        const auto id=RegionFor({unit.second.center.x,unit.second.center.y,0});auto& region=regions_[id];region.id=id;
        region.pending_units.push_back(unit.first);region.remaining_area+=unit.second.area*coverage.Remaining(unit.first);
    }
}
}  // namespace autonomy::task::exploration
