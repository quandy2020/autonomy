/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/frontier/frontier_extractor.hpp"
#include <set>
namespace autonomy::task::exploration {
std::vector<CoverageUnit> FrontierExtractor::Extract(const MapReadView& view,double height,
    const std::optional<Bounds2d>& allowed_region) const {
    std::vector<CoverageUnit> result;if(!view.grid || !std::isfinite(height))return result;
    const auto& map=*view.grid;std::set<std::pair<int,int>> cells;
    for(int y=0;y<map.height();++y)for(int x=0;x<map.width();++x) {
        if(map.At(x,y).state!=autonomy::navigation::CellState::kFree)continue;
        for(auto offset:{std::make_pair(1,0),std::make_pair(-1,0),std::make_pair(0,1),std::make_pair(0,-1)}) {
            int nx=x+offset.first,ny=y+offset.second;
            if(!map.Contains(nx,ny) || map.At(nx,ny).state==autonomy::navigation::CellState::kUnknown)
                cells.insert({nx,ny});
        }
    }
    for(const auto& cell:cells) {
        Pose2d center{map.origin_x()+(cell.first+.5)*map.resolution(),
                      map.origin_y()+(cell.second+.5)*map.resolution(),0};
        if(allowed_region && !allowed_region->Contains(center))continue;
        CoverageUnit unit;
        // Epoch owns the quantization lattice; IDs survive rolling origin changes.
        unit.id={static_cast<int>(std::floor(center.x/map.resolution())),
                 static_cast<int>(std::floor(center.y/map.resolution())),0,0,CoverageKind::kFrontier};
        unit.center={center.x,center.y,height};unit.area=map.resolution()*map.resolution();
        unit.tolerance=map.resolution()/std::sqrt(2.0);result.push_back(unit);
    }
    return result;
}
}  // namespace autonomy::task::exploration
