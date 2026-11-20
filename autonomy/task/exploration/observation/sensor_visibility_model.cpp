/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/observation/sensor_visibility_model.hpp"
#include <algorithm>

namespace autonomy::task::exploration {
perception::RigidTransform WorldFromSensor(const Pose2d& base,const perception::RigidTransform& sensor) {
    perception::RigidTransform result;
    const double c=std::cos(base.yaw),s=std::sin(base.yaw);
    for(int k=0;k<3;++k) {
        result.rotation[k]=c*sensor.rotation[k]-s*sensor.rotation[3+k];
        result.rotation[3+k]=s*sensor.rotation[k]+c*sensor.rotation[3+k];
        result.rotation[6+k]=sensor.rotation[6+k];
    }
    result.translation={base.x+c*sensor.translation.x-s*sensor.translation.y,
                        base.y+s*sensor.translation.x+c*sensor.translation.y,sensor.translation.z};
    return result;
}
bool RayVisible(const perception::Vec3& origin,const CoverageUnit& unit,const MapReadView& view) {
    if(!view.grid || !origin.Finite())return false;
    const auto& map=*view.grid;
    auto a=map.WorldToCell({origin.x,origin.y,0}),b=map.WorldToCell({unit.center.x,unit.center.y,0});
    if(!a)return false;
    if(!b && unit.id.kind==CoverageKind::kFrontier) {
        const int x=static_cast<int>(std::floor((unit.center.x-map.origin_x())/map.resolution()));
        const int y=static_cast<int>(std::floor((unit.center.y-map.origin_y())/map.resolution()));
        if(x>=-1 && x<=map.width() && y>=-1 && y<=map.height())
            b=autonomy::navigation::CellIndex{x,y};
    }
    if(!b)return false;
    const double dx=unit.center.x-origin.x,dy=unit.center.y-origin.y;
    const int sx=dx>0?1:dx<0?-1:0,sy=dy>0?1:dy<0?-1:0;
    const double inf=std::numeric_limits<double>::infinity();
    const double tx=sx?map.resolution()/std::abs(dx):inf,ty=sy?map.resolution()/std::abs(dy):inf;
    double nx=sx?(map.origin_x()+(a->x+(sx>0?1:0))*map.resolution()-origin.x)/dx:inf;
    double ny=sy?(map.origin_y()+(a->y+(sy>0?1:0))*map.resolution()-origin.y)/dy:inf;
    int x=a->x,y=a->y;
    for(std::size_t count=0;count<map.size()+1;++count) {
        if(!map.Contains(x,y))return x==b->x && y==b->y && unit.id.kind==CoverageKind::kFrontier;
        const auto state=map.At(x,y).state;
        if(x==b->x && y==b->y)
            return state==autonomy::navigation::CellState::kFree ||
                   (state==autonomy::navigation::CellState::kUnknown && unit.id.kind==CoverageKind::kFrontier) ||
                   (state==autonomy::navigation::CellState::kOccupied && unit.id.kind!=CoverageKind::kFrontier);
        if(state!=autonomy::navigation::CellState::kFree)return false;
        if(std::abs(nx-ny)<1e-12) {
            // Conservative supercover at a grid corner; do not see through thin diagonal walls.
            for(auto neighbor:{std::make_pair(x+sx,y),std::make_pair(x,y+sy)})
                if(!map.Contains(neighbor.first,neighbor.second) ||
                   map.At(neighbor.first,neighbor.second).state!=autonomy::navigation::CellState::kFree)return false;
            x+=sx;y+=sy;nx+=tx;ny+=ty;
        } else if(nx<ny) {x+=sx;nx+=tx;} else {y+=sy;ny+=ty;}
    }
    return false;
}
double QualityWeight(const CoverageUnit& unit,const perception::Vec3& origin,double range,
                      double maximum_range,const ObservationQuality& q) {
    for(double weight:{q.valid_fraction,q.localization_weight,q.motion_weight,q.history_weight})
        if(!std::isfinite(weight) || weight<0 || weight>1)return 0;
    if(!std::isfinite(range) || range<=0 || range>maximum_range)return 0;
    auto direction=(unit.center-origin)*(1.0/range);
    double incidence=1,normal=unit.normal.Norm();
    if(normal>1e-6)incidence=std::max(0.0,-(unit.normal.x*direction.x+unit.normal.y*direction.y+
                                         unit.normal.z*direction.z)/normal);
    const double distance_weight=1.0/(1.0+range*range/(maximum_range*maximum_range));
    return distance_weight*incidence*q.valid_fraction*q.localization_weight*q.motion_weight*q.history_weight;
}
}  // namespace autonomy::task::exploration
