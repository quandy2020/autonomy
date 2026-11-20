/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/routing/traversal_graph.hpp"
namespace autonomy::task::exploration {
bool TraversalGraph::Record(const Pose2d& pose,double stamp,const MapReadView& map,const PathProvider& paths){
    if(!autonomy::navigation::IsFinite(pose) || !std::isfinite(stamp))return false;
    if(epoch_!=map.map_epoch)Reset(map.map_epoch);
    if(!vertices_.empty() && (stamp<vertices_.back().stamp || autonomy::navigation::Distance(pose,vertices_.back().pose)<.3))return false;
    TraversalVertex v{++next_id_,pose,stamp};
    if(!vertices_.empty()) {
        auto route=paths.Route(vertices_.back().pose,pose);
        if(!route.success || route.map_epoch!=epoch_)return false;
        edges_[{vertices_.back().id,v.id}]=route.length;
    }
    vertices_.push_back(v);return true;
}
}  // namespace autonomy::task::exploration
