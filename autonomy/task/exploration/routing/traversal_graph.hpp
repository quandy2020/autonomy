/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/integration/path_provider.hpp"
namespace autonomy::task::exploration {
struct TraversalVertex {std::uint64_t id=0;Pose2d pose;double stamp=0;};
class TraversalGraph {
public:
    bool Record(const Pose2d& pose,double stamp,const MapReadView& map,const PathProvider& paths);
    void Reset(std::uint64_t epoch){epoch_=epoch;vertices_.clear();edges_.clear();}
    const std::vector<TraversalVertex>& vertices() const{return vertices_;}
    const std::map<std::pair<std::uint64_t,std::uint64_t>,double>& edges() const{return edges_;}
private:
    std::uint64_t epoch_=0,next_id_=0;
    std::vector<TraversalVertex> vertices_;
    std::map<std::pair<std::uint64_t,std::uint64_t>,double> edges_;
};
}  // namespace autonomy::task::exploration
