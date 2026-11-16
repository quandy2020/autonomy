#ifndef AUTONOMY_PERCEPTION_TASKS_SCENE_GRAPH_ROOM_ATTRIBUTION_HPP_
#define AUTONOMY_PERCEPTION_TASKS_SCENE_GRAPH_ROOM_ATTRIBUTION_HPP_

#include <string>
#include "autonomy/perception/tasks/scene_graph/room_geometry.hpp"

namespace autonomy::perception::scene_graph {

struct RoomAnnotation {
    std::string id;
    std::vector<Point2D> contour;
};

// The caller transforms both observations into the same map revision.
// A point on a boundary, an overlap, or outside all rooms has no assignment.
std::optional<std::string> FindRoom(
    Point2D map_center, const std::vector<RoomAnnotation>& rooms);

}  // namespace autonomy::perception::scene_graph

#endif
