#ifndef AUTONOMY_PERCEPTION_TASKS_SCENE_GRAPH_ROOM_GEOMETRY_HPP_
#define AUTONOMY_PERCEPTION_TASKS_SCENE_GRAPH_ROOM_GEOMETRY_HPP_

#include <array>
#include <optional>
#include <vector>

namespace autonomy::perception::scene_graph {

struct Point2D {
    double x;
    double y;
};

// Pixel coordinates are mapped to source-grid coordinates by this affine
// transform. A coordinate at (0, 0) in the grid denotes the origin corner.
struct RoomMapTransform {
    std::array<double, 6> model_to_grid;
    double metres_per_cell;
    Point2D origin;
    double yaw_rad;
};

struct RoomGeometry {
    std::vector<Point2D> contour;
    Point2D center;
    double width_m;
    double height_m;
    double yaw_rad;
};

// Compute a map-frame contour and its minimum-area enclosing rectangle.
// Returns nullopt for invalid transforms and degenerate contours.
std::optional<RoomGeometry> FitRoom(
    const std::vector<Point2D>& model_contour, const RoomMapTransform& transform);

}  // namespace autonomy::perception::scene_graph

#endif
