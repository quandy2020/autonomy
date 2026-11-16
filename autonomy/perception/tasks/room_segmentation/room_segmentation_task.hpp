#ifndef AUTONOMY_PERCEPTION_TASKS_ROOM_SEGMENTATION_TASK_HPP_
#define AUTONOMY_PERCEPTION_TASKS_ROOM_SEGMENTATION_TASK_HPP_

#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>

#include "autonomy/common/network/common/tensor.hpp"
#include "autonomy/perception/tasks/scene_graph/room_geometry.hpp"

namespace autonomy::perception::room_segmentation {

// Native-size float32 occupancy input. Source row order is preserved,
// known cells map to [0,1], and unknown cells map to -1.
bool PrepareRoomGrid(
    const automsgs::msgs::map_msgs::OccupancyGrid& grid,
    const common::network::ModelTensorInfo& input,
    common::network::Tensor* tensor, scene_graph::RoomMapTransform* transform,
    std::string* error);

// Export contract: float32 vertices [sum(vertex_counts),2], int32
// vertex_counts [R]. Vertices must be in the model's map-image coordinates.
bool DecodeRooms(const common::network::Tensor& vertices,
                 const common::network::Tensor& vertex_counts,
                 const scene_graph::RoomMapTransform& transform,
                 std::vector<scene_graph::RoomGeometry>* rooms,
                 std::string* error);

}  // namespace autonomy::perception::room_segmentation

#endif
