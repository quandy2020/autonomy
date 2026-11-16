#include "autonomy/perception/tasks/room_segmentation/room_segmentation_task.hpp"

#include <cmath>
#include <cstring>
#include <utility>

namespace autonomy::perception::room_segmentation {

bool PrepareRoomGrid(
    const automsgs::msgs::map_msgs::OccupancyGrid& grid,
    const common::network::ModelTensorInfo& input,
    common::network::Tensor* tensor, scene_graph::RoomMapTransform* transform,
    std::string* error) {
    auto fail = [&](const char* detail) {
        if (error)
            *error = detail;
        return false;
    };
    const auto& info = grid.info();
    const auto& pose = info.origin();
    const auto& q = pose.orientation();
    const double norm = std::hypot(q.z(), q.w());
    if (!tensor || !transform || grid.header().frame_id().empty() ||
        info.width() == 0 || info.height() == 0 ||
        static_cast<uint64_t>(info.width()) * info.height() !=
            static_cast<uint64_t>(grid.data_size()) ||
        !std::isfinite(info.resolution()) || info.resolution() <= 0 ||
        !std::isfinite(pose.position().x()) || !std::isfinite(pose.position().y()) ||
        !std::isfinite(norm) || std::abs(norm - 1) > 1e-3 ||
        !(std::abs(q.x()) <= 1e-6 && std::abs(q.y()) <= 1e-6)) {
        return fail("Invalid occupancy grid layout, frame or planar origin.");
    }
    std::vector<int64_t> shape;
    if (input.element_type != common::network::ElementType::kFloat32 ||
        !common::network::ResolveShapeForElementCount(
            input, static_cast<size_t>(grid.data_size()), &shape, error) ||
        shape != std::vector<int64_t>{1, 1, info.height(), info.width()}) {
        return fail("RRCE grid input must match float32 [1,1,height,width].");
    }

    std::vector<float> cells;
    cells.reserve(grid.data_size());
    for (const auto value : grid.data()) {
        if (value < -1 || value > 100)
            return fail("Occupancy values must be -1 or in [0,100].");
        cells.push_back(value < 0 ? -1.0F : value / 100.0F);
    }
    *tensor = common::network::Tensor::FromFloat32(std::move(cells));
    // Model integer indices denote cell centres. The map pose is the corner.
    *transform = {{1, 0, 0.5, 0, 1, 0.5}, info.resolution(),
                  {pose.position().x(), pose.position().y()},
                  2 * std::atan2(q.z(), q.w())};
    return true;
}

bool DecodeRooms(const common::network::Tensor& vertices,
                 const common::network::Tensor& vertex_counts,
                 const scene_graph::RoomMapTransform& transform,
                 std::vector<scene_graph::RoomGeometry>* rooms,
                 std::string* error) {
    auto fail = [&](const char* detail) {
        if (error)
            *error = detail;
        return false;
    };
    const float* coordinates = nullptr;
    size_t count = 0;
    if (!rooms || vertex_counts.element_type() != common::network::ElementType::kInt32 ||
        vertex_counts.byte_size() % sizeof(int32_t) ||
        !vertices.TryViewFloat32(&coordinates, &count, error) || count % 2 ||
        (count && !coordinates))
        return fail("Expected packed float32 coordinate pairs and int32 vertex counts.");

    std::vector<scene_graph::RoomGeometry> decoded;
    size_t cursor = 0;
    const size_t total = count / 2;
    for (size_t i = 0; i < vertex_counts.element_count(); ++i) {
        int32_t length;
        std::memcpy(&length, vertex_counts.bytes() + i * sizeof(length), sizeof(length));
        if (length < 3 || static_cast<size_t>(length) > total - cursor)
            return fail("Invalid room vertex count.");

        std::vector<scene_graph::Point2D> polygon;
        polygon.reserve(length);
        for (int32_t j = 0; j < length; ++j, ++cursor)
            polygon.push_back({coordinates[cursor * 2], coordinates[cursor * 2 + 1]});
        auto room = scene_graph::FitRoom(polygon, transform);
        if (!room)
            return fail("Invalid room contour or map transform.");
        decoded.push_back(std::move(*room));
    }
    if (cursor != total)
        return fail("Room counts do not cover the vertices tensor.");
    *rooms = std::move(decoded);
    return true;
}

}  // namespace autonomy::perception::room_segmentation
