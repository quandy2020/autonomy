#include "autonomy/perception/tasks/scene_graph/room_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/imgproc.hpp>

namespace autonomy::perception::scene_graph {

std::optional<RoomGeometry> FitRoom(
    const std::vector<Point2D>& model_contour, const RoomMapTransform& transform) {
    const auto finite = [](double value) { return std::isfinite(value); };
    const auto& a = transform.model_to_grid;
    if (model_contour.size() < 3 ||
        !std::all_of(a.begin(), a.end(), finite) ||
        !finite(transform.metres_per_cell) || transform.metres_per_cell <= 0 ||
        !finite(transform.origin.x) || !finite(transform.origin.y) ||
        !finite(transform.yaw_rad))
        return std::nullopt;

    const auto anchor = model_contour.front();
    const double cos_yaw = std::cos(transform.yaw_rad);
    const double sin_yaw = std::sin(transform.yaw_rad);
    const auto to_map = [&](Point2D local) -> Point2D {
        return {transform.origin.x + cos_yaw * local.x - sin_yaw * local.y,
                transform.origin.y + sin_yaw * local.x + cos_yaw * local.y};
    };
    const double metres = transform.metres_per_cell;
    const Point2D offset{metres * (a[0] * anchor.x + a[1] * anchor.y + a[2]),
                         metres * (a[3] * anchor.x + a[4] * anchor.y + a[5])};
    std::vector<cv::Point2f> fit_points;
    fit_points.reserve(model_contour.size());
    RoomGeometry room{};
    room.contour.reserve(model_contour.size());
    for (const auto& point : model_contour) {
        // Fit relative to the anchor so large world origins do not lose
        // precision when OpenCV converts the coordinates to float.
        const double dx = point.x - anchor.x;
        const double dy = point.y - anchor.y;
        const double x = metres * (a[0] * dx + a[1] * dy);
        const double y = metres * (a[3] * dx + a[4] * dy);
        const auto world = to_map({offset.x + x, offset.y + y});
        if (!finite(world.x) || !finite(world.y) ||
            std::abs(x) > std::numeric_limits<float>::max() ||
            std::abs(y) > std::numeric_limits<float>::max())
            return std::nullopt;
        fit_points.emplace_back(static_cast<float>(x), static_cast<float>(y));
        room.contour.push_back(world);
    }

    const auto rect = cv::minAreaRect(fit_points);
    room.center = to_map({offset.x + rect.center.x, offset.y + rect.center.y});
    room.width_m = rect.size.width;
    room.height_m = rect.size.height;
    if (!finite(room.center.x) || !finite(room.center.y) ||
        !finite(room.width_m) || !finite(room.height_m) ||
        room.width_m <= 1e-6 || room.height_m <= 1e-6 || !finite(rect.angle))
        return std::nullopt;

    const double pi = std::acos(-1.0);
    double yaw = transform.yaw_rad + rect.angle * pi / 180.0;
    if (room.width_m < room.height_m) {
        std::swap(room.width_m, room.height_m);
        yaw += pi / 2;
    }
    room.yaw_rad = std::remainder(yaw, pi);
    return room;
}

}  // namespace autonomy::perception::scene_graph
