/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/visualization/converter/map_msgs_converter.hpp"

#include "autolink/autolink.hpp"
#include "autonomy/visualization/converter/converter_detail.hpp"

namespace autonomy {
namespace visualization {
namespace converter {
namespace impl {

using detail::ExtractFrameId;
using detail::ExtractTimestamp;
using detail::SetGridHeader;

foxglove::schemas::Grid ToFoxgloveImpl(
    const autonomy::commsgs::proto::map_msgs::OccupancyGrid& message) {
    foxglove::schemas::Grid grid;

    if (!message.has_info()) {
        AERROR << "OccupancyGrid missing info field";
        return grid;
    }

    const auto& info = message.info();

    SetGridHeader(grid, ExtractTimestamp(message), ExtractFrameId(message));

    // 设置网格原点
    if (info.has_origin()) {
        grid.pose = foxglove::schemas::Pose();
        grid.pose->position = foxglove::schemas::Vector3();
        grid.pose->position->x = info.origin().position().x();
        grid.pose->position->y = info.origin().position().y();
        grid.pose->position->z = info.origin().position().z();

        grid.pose->orientation = foxglove::schemas::Quaternion();
        grid.pose->orientation->x = info.origin().orientation().x();
        grid.pose->orientation->y = info.origin().orientation().y();
        grid.pose->orientation->z = info.origin().orientation().z();
        grid.pose->orientation->w = info.origin().orientation().w();
    }

    grid.column_count = info.width();

    if (info.resolution() > 0.0f) {
        grid.cell_size = foxglove::schemas::Vector2();
        grid.cell_size->x = info.resolution();
        grid.cell_size->y = info.resolution();
    }

    // 设置数据字段 - RGBA
    foxglove::schemas::PackedElementField red_field;
    red_field.name = "red";
    red_field.offset = 0;
    red_field.type = foxglove::schemas::PackedElementField::NumericType::UINT8;
    grid.fields.push_back(red_field);

    foxglove::schemas::PackedElementField green_field;
    green_field.name = "green";
    green_field.offset = 1;
    green_field.type =
        foxglove::schemas::PackedElementField::NumericType::UINT8;
    grid.fields.push_back(green_field);

    foxglove::schemas::PackedElementField blue_field;
    blue_field.name = "blue";
    blue_field.offset = 2;
    blue_field.type = foxglove::schemas::PackedElementField::NumericType::UINT8;
    grid.fields.push_back(blue_field);

    foxglove::schemas::PackedElementField alpha_field;
    alpha_field.name = "alpha";
    alpha_field.offset = 3;
    alpha_field.type =
        foxglove::schemas::PackedElementField::NumericType::UINT8;
    grid.fields.push_back(alpha_field);

    grid.cell_stride = 4;
    grid.row_stride = grid.column_count * grid.cell_stride;

    // 转换数据：int32 -> RGBA
    uint32_t expected_size = info.width() * info.height();
    if (message.data_size() != static_cast<int>(expected_size)) {
        AWARN << "OccupancyGrid data size mismatch: expected " << expected_size
              << ", got " << message.data_size();
    }

    grid.data.reserve(expected_size * grid.cell_stride);
    for (int i = 0;
         i < message.data_size() && i < static_cast<int>(expected_size); ++i) {
        int32_t occupancy_value = message.data(i);
        uint8_t r, g, b, a;

        if (occupancy_value == -1) {
            r = 128;
            g = 128;
            b = 128;
            a = 255;
        } else if (occupancy_value == 0) {
            r = 255;
            g = 255;
            b = 255;
            a = 255;
        } else if (occupancy_value == 100) {
            r = 0;
            g = 0;
            b = 0;
            a = 255;
        } else if (occupancy_value > 0 && occupancy_value < 100) {
            uint8_t gray =
                static_cast<uint8_t>(255 - (occupancy_value * 255 / 100));
            r = gray;
            g = gray;
            b = gray;
            a = 255;
        } else {
            r = 255;
            g = 0;
            b = 0;
            a = 255;
        }

        grid.data.push_back(static_cast<std::byte>(r));
        grid.data.push_back(static_cast<std::byte>(g));
        grid.data.push_back(static_cast<std::byte>(b));
        grid.data.push_back(static_cast<std::byte>(a));
    }

    AINFO << "Converted OccupancyGrid to Grid: " << info.width() << "x"
          << info.height() << " cells, resolution=" << info.resolution() << "m";

    return grid;
}

}  // namespace impl
}  // namespace converter
}  // namespace visualization
}  // namespace autonomy
