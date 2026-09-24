/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy/localization/atlas/map/dense/tiled_cloud.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

constexpr char kMagic[4] = {'A', 'T', 'L', '1'};

struct TileKey {
    int x = 0;
    int y = 0;
    bool operator==(const TileKey& other) const {
        return x == other.x && y == other.y;
    }
};

struct TileHash {
    std::size_t operator()(const TileKey& key) const {
        return (static_cast<std::size_t>(key.x) * 73856093u) ^
               (static_cast<std::size_t>(key.y) * 19349663u);
    }
};

bool WriteTile(const std::filesystem::path& path, const PointCloud& cloud) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }
    const auto count = static_cast<uint32_t>(cloud.size());
    out.write(kMagic, 4);
    out.write(reinterpret_cast<const char*>(&count), sizeof(count));
    for (const auto& point : cloud) {
        const float xyz[3] = {point.x, point.y, point.z};
        out.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
    }
    return static_cast<bool>(out);
}

bool ReadTile(const std::filesystem::path& path, PointCloud* cloud) {
    std::ifstream in(path, std::ios::binary);
    if (!in || cloud == nullptr) {
        return false;
    }
    char magic[4] = {};
    uint32_t count = 0;
    in.read(magic, 4);
    in.read(reinterpret_cast<char*>(&count), sizeof(count));
    if (!in || std::memcmp(magic, kMagic, 4) != 0) {
        return false;
    }
    for (uint32_t i = 0; i < count; ++i) {
        float xyz[3] = {};
        in.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
        if (!in) {
            return false;
        }
        PointXYZI point;
        point.x = xyz[0];
        point.y = xyz[1];
        point.z = xyz[2];
        cloud->push_back(point);
    }
    return true;
}

}  // namespace

bool SaveTiledCloud(const std::string& directory, const PointCloud& cloud,
                    double chunk_size) {
    if (directory.empty() || cloud.empty()) {
        return false;
    }
    const double chunk = chunk_size > 1.0 ? chunk_size : 20.0;
    std::unordered_map<TileKey, PointCloud, TileHash> tiles;
    for (const auto& point : cloud) {
        TileKey key;
        key.x = static_cast<int>(std::floor(point.x / chunk));
        key.y = static_cast<int>(std::floor(point.y / chunk));
        tiles[key].push_back(point);
    }
    std::error_code error;
    const auto root = std::filesystem::path(directory) / "tiles";
    std::filesystem::create_directories(root, error);
    if (error) {
        return false;
    }
    for (const auto& tile : tiles) {
        const auto path =
            root / (std::to_string(tile.first.x) + "_" +
                    std::to_string(tile.first.y) + ".tile");
        if (!WriteTile(path, tile.second)) {
            return false;
        }
    }
    return true;
}

bool LoadTiledCloud(const std::string& directory, PointCloud* cloud) {
    if (directory.empty() || cloud == nullptr) {
        return false;
    }
    cloud->clear();
    const auto root = std::filesystem::path(directory) / "tiles";
    std::error_code error;
    if (!std::filesystem::is_directory(root, error)) {
        return false;
    }
    for (const auto& entry : std::filesystem::directory_iterator(root, error)) {
        if (error || !entry.is_regular_file() ||
            entry.path().extension() != ".tile") {
            continue;
        }
        if (!ReadTile(entry.path(), cloud)) {
            return false;
        }
    }
    return !cloud->empty();
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
