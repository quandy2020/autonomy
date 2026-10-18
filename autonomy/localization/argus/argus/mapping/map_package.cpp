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

#include "argus/mapping/map_package.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>

namespace argus {
namespace mapping {
namespace {

namespace fs = std::filesystem;

}  // namespace

std::uint64_t FileFnv64(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return 0;
    }
    std::uint64_t h = 14695981039346656037ull;
    constexpr std::uint64_t kPrime = 1099511628211ull;
    char buf[4096];
    while (in) {
        in.read(buf, sizeof(buf));
        const std::streamsize n = in.gcount();
        for (std::streamsize i = 0; i < n; ++i) {
            h ^= static_cast<std::uint8_t>(buf[i]);
            h *= kPrime;
        }
    }
    return h;
}

bool AtomicWriteFile(const std::string& path, const std::string& data) {
    const fs::path p(path);
    const fs::path tmp = p.string() + ".tmp";
    {
        std::ofstream os(tmp, std::ios::binary | std::ios::trunc);
        if (!os) {
            return false;
        }
        os.write(data.data(), static_cast<std::streamsize>(data.size()));
        if (!os) {
            return false;
        }
    }
    std::error_code ec;
    fs::rename(tmp, p, ec);
    if (ec) {
        fs::remove(tmp, ec);
        return false;
    }
    return true;
}

bool SaveMapPackageMeta(const std::string& path, const MapPackageMeta& meta) {
    std::ostringstream os;
    os << "argus_map_package " << meta.version << '\n';
    os << "session_id " << meta.session_id << '\n';
    os << "prior_resolution " << meta.prior_resolution << '\n';
    os << "ikd_downsample_resolution " << meta.ikd_downsample_resolution
       << '\n';
    os << "map_points " << meta.map_points << '\n';
    os << "visual_landmarks " << meta.visual_landmarks << '\n';
    os << "keyframes " << meta.keyframes << '\n';
    os << "map_xyz_fnv64 " << meta.map_xyz_fnv64 << '\n';
    return AtomicWriteFile(path, os.str());
}

bool LoadMapPackageMeta(const std::string& path, MapPackageMeta* meta) {
    if (meta == nullptr) {
        return false;
    }
    std::ifstream is(path);
    if (!is) {
        return false;
    }
    MapPackageMeta out;
    std::string tag;
    if (!(is >> tag >> out.version) || tag != "argus_map_package") {
        return false;
    }
    if (out.version < 1 || out.version > kMapPackageVersion) {
        return false;
    }
    std::string line;
    std::getline(is, line);
    while (std::getline(is, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        if (!(ss >> tag)) {
            continue;
        }
        if (tag == "session_id") {
            ss >> out.session_id;
        } else if (tag == "prior_resolution") {
            ss >> out.prior_resolution;
        } else if (tag == "ikd_downsample_resolution") {
            ss >> out.ikd_downsample_resolution;
        } else if (tag == "map_points") {
            ss >> out.map_points;
        } else if (tag == "visual_landmarks") {
            ss >> out.visual_landmarks;
        } else if (tag == "keyframes") {
            ss >> out.keyframes;
        } else if (tag == "map_xyz_fnv64") {
            ss >> out.map_xyz_fnv64;
        }
    }
    *meta = out;
    return true;
}

bool IsMapPackageDirectory(const std::string& directory) {
    std::error_code ec;
    const fs::path dir(directory);
    if (!fs::is_directory(dir, ec)) {
        return false;
    }
    if (fs::exists(dir / "map_meta.txt", ec)) {
        return true;
    }
    return fs::exists(dir / "session.txt", ec) &&
           fs::exists(dir / "map.xyz", ec);
}

bool VerifyMapPackage(const std::string& directory, const MapPackageMeta& meta,
                      std::string* error) {
    if (meta.map_xyz_fnv64 == 0) {
        return true;
    }
    const std::string map_path = (fs::path(directory) / "map.xyz").string();
    const std::uint64_t h = FileFnv64(map_path);
    if (h == 0) {
        if (error) {
            *error = "map.xyz unreadable";
        }
        return false;
    }
    if (h != meta.map_xyz_fnv64) {
        if (error) {
            *error = "map.xyz fingerprint mismatch";
        }
        return false;
    }
    return true;
}

}  // namespace mapping
}  // namespace argus
