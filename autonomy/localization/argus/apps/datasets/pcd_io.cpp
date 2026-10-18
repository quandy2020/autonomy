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

#include "apps/datasets/pcd_io.hpp"

#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace argus {
namespace datasets {
namespace {

struct FieldDesc {
  std::string name;
  int size = 4;
  char type = 'F';  // F/I/U
  int count = 1;
  int offset = 0;
};

int TypeSize(char type, int size) {
  (void)type;
  return size;
}

bool ParseHeader(std::ifstream& ifs, std::vector<FieldDesc>* fields,
                 std::size_t* points, std::string* data_mode,
                 std::size_t* header_bytes) {
  fields->clear();
  *points = 0;
  *data_mode = "ascii";
  std::vector<std::string> names;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  std::string line;
  std::size_t consumed = 0;
  while (std::getline(ifs, line)) {
    consumed += line.size() + 1;
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    std::string key;
    iss >> key;
    if (key == "FIELDS") {
      names.clear();
      std::string n;
      while (iss >> n) {
        names.push_back(n);
      }
    } else if (key == "SIZE") {
      sizes.clear();
      int v = 0;
      while (iss >> v) {
        sizes.push_back(v);
      }
    } else if (key == "TYPE") {
      types.clear();
      std::string t;
      while (iss >> t) {
        types.push_back(t.empty() ? 'F' : t[0]);
      }
    } else if (key == "COUNT") {
      counts.clear();
      int v = 0;
      while (iss >> v) {
        counts.push_back(v);
      }
    } else if (key == "POINTS" || key == "WIDTH") {
      std::size_t v = 0;
      iss >> v;
      if (key == "POINTS" || *points == 0) {
        *points = v;
      }
    } else if (key == "DATA") {
      iss >> *data_mode;
      *header_bytes = consumed;
      break;
    }
  }
  if (names.empty() || *points == 0) {
    return false;
  }
  if (sizes.size() < names.size()) {
    sizes.assign(names.size(), 4);
  }
  if (types.size() < names.size()) {
    types.assign(names.size(), 'F');
  }
  if (counts.size() < names.size()) {
    counts.assign(names.size(), 1);
  }
  int offset = 0;
  fields->resize(names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    (*fields)[i].name = names[i];
    (*fields)[i].size = sizes[i];
    (*fields)[i].type = types[i];
    (*fields)[i].count = counts[i];
    (*fields)[i].offset = offset;
    offset += TypeSize(types[i], sizes[i]) * counts[i];
  }
  return true;
}

float ReadFloatField(const char* base, const FieldDesc& f) {
  if (f.type == 'F' && f.size == 4) {
    float v = 0.f;
    std::memcpy(&v, base + f.offset, sizeof(float));
    return v;
  }
  if (f.type == 'F' && f.size == 8) {
    double v = 0.0;
    std::memcpy(&v, base + f.offset, sizeof(double));
    return static_cast<float>(v);
  }
  if ((f.type == 'U' || f.type == 'I') && f.size == 1) {
    return static_cast<float>(
        *reinterpret_cast<const std::uint8_t*>(base + f.offset));
  }
  if ((f.type == 'U' || f.type == 'I') && f.size == 2) {
    std::uint16_t v = 0;
    std::memcpy(&v, base + f.offset, sizeof(v));
    return static_cast<float>(v);
  }
  if ((f.type == 'U' || f.type == 'I') && f.size == 4) {
    std::uint32_t v = 0;
    std::memcpy(&v, base + f.offset, sizeof(v));
    return static_cast<float>(v);
  }
  return 0.f;
}

const FieldDesc* FindField(const std::vector<FieldDesc>& fields,
                           const char* name) {
  for (const auto& f : fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

}  // namespace

sensor::PointCloudXYZI::Ptr LoadPcdXYZI(const std::string& path) {
  auto cloud = std::make_shared<sensor::PointCloudXYZI>();
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    return cloud;
  }
  std::vector<FieldDesc> fields;
  std::size_t npts = 0;
  std::string data_mode;
  std::size_t header_bytes = 0;
  if (!ParseHeader(ifs, &fields, &npts, &data_mode, &header_bytes)) {
    return cloud;
  }
  const FieldDesc* fx = FindField(fields, "x");
  const FieldDesc* fy = FindField(fields, "y");
  const FieldDesc* fz = FindField(fields, "z");
  if (fx == nullptr || fy == nullptr || fz == nullptr) {
    return cloud;
  }
  const FieldDesc* fi = FindField(fields, "intensity");
  if (fi == nullptr) {
    fi = FindField(fields, "reflectivity");
  }

  int point_step = 0;
  for (const auto& f : fields) {
    point_step += TypeSize(f.type, f.size) * f.count;
  }
  if (point_step <= 0 || npts == 0) {
    return cloud;
  }

  cloud->reserve(npts);
  for (char& c : data_mode) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }

  if (data_mode == "ascii") {
    std::string line;
    std::size_t i = 0;
    while (i < npts && std::getline(ifs, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }
      std::istringstream iss(line);
      std::vector<double> vals;
      double v = 0;
      while (iss >> v) {
        vals.push_back(v);
      }
      if (vals.size() < 3) {
        continue;
      }
      sensor::PointType p;
      p.x = static_cast<float>(vals[0]);
      p.y = static_cast<float>(vals[1]);
      p.z = static_cast<float>(vals[2]);
      p.intensity = vals.size() > 3 ? static_cast<float>(vals[3]) : 0.f;
      p.curvature = static_cast<float>(
          (static_cast<double>(i) / static_cast<double>(npts)) * 100.0);
      cloud->push_back(p);
      ++i;
    }
    return cloud;
  }

  if (data_mode == "binary_compressed") {
    // LZF-compressed PCD not supported in apps/datasets.
    return cloud;
  }

  // binary
  std::vector<char> buf(static_cast<std::size_t>(point_step) * npts);
  ifs.read(buf.data(), static_cast<std::streamsize>(buf.size()));
  if (static_cast<std::size_t>(ifs.gcount()) < buf.size()) {
    return cloud;
  }
  for (std::size_t i = 0; i < npts; ++i) {
    const char* base = buf.data() + i * static_cast<std::size_t>(point_step);
    sensor::PointType p;
    p.x = ReadFloatField(base, *fx);
    p.y = ReadFloatField(base, *fy);
    p.z = ReadFloatField(base, *fz);
    p.intensity = fi != nullptr ? ReadFloatField(base, *fi) : 0.f;
    p.curvature = static_cast<float>(
        (static_cast<double>(i) / static_cast<double>(npts)) * 100.0);
    cloud->push_back(p);
  }
  return cloud;
}

}  // namespace datasets
}  // namespace argus
