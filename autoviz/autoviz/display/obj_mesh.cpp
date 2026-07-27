/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/obj_mesh.hpp"

#include <QFile>
#include <QFileInfo>
#include <QStringList>
#include <QVector2D>

#include <algorithm>
#include <cstring>
#include <limits>
#include <unordered_map>

#include <automsgs/msgs/visualization_msgs/marker.pb.h>

namespace autoviz {
namespace display {
namespace {

int ParseFaceIndex(const QString& token, int part_index = 0) {
  const QStringList parts = token.split('/');
  if (part_index >= parts.size() || parts[part_index].isEmpty()) {
    return 0;
  }
  return parts[part_index].toInt();
}

}  // namespace

bool parseObjText(const std::string& text, ObjMesh* mesh) {
  if (mesh == nullptr) {
    return false;
  }
  mesh->vertices.clear();
  mesh->texcoords.clear();
  mesh->triangles.clear();

  std::vector<QVector3D> raw_vertices;
  std::vector<QVector2D> raw_texcoords;
  struct CornerKey {
    int vi = 0;
    int ti = -1;
    bool operator==(const CornerKey& other) const {
      return vi == other.vi && ti == other.ti;
    }
  };
  struct CornerKeyHash {
    std::size_t operator()(const CornerKey& key) const {
      return static_cast<std::size_t>(key.vi) ^
             (static_cast<std::size_t>(key.ti + 1) << 16);
    }
  };
  std::unordered_map<CornerKey, int, CornerKeyHash> corner_map;

  auto corner_index = [&](int vi, int ti) {
    const CornerKey key{vi, ti};
    const auto found = corner_map.find(key);
    if (found != corner_map.end()) {
      return found->second;
    }
    const int index = static_cast<int>(mesh->vertices.size());
    mesh->vertices.push_back(raw_vertices[static_cast<std::size_t>(vi)]);
    if (ti >= 0 && ti < static_cast<int>(raw_texcoords.size())) {
      mesh->texcoords.push_back(raw_texcoords[static_cast<std::size_t>(ti)]);
    } else {
      mesh->texcoords.emplace_back(0.f, 0.f);
    }
    corner_map[key] = index;
    return index;
  };

  const QStringList lines =
      QString::fromStdString(text).split('\n', Qt::SkipEmptyParts);
  for (const auto& line : lines) {
    const QString trimmed = line.trimmed();
    if (trimmed.startsWith(QLatin1String("vt "))) {
      const QStringList parts = trimmed.split(' ', Qt::SkipEmptyParts);
      if (parts.size() >= 3) {
        raw_texcoords.emplace_back(parts[1].toFloat(), parts[2].toFloat());
      }
    } else if (trimmed.startsWith(QLatin1Char('v')) &&
               !trimmed.startsWith(QLatin1String("vn"))) {
      const QStringList parts = trimmed.split(' ', Qt::SkipEmptyParts);
      if (parts.size() >= 4) {
        raw_vertices.emplace_back(parts[1].toFloat(), parts[2].toFloat(),
                                 parts[3].toFloat());
      }
    } else if (trimmed.startsWith(QLatin1Char('f'))) {
      const QStringList parts = trimmed.split(' ', Qt::SkipEmptyParts);
      if (parts.size() < 4) {
        continue;
      }
      std::vector<int> indices;
      indices.reserve(static_cast<std::size_t>(parts.size() - 1));
      for (int i = 1; i < parts.size(); ++i) {
        int vi = ParseFaceIndex(parts[i], 0);
        int ti = ParseFaceIndex(parts[i], 1);
        if (vi < 0) {
          vi = static_cast<int>(raw_vertices.size()) + vi + 1;
        }
        if (ti < 0 && ti != 0) {
          ti = static_cast<int>(raw_texcoords.size()) + ti + 1;
        }
        if (ti == 0) {
          ti = -1;
        } else {
          ti -= 1;
        }
        vi -= 1;
        if (vi < 0 || vi >= static_cast<int>(raw_vertices.size())) {
          continue;
        }
        indices.push_back(corner_index(vi, ti));
      }
      for (std::size_t i = 1; i + 1 < indices.size(); ++i) {
        mesh->triangles.push_back({indices[0], indices[i], indices[i + 1]});
      }
    }
  }
  if (mesh->texcoords.size() != mesh->vertices.size()) {
    mesh->texcoords.clear();
  }
  return !mesh->vertices.empty();
}

namespace {

std::string meshFileToString(
    const automsgs::msgs::visualization_msgs::MeshFile& mesh_file) {
  std::string bytes = mesh_file.data();
  while (!bytes.empty() && bytes.back() == '\0') {
    bytes.pop_back();
  }
  return bytes;
}

bool looksLikeStlAscii(const std::string& text) {
  const std::string sample =
      text.substr(0, std::min(text.size(), static_cast<std::size_t>(256)));
  const QString lower = QString::fromStdString(sample).trimmed().toLower();
  return lower.startsWith(QLatin1String("solid")) ||
         lower.contains(QLatin1String("facet"));
}

bool looksLikeStlBinary(const std::string& data) {
  if (data.size() < 84) {
    return false;
  }
  uint32_t count = 0;
  std::memcpy(&count, data.data() + 80, sizeof(count));
  return data.size() >= 84ULL + static_cast<std::uint64_t>(count) * 50ULL;
}

}  // namespace

void ensureMeshTexcoords(ObjMesh* mesh) {
  if (mesh == nullptr || mesh->vertices.empty()) {
    return;
  }
  if (mesh->texcoords.size() == mesh->vertices.size()) {
    return;
  }
  mesh->texcoords.clear();
  mesh->texcoords.reserve(mesh->vertices.size());
  QVector3D min_v(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                  std::numeric_limits<float>::max());
  QVector3D max_v(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(),
                  -std::numeric_limits<float>::max());
  for (const QVector3D& vertex : mesh->vertices) {
    min_v.setX(std::min(min_v.x(), vertex.x()));
    min_v.setY(std::min(min_v.y(), vertex.y()));
    min_v.setZ(std::min(min_v.z(), vertex.z()));
    max_v.setX(std::max(max_v.x(), vertex.x()));
    max_v.setY(std::max(max_v.y(), vertex.y()));
    max_v.setZ(std::max(max_v.z(), vertex.z()));
  }
  const QVector3D extent = max_v - min_v;
  for (const QVector3D& vertex : mesh->vertices) {
    const float u = extent.x() > 1e-6f ? (vertex.x() - min_v.x()) / extent.x() : 0.5f;
    const float v = extent.z() > 1e-6f ? (vertex.z() - min_v.z()) / extent.z() : 0.5f;
    mesh->texcoords.emplace_back(u, v);
  }
}

bool parseStlBinary(const std::string& data, ObjMesh* mesh) {
  if (mesh == nullptr || !looksLikeStlBinary(data)) {
    return false;
  }
  uint32_t count = 0;
  std::memcpy(&count, data.data() + 80, sizeof(count));
  mesh->vertices.clear();
  mesh->texcoords.clear();
  mesh->triangles.clear();
  mesh->vertices.reserve(static_cast<std::size_t>(count) * 3);
  mesh->triangles.reserve(count);
  std::size_t offset = 84;
  for (uint32_t i = 0; i < count; ++i) {
    offset += 12;  // normal
    const int base = static_cast<int>(mesh->vertices.size());
    for (int v = 0; v < 3; ++v) {
      float coords[3] = {};
      std::memcpy(coords, data.data() + offset, sizeof(coords));
      offset += 12;
      mesh->vertices.emplace_back(coords[0], coords[1], coords[2]);
    }
    offset += 2;  // attribute byte count
    mesh->triangles.push_back({base, base + 1, base + 2});
  }
  return !mesh->vertices.empty();
}

bool parseStlAscii(const std::string& text, ObjMesh* mesh) {
  if (mesh == nullptr) {
    return false;
  }
  mesh->vertices.clear();
  mesh->triangles.clear();

  std::vector<int> face_indices;
  const QStringList lines =
      QString::fromStdString(text).split('\n', Qt::SkipEmptyParts);
  for (const auto& line : lines) {
    const QString trimmed = line.trimmed();
    if (trimmed.startsWith(QLatin1String("vertex"))) {
      const QStringList parts = trimmed.split(' ', Qt::SkipEmptyParts);
      if (parts.size() >= 4) {
        mesh->vertices.emplace_back(parts[1].toFloat(), parts[2].toFloat(),
                                    parts[3].toFloat());
        face_indices.push_back(static_cast<int>(mesh->vertices.size()) - 1);
      }
    } else if (trimmed.startsWith(QLatin1String("endfacet"))) {
      if (face_indices.size() >= 3) {
        for (std::size_t i = 1; i + 1 < face_indices.size(); ++i) {
          mesh->triangles.push_back(
              {face_indices[0], face_indices[i], face_indices[i + 1]});
        }
      }
      face_indices.clear();
    }
  }
  return !mesh->vertices.empty();
}

bool parseMeshData(const std::string& data, ObjMesh* mesh) {
  if (mesh == nullptr || data.empty()) {
    return false;
  }
  if (looksLikeStlAscii(data)) {
    return parseStlAscii(data, mesh);
  }
  if (looksLikeStlBinary(data)) {
    return parseStlBinary(data, mesh);
  }
  return parseObjText(data, mesh);
}

bool loadObjFile(const std::string& path, ObjMesh* mesh) {
  QFile file(QString::fromStdString(path));
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return false;
  }
  return parseObjText(file.readAll().toStdString(), mesh);
}

bool loadStlFile(const std::string& path, ObjMesh* mesh) {
  QFile file(QString::fromStdString(path));
  if (!file.open(QIODevice::ReadOnly)) {
    return false;
  }
  const QByteArray bytes = file.readAll();
  const std::string data(bytes.constData(), static_cast<std::size_t>(bytes.size()));
  if (looksLikeStlAscii(data)) {
    return parseStlAscii(data, mesh);
  }
  return parseStlBinary(data, mesh);
}

bool loadMeshFile(const std::string& path, ObjMesh* mesh) {
  const QString suffix =
      QFileInfo(QString::fromStdString(path)).suffix().toLower();
  if (suffix == QLatin1String("stl")) {
    return loadStlFile(path, mesh);
  }
  return loadObjFile(path, mesh);
}

bool parseMarkerMesh(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    ObjMesh* mesh) {
  if (mesh == nullptr) {
    return false;
  }
  constexpr int32_t kMeshResource = 10;
  constexpr int32_t kTriangleList = 11;

  if (marker.type() == kTriangleList) {
    mesh->vertices.clear();
    mesh->triangles.clear();
    mesh->vertices.reserve(static_cast<std::size_t>(marker.points_size()));
    for (const auto& point : marker.points()) {
      mesh->vertices.emplace_back(static_cast<float>(point.x()),
                                  static_cast<float>(point.y()),
                                  static_cast<float>(point.z()));
    }
    for (int i = 0; i + 2 < marker.points_size(); i += 3) {
      mesh->triangles.push_back({i, i + 1, i + 2});
    }
    return !mesh->vertices.empty();
  }

  if (marker.type() == kMeshResource) {
    if (!marker.mesh_file().data().empty()) {
      return parseMeshData(meshFileToString(marker.mesh_file()), mesh);
    }
    if (!marker.mesh_resource().empty() &&
        marker.mesh_resource().find("embedded://") == std::string::npos) {
      return loadMeshFile(marker.mesh_resource(), mesh);
    }
  }
  return false;
}

}  // namespace display
}  // namespace autoviz
