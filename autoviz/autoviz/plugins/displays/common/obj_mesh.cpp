/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/common/obj_mesh.hpp"

#include <QColor>
#include <QDir>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>
#include <QMatrix4x4>
#include <QStringList>
#include <QVector2D>

#include <algorithm>
#include <cstring>
#include <limits>
#include <unordered_map>

#include <automsgs/msgs/visualization_msgs/marker.pb.h>

#ifdef AUTOVIZ_USE_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#endif

#include "autoviz/rendering/mesh_resource.hpp"

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
  mesh->normals.clear();
  mesh->triangles.clear();
  mesh->submeshes.clear();
  mesh->parts.clear();
  mesh->has_material_groups = false;

  std::vector<QVector3D> raw_vertices;
  std::vector<QVector2D> raw_texcoords;
  std::vector<QVector3D> raw_normals;
  struct CornerKey {
    int vi = 0;
    int ti = -1;
    int ni = -1;
    bool operator==(const CornerKey& other) const {
      return vi == other.vi && ti == other.ti && ni == other.ni;
    }
  };
  struct CornerKeyHash {
    std::size_t operator()(const CornerKey& key) const {
      return static_cast<std::size_t>(key.vi) ^
             (static_cast<std::size_t>(key.ti + 1) << 10) ^
             (static_cast<std::size_t>(key.ni + 1) << 20);
    }
  };
  std::unordered_map<CornerKey, int, CornerKeyHash> corner_map;
  std::string current_material;
  int span_begin = 0;
  auto flush_material = [&]() {
    const int count = static_cast<int>(mesh->triangles.size()) - span_begin;
    if (count <= 0) {
      return;
    }
    ObjSubmesh span;
    span.name = current_material;
    span.triangle_begin = span_begin;
    span.triangle_count = count;
    mesh->submeshes.push_back(std::move(span));
    span_begin = static_cast<int>(mesh->triangles.size());
  };

  auto corner_index = [&](int vi, int ti, int ni) {
    const CornerKey key{vi, ti, ni};
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
    if (ni >= 0 && ni < static_cast<int>(raw_normals.size())) {
      mesh->normals.push_back(raw_normals[static_cast<std::size_t>(ni)]);
    } else {
      mesh->normals.emplace_back(0.f, 0.f, 0.f);
    }
    corner_map[key] = index;
    return index;
  };

  const QStringList lines =
      QString::fromStdString(text).split('\n', Qt::SkipEmptyParts);
  for (const auto& line : lines) {
    const QString trimmed = line.trimmed();
    if (trimmed.startsWith(QLatin1String("usemtl"))) {
      flush_material();
      current_material = trimmed.mid(6).trimmed().toStdString();
    } else if (trimmed.startsWith(QLatin1String("vn "))) {
      const QStringList parts = trimmed.split(' ', Qt::SkipEmptyParts);
      if (parts.size() >= 4) {
        raw_normals.emplace_back(parts[1].toFloat(), parts[2].toFloat(),
                                 parts[3].toFloat());
      }
    } else if (trimmed.startsWith(QLatin1String("vt "))) {
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
        int ni = ParseFaceIndex(parts[i], 2);
        if (vi < 0) {
          vi = static_cast<int>(raw_vertices.size()) + vi + 1;
        }
        if (ti < 0 && ti != 0) {
          ti = static_cast<int>(raw_texcoords.size()) + ti + 1;
        }
        if (ni < 0 && ni != 0) {
          ni = static_cast<int>(raw_normals.size()) + ni + 1;
        }
        if (ti == 0) {
          ti = -1;
        } else {
          ti -= 1;
        }
        if (ni == 0) {
          ni = -1;
        } else {
          ni -= 1;
        }
        vi -= 1;
        if (vi < 0 || vi >= static_cast<int>(raw_vertices.size())) {
          continue;
        }
        indices.push_back(corner_index(vi, ti, ni));
      }
      for (std::size_t i = 1; i + 1 < indices.size(); ++i) {
        mesh->triangles.push_back({indices[0], indices[i], indices[i + 1]});
      }
    }
  }
  flush_material();
  if (mesh->texcoords.size() != mesh->vertices.size()) {
    mesh->texcoords.clear();
  }
  // Keep file normals only when every expanded corner got a real vn.
  bool has_file_normals = !raw_normals.empty() &&
                          mesh->normals.size() == mesh->vertices.size();
  if (has_file_normals) {
    for (const QVector3D& n : mesh->normals) {
      if (n.lengthSquared() < 1e-12f) {
        has_file_normals = false;
        break;
      }
    }
  }
  if (!has_file_normals) {
    mesh->normals.clear();
  }
  ensureMeshNormals(mesh);
  return !mesh->vertices.empty();
}

namespace {

struct MtlMaterial {
  QColor diffuse{255, 255, 255};
  QString texture;
};

std::unordered_map<std::string, MtlMaterial> ParseMtlFile(const QString& path) {
  std::unordered_map<std::string, MtlMaterial> library;
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return library;
  }
  const QDir directory = QFileInfo(path).absoluteDir();
  std::string current;
  const QStringList lines =
      QString::fromUtf8(file.readAll()).split('\n', Qt::SkipEmptyParts);
  for (const QString& line : lines) {
    const QString trimmed = line.trimmed();
    if (trimmed.startsWith(QLatin1String("newmtl"))) {
      current = trimmed.mid(6).trimmed().toStdString();
      library[current] = MtlMaterial{};
    } else if (current.empty()) {
      continue;
    } else if (trimmed.startsWith(QLatin1String("Kd "))) {
      const QStringList parts = trimmed.split(' ', Qt::SkipEmptyParts);
      if (parts.size() >= 4) {
        library[current].diffuse =
            QColor::fromRgbF(std::clamp(parts[1].toFloat(), 0.f, 1.f),
                             std::clamp(parts[2].toFloat(), 0.f, 1.f),
                             std::clamp(parts[3].toFloat(), 0.f, 1.f));
      }
    } else if (trimmed.startsWith(QLatin1String("map_Kd"))) {
      const QStringList parts = trimmed.split(' ', Qt::SkipEmptyParts);
      if (parts.size() >= 2) {
        library[current].texture =
            directory.filePath(parts.back());
      }
    }
  }
  return library;
}

}  // namespace

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

void ensureMeshNormals(ObjMesh* mesh) {
  if (mesh == nullptr || mesh->vertices.empty()) {
    return;
  }
  if (mesh->normals.size() == mesh->vertices.size()) {
    bool any = false;
    for (QVector3D& n : mesh->normals) {
      const float len = n.length();
      if (len > 1e-6f) {
        n /= len;
        any = true;
      }
    }
    if (any) {
      return;
    }
  }
  mesh->normals.assign(mesh->vertices.size(), QVector3D(0.f, 0.f, 0.f));
  for (const auto& tri : mesh->triangles) {
    if (tri[0] < 0 || tri[1] < 0 || tri[2] < 0 ||
        tri[0] >= static_cast<int>(mesh->vertices.size()) ||
        tri[1] >= static_cast<int>(mesh->vertices.size()) ||
        tri[2] >= static_cast<int>(mesh->vertices.size())) {
      continue;
    }
    const QVector3D& a = mesh->vertices[static_cast<std::size_t>(tri[0])];
    const QVector3D& b = mesh->vertices[static_cast<std::size_t>(tri[1])];
    const QVector3D& c = mesh->vertices[static_cast<std::size_t>(tri[2])];
    const QVector3D face = QVector3D::crossProduct(b - a, c - a);
    mesh->normals[static_cast<std::size_t>(tri[0])] += face;
    mesh->normals[static_cast<std::size_t>(tri[1])] += face;
    mesh->normals[static_cast<std::size_t>(tri[2])] += face;
  }
  for (QVector3D& n : mesh->normals) {
    const float len = n.length();
    if (len > 1e-6f) {
      n /= len;
    } else {
      n = QVector3D(0.f, 0.f, 1.f);
    }
  }
}

/** Slice @c submeshes into stable @c parts for Ogre textured-link pointers. */
void FinalizeMaterialParts(ObjMesh* mesh) {
  if (mesh == nullptr) {
    return;
  }
  mesh->parts.clear();
  if (!mesh->has_material_groups || mesh->submeshes.empty()) {
    return;
  }
  mesh->parts.reserve(mesh->submeshes.size());
  for (const ObjSubmesh& part : mesh->submeshes) {
    ObjMesh slice;
    if (part.triangle_count <= 0) {
      mesh->parts.push_back(std::move(slice));
      continue;
    }
    const int begin = part.triangle_begin;
    const int end = begin + part.triangle_count;
    if (begin < 0 || end > static_cast<int>(mesh->triangles.size())) {
      mesh->parts.push_back(std::move(slice));
      continue;
    }
    slice.vertices = mesh->vertices;
    slice.texcoords = mesh->texcoords;
    slice.normals = mesh->normals;
    slice.triangles.assign(mesh->triangles.begin() + begin,
                           mesh->triangles.begin() + end);
    ensureMeshTexcoords(&slice);
    ensureMeshNormals(&slice);
    mesh->parts.push_back(std::move(slice));
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
  const QByteArray bytes = file.readAll();
  if (!parseObjText(std::string(bytes.constData(), static_cast<std::size_t>(bytes.size())),
                    mesh)) {
    return false;
  }
  QString mtllib;
  const QStringList lines =
      QString::fromUtf8(bytes).split('\n', Qt::SkipEmptyParts);
  for (const QString& line : lines) {
    const QString trimmed = line.trimmed();
    if (trimmed.startsWith(QLatin1String("mtllib"))) {
      mtllib = trimmed.mid(6).trimmed();
      break;
    }
  }
  if (mtllib.isEmpty() || mesh->submeshes.empty()) {
    mesh->submeshes.clear();
    return true;
  }
  const QString mtl_path = QFileInfo(QString::fromStdString(path)).absoluteDir().filePath(mtllib);
  const auto library = ParseMtlFile(mtl_path);
  if (library.empty()) {
    mesh->submeshes.clear();
    return true;
  }
  for (ObjSubmesh& part : mesh->submeshes) {
    const auto found = library.find(part.name);
    if (found == library.end()) {
      continue;
    }
    part.diffuse = found->second.diffuse;
    part.texture_path = found->second.texture.toStdString();
    if (!part.texture_path.empty()) {
      part.texture = QImage(QString::fromStdString(part.texture_path));
    }
  }
  mesh->has_material_groups = true;
  FinalizeMaterialParts(mesh);
  return true;
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

namespace {

QString LocalTag(const QDomNode& node) {
  const QString name = node.nodeName();
  const int colon = name.lastIndexOf(QLatin1Char(':'));
  return colon < 0 ? name : name.mid(colon + 1);
}

QMatrix4x4 ColladaMatrix(const QString& text) {
  const QStringList parts = text.simplified().split(QLatin1Char(' '), Qt::SkipEmptyParts);
  // COLLADA stores the 16 values in column-major order.
  float values[16] = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
                      0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
  for (int i = 0; i < parts.size() && i < 16; ++i) {
    values[i] = parts[i].toFloat();
  }
  return QMatrix4x4(values);
}

struct ColladaGeometry {
  std::vector<QVector3D> vertices;
  std::vector<std::array<int, 3>> triangles;
};

ColladaGeometry ReadColladaMesh(const QDomElement& mesh_node) {
  ColladaGeometry geometry;
  std::unordered_map<QString, std::vector<float>> arrays;
  QString position_source;
  for (QDomNode child = mesh_node.firstChild(); !child.isNull(); child = child.nextSibling()) {
    const QDomElement element = child.toElement();
    if (element.isNull()) {
      continue;
    }
    if (LocalTag(element) == QLatin1String("source")) {
      for (QDomNode source_child = element.firstChild(); !source_child.isNull();
           source_child = source_child.nextSibling()) {
        const QDomElement source_element = source_child.toElement();
        if (!source_element.isNull() &&
            LocalTag(source_element) == QLatin1String("float_array")) {
          std::vector<float> values;
          const QStringList parts =
              source_element.text().simplified().split(QLatin1Char(' '), Qt::SkipEmptyParts);
          values.reserve(static_cast<std::size_t>(parts.size()));
          for (const QString& part : parts) {
            values.push_back(part.toFloat());
          }
          arrays.emplace(element.attribute(QStringLiteral("id")), std::move(values));
        }
      }
    } else if (LocalTag(element) == QLatin1String("vertices")) {
      for (QDomNode input = element.firstChild(); !input.isNull(); input = input.nextSibling()) {
        const QDomElement input_element = input.toElement();
        if (!input_element.isNull() && LocalTag(input_element) == QLatin1String("input") &&
            input_element.attribute(QStringLiteral("semantic")) == QLatin1String("POSITION")) {
          position_source = input_element.attribute(QStringLiteral("source"));
          if (position_source.startsWith(QLatin1Char('#'))) {
            position_source.remove(0, 1);
          }
        }
      }
    }
  }
  const auto positions = arrays.find(position_source);
  if (positions == arrays.end()) {
    return geometry;
  }
  geometry.vertices.reserve(positions->second.size() / 3);
  for (std::size_t i = 0; i + 2 < positions->second.size(); i += 3) {
    geometry.vertices.emplace_back(positions->second[i], positions->second[i + 1],
                                   positions->second[i + 2]);
  }

  for (QDomNode child = mesh_node.firstChild(); !child.isNull(); child = child.nextSibling()) {
    const QDomElement triangles = child.toElement();
    if (triangles.isNull() || LocalTag(triangles) != QLatin1String("triangles")) {
      continue;
    }
    int stride = 1;
    int vertex_offset = 0;
    QString indices_text;
    for (QDomNode input = triangles.firstChild(); !input.isNull(); input = input.nextSibling()) {
      const QDomElement input_element = input.toElement();
      if (input_element.isNull()) {
        continue;
      }
      if (LocalTag(input_element) == QLatin1String("input")) {
        const int offset = input_element.attribute(QStringLiteral("offset")).toInt();
        stride = std::max(stride, offset + 1);
        if (input_element.attribute(QStringLiteral("semantic")) == QLatin1String("VERTEX")) {
          vertex_offset = offset;
        }
      } else if (LocalTag(input_element) == QLatin1String("p")) {
        indices_text = input_element.text();
      }
    }
    const QStringList indices =
        indices_text.simplified().split(QLatin1Char(' '), Qt::SkipEmptyParts);
    const int step = stride * 3;
    for (int i = 0; i + step - 1 < indices.size(); i += step) {
      const int i0 = indices[i + vertex_offset].toInt();
      const int i1 = indices[i + stride + vertex_offset].toInt();
      const int i2 = indices[i + 2 * stride + vertex_offset].toInt();
      if (i0 < 0 || i1 < 0 || i2 < 0 ||
          i0 >= static_cast<int>(geometry.vertices.size()) ||
          i1 >= static_cast<int>(geometry.vertices.size()) ||
          i2 >= static_cast<int>(geometry.vertices.size())) {
        continue;
      }
      geometry.triangles.push_back({i0, i1, i2});
    }
  }
  return geometry;
}

void AppendColladaNode(const QDomElement& node, const QMatrix4x4& parent,
                       const std::unordered_map<QString, ColladaGeometry>& geometries,
                       ObjMesh* mesh) {
  QMatrix4x4 world = parent;
  for (QDomNode child = node.firstChild(); !child.isNull(); child = child.nextSibling()) {
    const QDomElement element = child.toElement();
    if (!element.isNull() && LocalTag(element) == QLatin1String("matrix")) {
      world = parent * ColladaMatrix(element.text());
      break;
    }
  }
  for (QDomNode child = node.firstChild(); !child.isNull(); child = child.nextSibling()) {
    const QDomElement element = child.toElement();
    if (element.isNull()) {
      continue;
    }
    if (LocalTag(element) == QLatin1String("instance_geometry")) {
      QString url = element.attribute(QStringLiteral("url"));
      if (url.startsWith(QLatin1Char('#'))) {
        url.remove(0, 1);
      }
      const auto found = geometries.find(url);
      if (found == geometries.end()) {
        continue;
      }
      const int base = static_cast<int>(mesh->vertices.size());
      mesh->vertices.reserve(mesh->vertices.size() + found->second.vertices.size());
      for (const QVector3D& vertex : found->second.vertices) {
        mesh->vertices.push_back(world.map(vertex));
      }
      mesh->triangles.reserve(mesh->triangles.size() + found->second.triangles.size());
      for (const auto& triangle : found->second.triangles) {
        mesh->triangles.push_back(
            {base + triangle[0], base + triangle[1], base + triangle[2]});
      }
    } else if (LocalTag(element) == QLatin1String("node")) {
      AppendColladaNode(element, world, geometries, mesh);
    }
  }
}

#ifdef AUTOVIZ_USE_ASSIMP
QImage LoadAssimpTexture(const aiScene* scene, const char* tex_name,
                         const std::string& mesh_path) {
  if (scene == nullptr || tex_name == nullptr || tex_name[0] == '\0') {
    return {};
  }
  // Embedded: "*0", "*1", …
  if (tex_name[0] == '*') {
    const aiTexture* embedded = scene->GetEmbeddedTexture(tex_name);
    if (embedded == nullptr) {
      return {};
    }
    if (embedded->mHeight == 0) {
      return QImage::fromData(reinterpret_cast<const uchar*>(embedded->pcData),
                              static_cast<int>(embedded->mWidth));
    }
    QImage image(static_cast<int>(embedded->mWidth),
                 static_cast<int>(embedded->mHeight), QImage::Format_RGBA8888);
    for (unsigned int y = 0; y < embedded->mHeight; ++y) {
      for (unsigned int x = 0; x < embedded->mWidth; ++x) {
        const aiTexel& texel =
            embedded->pcData[y * embedded->mWidth + x];
        image.setPixelColor(static_cast<int>(x), static_cast<int>(y),
                            QColor(texel.r, texel.g, texel.b, texel.a));
      }
    }
    return image;
  }
  QFileInfo mesh_info(QString::fromStdString(mesh_path));
  const QString candidate =
      mesh_info.absoluteDir().filePath(QString::fromUtf8(tex_name));
  QImage image(candidate);
  if (!image.isNull()) {
    return image;
  }
  // Assimp may pass "hip.jpg" while COLLADA listed only the image id "hip".
  const QString stem = QFileInfo(QString::fromUtf8(tex_name)).completeBaseName();
  for (const char* ext : {".jpg", ".jpeg", ".png", ".JPG", ".PNG"}) {
    image = QImage(mesh_info.absoluteDir().filePath(stem + QLatin1String(ext)));
    if (!image.isNull()) {
      return image;
    }
  }
  return {};
}

QMatrix4x4 AssimpMatrixToQt(const aiMatrix4x4& m) {
  return QMatrix4x4(m.a1, m.a2, m.a3, m.a4, m.b1, m.b2, m.b3, m.b4, m.c1, m.c2,
                    m.c3, m.c4, m.d1, m.d2, m.d3, m.d4);
}

void FillSubmeshMaterial(const aiScene* scene, unsigned int material_index,
                         const std::string& path, ObjSubmesh* part) {
  if (part == nullptr || scene == nullptr ||
      material_index >= scene->mNumMaterials) {
    return;
  }
  const aiMaterial* mat = scene->mMaterials[material_index];
  aiColor4D diffuse(1.f, 1.f, 1.f, 1.f);
  if (aiGetMaterialColor(mat, AI_MATKEY_COLOR_DIFFUSE, &diffuse) == AI_SUCCESS) {
    part->diffuse =
        QColor::fromRgbF(diffuse.r, diffuse.g, diffuse.b, diffuse.a);
  }
  aiString tex_name;
  if (mat->GetTexture(aiTextureType_DIFFUSE, 0, &tex_name) == AI_SUCCESS ||
      mat->GetTexture(aiTextureType_BASE_COLOR, 0, &tex_name) == AI_SUCCESS) {
    part->texture = LoadAssimpTexture(scene, tex_name.C_Str(), path);
    if (part->texture.isNull() && tex_name.length > 0 &&
        tex_name.C_Str()[0] != '*') {
      part->texture_path = QFileInfo(QString::fromStdString(path))
                               .absoluteDir()
                               .filePath(QString::fromUtf8(tex_name.C_Str()))
                               .toStdString();
      part->texture = QImage(QString::fromStdString(part->texture_path));
    }
  }
  if (part->name.empty()) {
    aiString mat_name;
    if (mat->Get(AI_MATKEY_NAME, mat_name) == AI_SUCCESS) {
      part->name = mat_name.C_Str();
    }
  }
}

void AppendAssimpNode(const aiNode* node, const QMatrix4x4& parent,
                      const aiScene* scene, const std::string& path,
                      ObjMesh* mesh) {
  if (node == nullptr || scene == nullptr || mesh == nullptr) {
    return;
  }
  const QMatrix4x4 world = parent * AssimpMatrixToQt(node->mTransformation);
  for (unsigned int i = 0; i < node->mNumMeshes; ++i) {
    const unsigned int mesh_index = node->mMeshes[i];
    if (mesh_index >= scene->mNumMeshes) {
      continue;
    }
    const aiMesh* src = scene->mMeshes[mesh_index];
    if (src == nullptr || src->mNumVertices == 0 || src->mNumFaces == 0) {
      continue;
    }
    ObjSubmesh part;
    part.name = src->mName.C_Str();
    part.triangle_begin = static_cast<int>(mesh->triangles.size());
    const int base = static_cast<int>(mesh->vertices.size());
    mesh->vertices.reserve(mesh->vertices.size() + src->mNumVertices);
    mesh->texcoords.reserve(mesh->texcoords.size() + src->mNumVertices);
    mesh->normals.reserve(mesh->normals.size() + src->mNumVertices);
    const bool has_uv = src->HasTextureCoords(0);
    const bool has_n = src->HasNormals();
    for (unsigned int vi = 0; vi < src->mNumVertices; ++vi) {
      const QVector3D local(src->mVertices[vi].x, src->mVertices[vi].y,
                            src->mVertices[vi].z);
      mesh->vertices.push_back(world.map(local));
      if (has_uv) {
        mesh->texcoords.emplace_back(src->mTextureCoords[0][vi].x,
                                     src->mTextureCoords[0][vi].y);
      } else {
        mesh->texcoords.emplace_back(0.f, 0.f);
      }
      if (has_n) {
        const QVector3D n =
            world.mapVector(QVector3D(src->mNormals[vi].x, src->mNormals[vi].y,
                                      src->mNormals[vi].z))
                .normalized();
        mesh->normals.push_back(n);
      } else {
        mesh->normals.emplace_back(0.f, 0.f, 1.f);
      }
    }
    int added = 0;
    for (unsigned int fi = 0; fi < src->mNumFaces; ++fi) {
      const aiFace& face = src->mFaces[fi];
      if (face.mNumIndices != 3) {
        continue;
      }
      mesh->triangles.push_back(
          {base + static_cast<int>(face.mIndices[0]),
           base + static_cast<int>(face.mIndices[1]),
           base + static_cast<int>(face.mIndices[2])});
      ++added;
    }
    part.triangle_count = added;
    FillSubmeshMaterial(scene, src->mMaterialIndex, path, &part);
    mesh->submeshes.push_back(std::move(part));
  }
  for (unsigned int c = 0; c < node->mNumChildren; ++c) {
    AppendAssimpNode(node->mChildren[c], world, scene, path, mesh);
  }
}

bool loadColladaAssimp(const std::string& path, ObjMesh* mesh) {
  if (mesh == nullptr) {
    return false;
  }
  Assimp::Importer importer;
  // Match parseCollada: keep file up-axis / node matrices (ANYmal hip_* DAEs
  // carry a ~90° visual_scene transform). Do not bake with
  // PreTransformVertices — that path drifted from URDF link frames.
  importer.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION, true);
  // No FlipUVs: Ogre textured links already flip V (1-v), matching OBJ path.
  const aiScene* scene = importer.ReadFile(
      path, aiProcess_Triangulate | aiProcess_GenSmoothNormals |
                aiProcess_JoinIdenticalVertices | aiProcess_SortByPType |
                aiProcess_GenUVCoords);
  if (scene == nullptr || !scene->HasMeshes() || scene->mRootNode == nullptr) {
    return false;
  }

  mesh->vertices.clear();
  mesh->texcoords.clear();
  mesh->normals.clear();
  mesh->triangles.clear();
  mesh->submeshes.clear();
  mesh->parts.clear();
  mesh->has_material_groups = false;

  AppendAssimpNode(scene->mRootNode, QMatrix4x4{}, scene, path, mesh);

  if (mesh->triangles.empty()) {
    return false;
  }
  mesh->has_material_groups = !mesh->submeshes.empty();
  ensureMeshNormals(mesh);
  FinalizeMaterialParts(mesh);
  return true;
}
#endif  // AUTOVIZ_USE_ASSIMP

bool parseCollada(const QByteArray& bytes, ObjMesh* mesh) {
  if (mesh == nullptr) {
    return false;
  }
  mesh->vertices.clear();
  mesh->texcoords.clear();
  mesh->triangles.clear();
  mesh->submeshes.clear();
  mesh->parts.clear();
  mesh->has_material_groups = false;

  QDomDocument document;
  if (!document.setContent(bytes)) {
    return false;
  }
  std::unordered_map<QString, ColladaGeometry> geometries;
  const QDomNodeList geometry_nodes =
      document.elementsByTagName(QStringLiteral("geometry"));
  for (int i = 0; i < geometry_nodes.size(); ++i) {
    const QDomElement geometry = geometry_nodes.at(i).toElement();
    QDomElement mesh_node;
    for (QDomNode child = geometry.firstChild(); !child.isNull(); child = child.nextSibling()) {
      const QDomElement element = child.toElement();
      if (!element.isNull() && LocalTag(element) == QLatin1String("mesh")) {
        mesh_node = element;
        break;
      }
    }
    if (mesh_node.isNull()) {
      continue;
    }
    ColladaGeometry parsed = ReadColladaMesh(mesh_node);
    if (!parsed.vertices.empty() && !parsed.triangles.empty()) {
      geometries.emplace(geometry.attribute(QStringLiteral("id")), std::move(parsed));
    }
  }
  if (geometries.empty()) {
    return false;
  }

  float unit = 1.f;
  const QDomNodeList units = document.elementsByTagName(QStringLiteral("unit"));
  if (units.size() > 0) {
    const QString meter = units.at(0).toElement().attribute(QStringLiteral("meter"));
    if (!meter.isEmpty()) {
      unit = meter.toFloat();
    }
  }
  QMatrix4x4 root;
  root.scale(unit > 0.f ? unit : 1.f);

  bool placed = false;
  const QDomNodeList scenes = document.elementsByTagName(QStringLiteral("visual_scene"));
  for (int i = 0; i < scenes.size(); ++i) {
    for (QDomNode child = scenes.at(i).firstChild(); !child.isNull();
         child = child.nextSibling()) {
      const QDomElement element = child.toElement();
      if (!element.isNull() && LocalTag(element) == QLatin1String("node")) {
        AppendColladaNode(element, root, geometries, mesh);
        placed = true;
      }
    }
  }
  if (!placed) {
    for (const auto& entry : geometries) {
      const int base = static_cast<int>(mesh->vertices.size());
      for (const QVector3D& vertex : entry.second.vertices) {
        mesh->vertices.push_back(root.map(vertex));
      }
      for (const auto& triangle : entry.second.triangles) {
        mesh->triangles.push_back(
            {base + triangle[0], base + triangle[1], base + triangle[2]});
      }
    }
  }
  return !mesh->triangles.empty();
}

}  // namespace

bool loadMeshFile(const std::string& path, ObjMesh* mesh) {
  // Accept package:// / file://$(find …) the same way RViz resource_retriever does.
  std::string resolved = path;
  if (const auto mapped =
          rendering::MeshResourceResolver::instance().resolvePath(path)) {
    resolved = *mapped;
  }
  const QString suffix =
      QFileInfo(QString::fromStdString(resolved)).suffix().toLower();
  if (suffix == QLatin1String("stl")) {
    return loadStlFile(resolved, mesh);
  }
  if (suffix == QLatin1String("dae")) {
#ifdef AUTOVIZ_USE_ASSIMP
    // Assimp reads COLLADA materials + embedded images (ANYmal C / RViz parity).
    if (loadColladaAssimp(resolved, mesh)) {
      return true;
    }
#endif
    QFile file(QString::fromStdString(resolved));
    if (!file.open(QIODevice::ReadOnly)) {
      return false;
    }
    return parseCollada(file.readAll(), mesh);
  }
  return loadObjFile(resolved, mesh);
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
    mesh->colors.clear();
    mesh->vertices.reserve(static_cast<std::size_t>(marker.points_size()));
    for (const auto& point : marker.points()) {
      mesh->vertices.emplace_back(static_cast<float>(point.x()),
                                  static_cast<float>(point.y()),
                                  static_cast<float>(point.z()));
    }
    for (int i = 0; i + 2 < marker.points_size(); i += 3) {
      mesh->triangles.push_back({i, i + 1, i + 2});
    }
    // RViz TriangleListMarker: colors.size()==points → per-vertex;
    // colors.size()==points/3 → per-triangle (expand to vertices).
    const int n = marker.points_size();
    const int nc = marker.colors_size();
    auto to_qcolor = [](const automsgs::msgs::std_msgs::ColorRGBA& c) {
      return QColor(static_cast<int>(std::clamp(c.r(), 0.f, 1.f) * 255.f),
                    static_cast<int>(std::clamp(c.g(), 0.f, 1.f) * 255.f),
                    static_cast<int>(std::clamp(c.b(), 0.f, 1.f) * 255.f),
                    static_cast<int>(std::clamp(c.a(), 0.f, 1.f) * 255.f));
    };
    if (nc == n && n > 0) {
      mesh->colors.reserve(static_cast<std::size_t>(n));
      for (const auto& c : marker.colors()) {
        mesh->colors.push_back(to_qcolor(c));
      }
    } else if (nc * 3 == n && nc > 0) {
      mesh->colors.resize(static_cast<std::size_t>(n));
      for (int t = 0; t < nc; ++t) {
        const QColor qc = to_qcolor(marker.colors(t));
        mesh->colors[static_cast<std::size_t>(t * 3 + 0)] = qc;
        mesh->colors[static_cast<std::size_t>(t * 3 + 1)] = qc;
        mesh->colors[static_cast<std::size_t>(t * 3 + 2)] = qc;
      }
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
