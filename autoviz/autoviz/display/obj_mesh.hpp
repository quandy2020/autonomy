/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <array>
#include <string>
#include <vector>

#include <QVector2D>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/marker.pb.h>

namespace autoviz {
namespace display {

struct ObjMesh {
  std::vector<QVector3D> vertices;
  std::vector<QVector2D> texcoords;
  std::vector<std::array<int, 3>> triangles;
};

/** Fill missing per-vertex UVs with box projection. */
void ensureMeshTexcoords(ObjMesh* mesh);

bool parseObjText(const std::string& text, ObjMesh* mesh);
bool parseStlBinary(const std::string& data, ObjMesh* mesh);
bool parseStlAscii(const std::string& text, ObjMesh* mesh);
bool parseMeshData(const std::string& data, ObjMesh* mesh);
bool loadObjFile(const std::string& path, ObjMesh* mesh);
bool loadStlFile(const std::string& path, ObjMesh* mesh);
bool loadMeshFile(const std::string& path, ObjMesh* mesh);
bool parseMarkerMesh(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    ObjMesh* mesh);

}  // namespace display
}  // namespace autoviz
