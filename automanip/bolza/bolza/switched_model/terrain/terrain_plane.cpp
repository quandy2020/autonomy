//
// Created by rgrandia on 21.04.20.
//

#include "bolza/switched_model/terrain/terrain_plane.hpp"

#include "bolza/switched_model/core/rotations.hpp"

#include <bolza/core/misc/load_data.hpp>

namespace switched_model {

TerrainPlane LoadTerrainPlane(const std::string& filename, bool verbose) {
  bolza::property_tree::ptree pt;
  bolza::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << "\n #### terrain plane:" << std::endl;
    std::cerr << " #### ==================================================" << std::endl;
  }

  TerrainPlane plane;
  const std::string positionPrefix{"terrainPlane.position."};
  bolza::loadData::LoadPtreeValue(pt, plane.positionInWorld.x(), positionPrefix + "x", verbose);
  bolza::loadData::LoadPtreeValue(pt, plane.positionInWorld.y(), positionPrefix + "y", verbose);
  bolza::loadData::LoadPtreeValue(pt, plane.positionInWorld.z(), positionPrefix + "z", verbose);

  vector3_t eulerAnglesXYZ(0, 0, 0);
  const std::string orientationprefix{"terrainPlane.orientation."};
  bolza::loadData::LoadPtreeValue(pt, eulerAnglesXYZ.x(), orientationprefix + "roll", verbose);
  bolza::loadData::LoadPtreeValue(pt, eulerAnglesXYZ.y(), orientationprefix + "pitch", verbose);
  bolza::loadData::LoadPtreeValue(pt, eulerAnglesXYZ.z(), orientationprefix + "yaw", verbose);
  plane.orientationWorldToTerrain = RotationMatrixOriginToBase(eulerAnglesXYZ);

  if (verbose) {
    std::cerr << " #### ==================================================" << std::endl;
  }
  return plane;
}

}  // namespace switched_model
