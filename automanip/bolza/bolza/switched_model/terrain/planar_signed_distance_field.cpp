//
// Created by rgrandia on 08.09.22.
//

#include "bolza/switched_model/terrain/planar_signed_distance_field.hpp"

namespace switched_model {

PlanarSignedDistanceField::PlanarSignedDistanceField(TerrainPlane terrainPlane) : terrainPlane_(std::move(terrainPlane)) {}

PlanarSignedDistanceField::PlanarSignedDistanceField(const PlanarSignedDistanceField& other) : terrainPlane_(other.terrainPlane_) {}

scalar_t PlanarSignedDistanceField::Value(const vector3_t& position) const {
  return TerrainSignedDistanceFromPositionInWorld(position, terrainPlane_);
}

vector3_t PlanarSignedDistanceField::Derivative(const vector3_t& position) const {
  return SurfaceNormalInWorld(terrainPlane_);
}

std::pair<scalar_t, vector3_t> PlanarSignedDistanceField::ValueAndDerivative(const vector3_t& position) const {
  return {Value(position), Derivative(position)};
}

}  // namespace switched_model
