/*
 * Copyright 2026 The Openbot Authors
 *
 * Voxel occupancy distance field for CHOMP (MoveIt distance_field lite).
 */

#pragma once

#include <cmath>
#include <limits>
#include <vector>

#include "autonomy/manipulation/motion/scene/planning_scene.hpp"

namespace autonomy {
namespace manipulation {
namespace planning {

/**
 * @brief Axis-aligned voxel grid storing truncated signed distance.
 *
 * Built by rasterizing scene primitives / occupancy into binary occupancy,
 * then a multi-pass chamfer distance transform (positive outside).
 */
class VoxelDistanceField {
 public:
  /**
   * @brief Build DF from planning scene world objects + occupancy.
   * @param[in] scene Source geometry (may be null → empty field).
   * @param[in] resolution Voxel edge length (meters).
   * @param[in] padding Extra AABB margin around scene bounds.
   * @param[in] max_distance Truncation distance for queries.
   */
  void Build(const scene::PlanningScene* scene, double resolution = 0.05,
             double padding = 0.3, double max_distance = 0.5);

  bool empty() const { return nx_ <= 0 || data_.empty(); }

  /** @brief Truncated SDF at world point (positive outside obstacles). */
  double Distance(double x, double y, double z) const;

  /** @brief Central-difference gradient of Distance. */
  void Gradient(double x, double y, double z, double* gx, double* gy,
                double* gz) const;

  double resolution() const { return resolution_; }

 private:
  int Index(int ix, int iy, int iz) const {
    return iz * nx_ * ny_ + iy * nx_ + ix;
  }
  bool InBounds(int ix, int iy, int iz) const {
    return ix >= 0 && iy >= 0 && iz >= 0 && ix < nx_ && iy < ny_ && iz < nz_;
  }
  void WorldToIndex(double x, double y, double z, int* ix, int* iy,
                    int* iz) const;
  void RasterizeSphere(double cx, double cy, double cz, double r);
  void RasterizeBox(double cx, double cy, double cz, double sx, double sy,
                    double sz);
  void ChamferTransform();

  double resolution_ = 0.05;
  double max_distance_ = 0.5;
  double ox_ = 0, oy_ = 0, oz_ = 0;
  int nx_ = 0, ny_ = 0, nz_ = 0;
  std::vector<float> data_;  // truncated SDF
};

}  // namespace planning
}  // namespace manipulation
}  // namespace autonomy
