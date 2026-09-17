/*
 * Copyright 2026 The Openbot Authors
 *
 * Chamfer voxel distance field from PlanningScene geometry.
 */

#include "autonomy/manipulation/planner/chomp/voxel_distance_field.hpp"

#include <algorithm>
#include <cmath>

#include <automsgs/msgs/shape_msgs/solid_primitive.pb.h>

#include "autonomy/manipulation/motion/scene/collision_object_helpers.hpp"

namespace autonomy {
namespace manipulation {
namespace planner {
namespace {

double PrimitiveSdf(const automsgs::msgs::moveit_msgs::CollisionObject& o, double x, double y,
                    double z) {
  using SP = automsgs::msgs::shape_msgs::SolidPrimitive;
  const auto pose = scene::GetObjectPose(o);
  const double dx = x - pose.position().x();
  const double dy = y - pose.position().y();
  const double dz = z - pose.position().z();
  double sx = 0.0;
  double sy = 0.0;
  double sz = 0.0;
  scene::GetPrimitiveSizes(o, &sx, &sy, &sz);
  if (scene::GetPrimitiveType(o) == SP::SPHERE) {
    return std::sqrt(dx * dx + dy * dy + dz * dz) - sx;
  }
  if (scene::GetPrimitiveType(o) == SP::CYLINDER) {
    const double radial = std::sqrt(dx * dx + dy * dy) - sx;
    const double axial = std::abs(dz) - 0.5 * sz;
    if (radial > 0.0 && axial > 0.0) {
      return std::sqrt(radial * radial + axial * axial);
    }
    return std::max(radial, axial);
  }
  const double qx = std::abs(dx) - 0.5 * sx;
  const double qy = std::abs(dy) - 0.5 * sy;
  const double qz = std::abs(dz) - 0.5 * sz;
  const double outside =
      std::sqrt(std::max(qx, 0.0) * std::max(qx, 0.0) +
                std::max(qy, 0.0) * std::max(qy, 0.0) +
                std::max(qz, 0.0) * std::max(qz, 0.0));
  const double inside = std::min({qx, qy, qz});
  return outside + std::min(inside, 0.0);
}

}  // namespace

void VoxelDistanceField::WorldToIndex(double x, double y, double z, int* ix,
                                      int* iy, int* iz) const {
  *ix = static_cast<int>(std::floor((x - ox_) / resolution_));
  *iy = static_cast<int>(std::floor((y - oy_) / resolution_));
  *iz = static_cast<int>(std::floor((z - oz_) / resolution_));
}

void VoxelDistanceField::RasterizeSphere(double cx, double cy, double cz,
                                         double r) {
  const int ir = static_cast<int>(std::ceil(r / resolution_)) + 1;
  int cx_i, cy_i, cz_i;
  WorldToIndex(cx, cy, cz, &cx_i, &cy_i, &cz_i);
  for (int iz = cz_i - ir; iz <= cz_i + ir; ++iz) {
    for (int iy = cy_i - ir; iy <= cy_i + ir; ++iy) {
      for (int ix = cx_i - ir; ix <= cx_i + ir; ++ix) {
        if (!InBounds(ix, iy, iz)) {
          continue;
        }
        const double x = ox_ + (ix + 0.5) * resolution_;
        const double y = oy_ + (iy + 0.5) * resolution_;
        const double z = oz_ + (iz + 0.5) * resolution_;
        const double d = std::sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy) +
                                   (z - cz) * (z - cz));
        if (d <= r) {
          data_[static_cast<std::size_t>(Index(ix, iy, iz))] = 0.0f;
        }
      }
    }
  }
}

void VoxelDistanceField::RasterizeBox(double cx, double cy, double cz,
                                      double sx, double sy, double sz) {
  const double hx = 0.5 * sx;
  const double hy = 0.5 * sy;
  const double hz = 0.5 * sz;
  int i0, j0, k0, i1, j1, k1;
  WorldToIndex(cx - hx, cy - hy, cz - hz, &i0, &j0, &k0);
  WorldToIndex(cx + hx, cy + hy, cz + hz, &i1, &j1, &k1);
  for (int iz = k0; iz <= k1; ++iz) {
    for (int iy = j0; iy <= j1; ++iy) {
      for (int ix = i0; ix <= i1; ++ix) {
        if (!InBounds(ix, iy, iz)) {
          continue;
        }
        data_[static_cast<std::size_t>(Index(ix, iy, iz))] = 0.0f;
      }
    }
  }
}

void VoxelDistanceField::ChamferTransform() {
  // Initialize free cells to max_distance, occupied stay 0.
  const float maxf = static_cast<float>(max_distance_);
  std::vector<char> occ(data_.size(), 0);
  for (std::size_t i = 0; i < data_.size(); ++i) {
    if (data_[i] <= 0.0f) {
      occ[i] = 1;
      data_[i] = 0.0f;
    } else {
      data_[i] = maxf;
    }
  }
  const float d1 = static_cast<float>(resolution_);
  const float d2 = static_cast<float>(resolution_ * std::sqrt(2.0));
  const float d3 = static_cast<float>(resolution_ * std::sqrt(3.0));
  auto at = [&](int ix, int iy, int iz) -> float& {
    return data_[static_cast<std::size_t>(Index(ix, iy, iz))];
  };
  // Forward pass (outside distance).
  for (int iz = 0; iz < nz_; ++iz) {
    for (int iy = 0; iy < ny_; ++iy) {
      for (int ix = 0; ix < nx_; ++ix) {
        float v = at(ix, iy, iz);
        if (ix > 0) {
          v = std::min(v, at(ix - 1, iy, iz) + d1);
        }
        if (iy > 0) {
          v = std::min(v, at(ix, iy - 1, iz) + d1);
        }
        if (iz > 0) {
          v = std::min(v, at(ix, iy, iz - 1) + d1);
        }
        if (ix > 0 && iy > 0) {
          v = std::min(v, at(ix - 1, iy - 1, iz) + d2);
        }
        if (ix > 0 && iz > 0) {
          v = std::min(v, at(ix - 1, iy, iz - 1) + d2);
        }
        if (iy > 0 && iz > 0) {
          v = std::min(v, at(ix, iy - 1, iz - 1) + d2);
        }
        if (ix > 0 && iy > 0 && iz > 0) {
          v = std::min(v, at(ix - 1, iy - 1, iz - 1) + d3);
        }
        at(ix, iy, iz) = v;
      }
    }
  }
  // Backward pass.
  for (int iz = nz_ - 1; iz >= 0; --iz) {
    for (int iy = ny_ - 1; iy >= 0; --iy) {
      for (int ix = nx_ - 1; ix >= 0; --ix) {
        float v = at(ix, iy, iz);
        if (ix + 1 < nx_) {
          v = std::min(v, at(ix + 1, iy, iz) + d1);
        }
        if (iy + 1 < ny_) {
          v = std::min(v, at(ix, iy + 1, iz) + d1);
        }
        if (iz + 1 < nz_) {
          v = std::min(v, at(ix, iy, iz + 1) + d1);
        }
        if (ix + 1 < nx_ && iy + 1 < ny_) {
          v = std::min(v, at(ix + 1, iy + 1, iz) + d2);
        }
        if (ix + 1 < nx_ && iz + 1 < nz_) {
          v = std::min(v, at(ix + 1, iy, iz + 1) + d2);
        }
        if (iy + 1 < ny_ && iz + 1 < nz_) {
          v = std::min(v, at(ix, iy + 1, iz + 1) + d2);
        }
        if (ix + 1 < nx_ && iy + 1 < ny_ && iz + 1 < nz_) {
          v = std::min(v, at(ix + 1, iy + 1, iz + 1) + d3);
        }
        at(ix, iy, iz) = v;
      }
    }
  }

  // Inside: invert and re-run distance from free→occupied for signed field.
  // Occupied voxels get negative distance to nearest free cell.
  std::vector<float> inside(data_.size(), maxf);
  for (std::size_t i = 0; i < data_.size(); ++i) {
    inside[i] = occ[i] ? maxf : 0.0f;
  }
  auto ati = [&](int ix, int iy, int iz) -> float& {
    return inside[static_cast<std::size_t>(Index(ix, iy, iz))];
  };
  for (int iz = 0; iz < nz_; ++iz) {
    for (int iy = 0; iy < ny_; ++iy) {
      for (int ix = 0; ix < nx_; ++ix) {
        float v = ati(ix, iy, iz);
        if (ix > 0) {
          v = std::min(v, ati(ix - 1, iy, iz) + d1);
        }
        if (iy > 0) {
          v = std::min(v, ati(ix, iy - 1, iz) + d1);
        }
        if (iz > 0) {
          v = std::min(v, ati(ix, iy, iz - 1) + d1);
        }
        ati(ix, iy, iz) = v;
      }
    }
  }
  for (int iz = nz_ - 1; iz >= 0; --iz) {
    for (int iy = ny_ - 1; iy >= 0; --iy) {
      for (int ix = nx_ - 1; ix >= 0; --ix) {
        float v = ati(ix, iy, iz);
        if (ix + 1 < nx_) {
          v = std::min(v, ati(ix + 1, iy, iz) + d1);
        }
        if (iy + 1 < ny_) {
          v = std::min(v, ati(ix, iy + 1, iz) + d1);
        }
        if (iz + 1 < nz_) {
          v = std::min(v, ati(ix, iy, iz + 1) + d1);
        }
        ati(ix, iy, iz) = v;
      }
    }
  }
  for (std::size_t i = 0; i < data_.size(); ++i) {
    if (occ[i]) {
      // Negative inside: -inside_dist (at least one voxel deep).
      data_[i] = -std::max(inside[i], d1);
    }
  }
}

void VoxelDistanceField::Build(const scene::PlanningScene* scene,
                               double resolution, double padding,
                               double max_distance) {
  data_.clear();
  nx_ = ny_ = nz_ = 0;
  resolution_ = std::max(1e-3, resolution);
  max_distance_ = std::max(resolution_, max_distance);
  if (!scene) {
    return;
  }

  const auto objs = scene->GetCollisionObjects();
  const auto occ = scene->OccupiedPoints();
  if (objs.empty() && occ.empty()) {
    return;
  }

  double xmin = 1e9, ymin = 1e9, zmin = 1e9;
  double xmax = -1e9, ymax = -1e9, zmax = -1e9;
  auto expand = [&](double x, double y, double z, double r) {
    xmin = std::min(xmin, x - r);
    ymin = std::min(ymin, y - r);
    zmin = std::min(zmin, z - r);
    xmax = std::max(xmax, x + r);
    ymax = std::max(ymax, y + r);
    zmax = std::max(zmax, z + r);
  };
  for (const auto& o : objs) {
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    scene::GetPrimitiveSizes(o, &sx, &sy, &sz);
    const auto pose = scene::GetObjectPose(o);
    const double r =
        0.5 * std::max({sx, sy, sz, 0.05}) + padding;
    expand(pose.position().x(), pose.position().y(), pose.position().z(), r);
  }
  const double ores = std::max(1e-3, scene->OccupancyResolution());
  for (const auto& p : occ) {
    expand(p.x, p.y, p.z, ores + padding);
  }
  if (xmax < xmin) {
    return;
  }

  ox_ = xmin;
  oy_ = ymin;
  oz_ = zmin;
  nx_ = std::max(1, static_cast<int>(std::ceil((xmax - xmin) / resolution_)));
  ny_ = std::max(1, static_cast<int>(std::ceil((ymax - ymin) / resolution_)));
  nz_ = std::max(1, static_cast<int>(std::ceil((zmax - zmin) / resolution_)));
  // Cap grid size for production memory.
  constexpr int kMaxDim = 160;
  if (nx_ > kMaxDim || ny_ > kMaxDim || nz_ > kMaxDim) {
    const double sx = (xmax - xmin) / kMaxDim;
    const double sy = (ymax - ymin) / kMaxDim;
    const double sz = (zmax - zmin) / kMaxDim;
    resolution_ = std::max({sx, sy, sz, resolution_});
    nx_ = std::max(1, static_cast<int>(std::ceil((xmax - xmin) / resolution_)));
    ny_ = std::max(1, static_cast<int>(std::ceil((ymax - ymin) / resolution_)));
    nz_ = std::max(1, static_cast<int>(std::ceil((zmax - zmin) / resolution_)));
  }

  data_.assign(static_cast<std::size_t>(nx_ * ny_ * nz_), 1.0f);

  for (const auto& o : objs) {
    using SP = automsgs::msgs::shape_msgs::SolidPrimitive;
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    scene::GetPrimitiveSizes(o, &sx, &sy, &sz);
    const auto pose = scene::GetObjectPose(o);
    if (scene::GetPrimitiveType(o) == SP::SPHERE) {
      RasterizeSphere(pose.position().x(), pose.position().y(),
                      pose.position().z(), sx);
    } else {
      RasterizeBox(pose.position().x(), pose.position().y(), pose.position().z(),
                   std::max(1e-3, sx), std::max(1e-3, sy), std::max(1e-3, sz));
    }
    // Seed also via analytic inside for mesh AABB already covered by box.
    (void)PrimitiveSdf;
  }
  for (const auto& p : occ) {
    RasterizeSphere(p.x, p.y, p.z, 0.5 * ores);
  }
  ChamferTransform();
}

double VoxelDistanceField::Distance(double x, double y, double z) const {
  if (empty()) {
    return max_distance_;
  }
  int ix, iy, iz;
  WorldToIndex(x, y, z, &ix, &iy, &iz);
  if (!InBounds(ix, iy, iz)) {
    return max_distance_;
  }
  return static_cast<double>(data_[static_cast<std::size_t>(Index(ix, iy, iz))]);
}

void VoxelDistanceField::Gradient(double x, double y, double z, double* gx,
                                  double* gy, double* gz) const {
  const double h = resolution_;
  *gx = (Distance(x + h, y, z) - Distance(x - h, y, z)) / (2.0 * h);
  *gy = (Distance(x, y + h, z) - Distance(x, y - h, z)) / (2.0 * h);
  *gz = (Distance(x, y, z + h) - Distance(x, y, z - h)) / (2.0 * h);
}

}  // namespace planner
}  // namespace manipulation
}  // namespace autonomy
