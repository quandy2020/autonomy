/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/perception/exploration/tare/rolling_grid.hpp"

#include <algorithm>
#include <unordered_set>

namespace autonomy::perception::exploration {

void RollingGrid::Resize(int nx, int ny, int nz) {
  nx_ = std::max(nx, 1);
  ny_ = std::max(ny, 1);
  nz_ = std::max(nz, 1);
}

int RollingGrid::SubToInd(int ix, int iy, int iz) const {
  const int lx = ((ix % nx_) + nx_) % nx_;
  const int ly = ((iy % ny_) + ny_) % ny_;
  const int lz = ((iz % nz_) + nz_) % nz_;
  return (lz * ny_ + ly) * nx_ + lx;
}

void RollingGrid::IndToSub(int ind, int* ix, int* iy, int* iz) const {
  const int cells_xy = nx_ * ny_;
  *iz = ind / cells_xy;
  const int rem = ind % cells_xy;
  *iy = rem / nx_;
  *ix = rem % nx_;
}

void RollingGrid::SetOriginCells(int ox, int oy, int oz) {
  origin_x_ = ox;
  origin_y_ = oy;
  origin_z_ = oz;
}

void RollingGrid::Roll(int dx, int dy, int dz, std::vector<int>* rolled_in,
                       std::vector<int>* rolled_out) {
  if (rolled_in) {
    rolled_in->clear();
  }
  if (rolled_out) {
    rolled_out->clear();
  }
  if (dx == 0 && dy == 0 && dz == 0) {
    return;
  }
  std::unordered_set<int> incoming;
  std::unordered_set<int> outgoing;
  const int new_ox = origin_x_ + dx;
  const int new_oy = origin_y_ + dy;
  const int new_oz = origin_z_ + dz;
  for (int iz = 0; iz < nz_; ++iz) {
    for (int iy = 0; iy < ny_; ++iy) {
      for (int ix = 0; ix < nx_; ++ix) {
        const int wx = new_ox + ix;
        const int wy = new_oy + iy;
        const int wz = new_oz + iz;
        const int old_ix = wx - origin_x_;
        const int old_iy = wy - origin_y_;
        const int old_iz = wz - origin_z_;
        const bool in_old = old_ix >= 0 && old_ix < nx_ && old_iy >= 0 &&
                            old_iy < ny_ && old_iz >= 0 && old_iz < nz_;
        const int new_ind = SubToInd(ix, iy, iz);
        if (!in_old) {
          incoming.insert(new_ind);
        }
        const int old_wx = origin_x_ + ix;
        const int old_wy = origin_y_ + iy;
        const int old_wz = origin_z_ + iz;
        const int new_ix = old_wx - new_ox;
        const int new_iy = old_wy - new_oy;
        const int new_iz = old_wz - new_oz;
        const bool in_new = new_ix >= 0 && new_ix < nx_ && new_iy >= 0 &&
                            new_iy < ny_ && new_iz >= 0 && new_iz < nz_;
        if (!in_new) {
          outgoing.insert(SubToInd(ix, iy, iz));
        }
      }
    }
  }
  origin_x_ = new_ox;
  origin_y_ = new_oy;
  origin_z_ = new_oz;
  if (rolled_in) {
    rolled_in->assign(incoming.begin(), incoming.end());
  }
  if (rolled_out) {
    rolled_out->assign(outgoing.begin(), outgoing.end());
  }
}

}  // namespace autonomy::perception::exploration
