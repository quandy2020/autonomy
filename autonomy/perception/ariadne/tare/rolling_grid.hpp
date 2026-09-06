/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <cstddef>
#include <vector>

namespace autonomy::perception::exploration {

// Rolling 3D index grid (TARE RollingGrid, header-only core).
class RollingGrid {
 public:
  void Resize(int nx, int ny, int nz);
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nz() const { return nz_; }

  int SubToInd(int ix, int iy, int iz) const;
  void IndToSub(int ind, int* ix, int* iy, int* iz) const;

  void SetOriginCells(int ox, int oy, int oz);
  int origin_x() const { return origin_x_; }
  int origin_y() const { return origin_y_; }
  int origin_z() const { return origin_z_; }

  void Roll(int dx, int dy, int dz, std::vector<int>* rolled_in,
            std::vector<int>* rolled_out);

 private:
  int nx_{0};
  int ny_{0};
  int nz_{0};
  int origin_x_{0};
  int origin_y_{0};
  int origin_z_{0};
};

}  // namespace autonomy::perception::exploration
