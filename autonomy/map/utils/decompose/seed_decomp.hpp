/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include "autonomy/map/utils/decompose/decomp_base.hpp"

namespace autonomy {
namespace map {
namespace utils {

/**
 * @brief Seed Decomp Class
 *
 * Dilate around the given point
 */
template <int Dim>
class SeedDecomp : public DecompBase<Dim> {
  public:
    ///Simple constructor
    SeedDecomp() {};
    /**
     * @brief Basic constructor
     * @param p1 One end of the line seg
     * @param p2 The other end of the line seg
     */
    SeedDecomp(const Vecf<Dim> &p) : p_(p) {}
    /**
     * @brief Inflate the seed with a sphere
     * @param radius Robot radius
     */
    void dilate(decimal_t radius) {
      this->ellipsoid_ = Ellipsoid<Dim>(radius * Matf<Dim, Dim>::Identity(), p_);
      this->find_polyhedron();
      add_local_bbox(this->polyhedron_);
    }

    /// Get the center
    Vecf<Dim> get_seed() const {
      return p_;
    }

  protected:
    ///Add the bounding box
    void add_local_bbox(Polyhedron<Dim> &Vs) {
      if(this->local_bbox_.norm() == 0)
        return;

      //**** virtual walls x-y-z
      Vecf<Dim> dir = Vecf<Dim>::UnitX();
      Vecf<Dim> dir_h = Vecf<Dim>::UnitY();

      Vecf<Dim> pp1 = p_ + dir_h * this->local_bbox_(1);
      Vecf<Dim> pp2 = p_ - dir_h * this->local_bbox_(1);
      Vs.add(Hyperplane<Dim>(pp1, dir_h));
      Vs.add(Hyperplane<Dim>(pp2, -dir_h));

      // along y
      Vecf<Dim> pp3 = p_ + dir * this->local_bbox_(0);
      Vecf<Dim> pp4 = p_ - dir * this->local_bbox_(0);
      Vs.add(Hyperplane<Dim>(pp3, dir));
      Vs.add(Hyperplane<Dim>(pp4, -dir));

      // along z
      if(Dim > 2) {
        Vecf<Dim> dir_v = Vecf<Dim>::UnitZ();
        Vecf<Dim> pp5 = p_ + dir_v * this->local_bbox_(2);
        Vecf<Dim> pp6 = p_ - dir_v * this->local_bbox_(2);
        Vs.add(Hyperplane<Dim>(pp5, dir_v));
        Vs.add(Hyperplane<Dim>(pp6, -dir_v));
      }
    }

    ///Seed location
    Vecf<Dim> p_;
};

typedef SeedDecomp<2> SeedDecomp2D;

typedef SeedDecomp<3> SeedDecomp3D;

}  // namespace utils
}  // namespace map
}  // namespace autonomy


