/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file geometries.hpp
 * @brief Fields2Cover Geometries.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <memory>
#include <type_traits>
#include <iterator>
#include <cstddef>
#include "autonomy/planning/planner/fields2cover/types/geometry.hpp"

namespace autonomy::planning::planner::fields2cover::types {

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
struct Geometries : public Geometry<T, R> {
 public:
  // Hidden from SWIG, which would also wrap the base copy/move constructors.
#ifndef SWIG
  using Geometry<T, R>::Geometry;
#endif
  /**
   * @brief Compute Area of the geometry.
   */
  double Area() const;
  SAMETYPE Clone() const;
  SAMETYPE simplify(double d_tol) const;

  #ifndef SWIG
  // Code adapted from OGRCurve::Iterator
  class Iterator {
    struct Private {
      Private();
      ~Private();
      Private(const Private &) = delete;
      Private &operator=(const Private&) = delete;

      bool MBUpdateChecked = true;
      CHILDRENTYPE MOChild{};
      Geometries<SAMETYPE, T, R, CHILDRENTYPE>* MPoSelf = nullptr;
      int MNPos = 0;
    };
    std::unique_ptr<Private> MPoPrivate;

    void update();

   public:
    Iterator(Geometries<SAMETYPE, T, R, CHILDRENTYPE>* PoSelf, int NPos);
    ~Iterator();

    CHILDRENTYPE& operator*();

    Iterator& operator++();

    bool operator!=(const Iterator& it) const;
  };

  // Code adapted from OGRCurve::ConstIterator
  class ConstIterator {
    struct Private {
      Private();
      Private(const Private &) = delete;
      Private &operator=(const Private&) = delete;


      CHILDRENTYPE MOChild{};
      const Geometries<SAMETYPE, T, R, CHILDRENTYPE>* MPoSelf = nullptr;
      int MNPos = 0;
    };
    std::unique_ptr<Private> MPoPrivate;

   public:
    ConstIterator(ConstIterator& OOther);
    ConstIterator(const Geometries<SAMETYPE, T, R, CHILDRENTYPE>* PoSelf,
        int NPos);

    ~ConstIterator();

    const CHILDRENTYPE& operator*() const;

    ConstIterator& operator++();

    bool operator!=(const ConstIterator& it) const;

    bool operator==(const Iterator& it) const;
  };

  Iterator begin();
  Iterator end();
  ConstIterator begin() const;
  ConstIterator end() const;
  ConstIterator begin(const SAMETYPE* PoSelf);
  ConstIterator end(const SAMETYPE* PoSelf);

  const CHILDRENTYPE operator[](int i) const;
  const CHILDRENTYPE at(int i) const;
  const CHILDRENTYPE back() const;

  SAMETYPE& Append(const SAMETYPE& other);

  #endif
};

}  // namespace autonomy::planning::planner::fields2cover::types

#include "autonomy/planning/planner/fields2cover/types/geometries_impl.hpp"


