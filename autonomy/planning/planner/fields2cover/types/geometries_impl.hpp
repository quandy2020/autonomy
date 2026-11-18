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

#include <type_traits>

/**
 * @file geometries_impl.hpp
 * @brief Fields2Cover Geometries impl.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


namespace autonomy::planning::planner::fields2cover::types {

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
double Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Area() const {
  return (this->data_ ? this->data_->get_Area() : 0.0);
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
SAMETYPE Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Clone() const {
  SAMETYPE cloned;
  cloned.ImportFromWkt(this->ExportToWkt());
  return cloned;
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
SAMETYPE Geometries<SAMETYPE, T, R, CHILDRENTYPE>::simplify(
    double d_tol) const {
  NativeGeom* s = this->data_->SimplifyPreserveTopology(d_tol);
  SAMETYPE f2c_s(s);
  GeomFactory::destroyGeometry(s);
  return f2c_s;
}


template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Iterator::Private::Private()
  = default;

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Iterator::Private::~Private() {}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
void Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Iterator::update() {
  if (R == WkbKind::kMultiPoint ||
      R == WkbKind::kLinearRing ||
      R == WkbKind::kLineString) {
    if (!MPoPrivate->MBUpdateChecked) {
      CHILDRENTYPE OChildBefore;
      static_cast<SAMETYPE*>(MPoPrivate->MPoSelf)->GetGeometry(
        MPoPrivate->MNPos, OChildBefore);
      if (OChildBefore != MPoPrivate->MOChild) {
          static_cast<SAMETYPE*>(MPoPrivate->MPoSelf)->SetGeometry(
             MPoPrivate->MNPos, MPoPrivate->MOChild);
      }
      MPoPrivate->MBUpdateChecked = true;
    }
  }
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Iterator::Iterator(
    Geometries<SAMETYPE, T, R, CHILDRENTYPE>* PoSelf, int NPos) :
      MPoPrivate(new Private()) {
  MPoPrivate->MPoSelf = PoSelf;
  MPoPrivate->MNPos = NPos;
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Iterator::~Iterator() {
  update();
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
CHILDRENTYPE& Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Iterator::operator*() {
  update();
  static_cast<SAMETYPE*>(MPoPrivate->MPoSelf)->GetGeometry(
      MPoPrivate->MNPos, MPoPrivate->MOChild);
  MPoPrivate->MBUpdateChecked = false;
  return MPoPrivate->MOChild;
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
typename Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Iterator&
    Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Iterator::operator++() {
  update();
  ++MPoPrivate->MNPos;
  return *this;
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
bool Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Iterator::operator!=(
    const Iterator& it) const {
  return MPoPrivate->MNPos != it.MPoPrivate->MNPos;
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator::Private::Private()
  = default;


template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator::ConstIterator(
    ConstIterator& OOther) = default;

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator::ConstIterator(
    const Geometries<SAMETYPE, T, R, CHILDRENTYPE>* PoSelf,
        int NPos) : MPoPrivate(new Private()) {
  MPoPrivate->MPoSelf = PoSelf;
  MPoPrivate->MNPos = NPos;
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator::~ConstIterator()
  = default;

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
const CHILDRENTYPE& Geometries<SAMETYPE, T, R, CHILDRENTYPE>
        ::ConstIterator::operator*() const {
  static_cast<const SAMETYPE*>(
    MPoPrivate->MPoSelf)->GetGeometry(MPoPrivate->MNPos,
      MPoPrivate->MOChild);
  return MPoPrivate->MOChild;
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
typename Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator&
    Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator::operator++() {
  ++MPoPrivate->MNPos;
  return *this;
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
bool Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator::operator!=(
    const ConstIterator& it) const {
  return MPoPrivate->MNPos != it.MPoPrivate->MNPos;
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
bool Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator::operator==(
    const Iterator& it) const {
  return MPoPrivate->MNPos == it.MPoPrivate->MNPos;
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
typename Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Iterator
    Geometries<SAMETYPE, T, R, CHILDRENTYPE>::begin() {
  return {this, 0};
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
typename Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Iterator
    Geometries<SAMETYPE, T, R, CHILDRENTYPE>::end() { return {this,
  static_cast<int>(static_cast<SAMETYPE*>(this)->size())};
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
typename Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator
    Geometries<SAMETYPE, T, R, CHILDRENTYPE>::begin() const {
  return {this, 0};
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
typename Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator
    Geometries<SAMETYPE, T, R, CHILDRENTYPE>::end() const {
  return {this, static_cast<int>(static_cast<SAMETYPE const*>(this)->size())};
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
typename Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator
    Geometries<SAMETYPE, T, R, CHILDRENTYPE>::begin(const SAMETYPE* PoSelf) {
  return PoSelf->begin();
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
typename Geometries<SAMETYPE, T, R, CHILDRENTYPE>::ConstIterator
    Geometries<SAMETYPE, T, R, CHILDRENTYPE>::end(const SAMETYPE* PoSelf) {
  return PoSelf->end();
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
const CHILDRENTYPE Geometries<SAMETYPE, T, R, CHILDRENTYPE>::operator[](
    int i) const {
  return static_cast<SAMETYPE const*>(this)->GetGeometry(i);
}
template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
const CHILDRENTYPE Geometries<SAMETYPE, T, R, CHILDRENTYPE>::at(int i) const {
  return static_cast<SAMETYPE const*>(this)->GetGeometry(i);
}
template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
const CHILDRENTYPE Geometries<SAMETYPE, T, R, CHILDRENTYPE>::back() const {
  size_t N = static_cast<SAMETYPE const*>(this)->size();
  return static_cast<SAMETYPE const*>(this)->GetGeometry(N-1);
}

template <class SAMETYPE, class T, WkbKind R, class CHILDRENTYPE>
SAMETYPE& Geometries<SAMETYPE, T, R, CHILDRENTYPE>::Append(
    const SAMETYPE& other) {
  this->insert(this->end(), other.begin(), other.end());
  return *this;
}


}  // namespace autonomy::planning::planner::fields2cover::types

