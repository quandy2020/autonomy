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
 * @file geometry_impl.hpp
 * @brief Fields2Cover Geometry impl.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */


#include <cstdlib>
#include <memory>
#include <vector>
#include <string>


namespace autonomy::planning::planner::fields2cover::types {

template <class T, WkbKind R>
Geometry<T, R>::Geometry() : data_(
  DownCast<T*>(GeomFactory::createGeometry(R)),
  [](T* f) {GeomFactory::destroyGeometry(f);}) {}

template <class T, WkbKind R>
Geometry<T, R>::Geometry(const T& g) : data_(DownCast<T*>(g.clone()),
  [](T* f) {GeomFactory::destroyGeometry(f);}) {}

template <class T, WkbKind R>
Geometry<T, R>::Geometry(std::shared_ptr<T> g) : data_(g) {}

template <class T, WkbKind R>
Geometry<T, R>::Geometry(T* g, EmptyDestructor) : data_(g, [](T* f) {}) {}

template <class T, WkbKind R>
Geometry<T, R>::Geometry(const T* g) : data_(
  DownCast<T*>(g ? g->clone() : GeomFactory::createGeometry(R)),
  [](T* f) {GeomFactory::destroyGeometry(f);}) {}

template <class T, WkbKind R>
Geometry<T, R>::~Geometry() = default;

template <class T, WkbKind R>
Geometry<T, R>::Geometry(const Geometry& g) = default;

template <class T, WkbKind R>
Geometry<T, R>::Geometry(Geometry&& g) = default;

template <class T, WkbKind R>
Geometry<T, R>& Geometry<T, R>::operator=(
    Geometry&& g) = default;

template <class T, WkbKind R>
Geometry<T, R>& Geometry<T, R>::operator=(
    const Geometry& g) = default;

template <class T, WkbKind R>
std::shared_ptr<T> Geometry<T, R>::operator->() {return data_;}

template <class T, WkbKind R>
std::shared_ptr<const T> Geometry<T, R>::operator->() const {return data_;}

template <class T, WkbKind R>
T* Geometry<T, R>::get() {return data_.get();}

template <class T, WkbKind R>
const T* Geometry<T, R>::get() const {return data_.get();}

template <class T, WkbKind R>
bool Geometry<T, R>::operator!=(const Geometry<T, R>& geom2) const {
  return !geom2->Equals(data_.get());
}

template <class T, WkbKind R>
bool Geometry<T, R>::operator==(const Geometry<T, R>& geom2) const {
  return geom2->Equals(data_.get());
}


template <class T, WkbKind R>
double Geometry<T, R>::GetDimMinX() const {
  Envelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MinX;
}

template <class T, WkbKind R>
double Geometry<T, R>::GetDimMaxX() const {
  Envelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MaxX;
}

template <class T, WkbKind R>
double Geometry<T, R>::GetDimMinY() const {
  Envelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MinY;
}

template <class T, WkbKind R>
double Geometry<T, R>::GetDimMaxY() const {
  Envelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MaxY;
}

template <class T, WkbKind R>
double Geometry<T, R>::GetHeight() const {
  Envelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MaxY - envelope.MinY;
}

template <class T, WkbKind R>
double Geometry<T, R>::GetWidth() const {
  Envelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MaxX - envelope.MinX;
}

template <class T, WkbKind R>
double Geometry<T, R>::GetMinSafeLength() const {
  Envelope envelope;
  data_->getEnvelope(&envelope);
  return (envelope.MaxX - envelope.MinX) +
    (envelope.MaxY - envelope.MinY);
}

template <class T, WkbKind R>
template <class T2, WkbKind R2>
double Geometry<T, R>::Distance(const Geometry<T2, R2>& p) const {
  return data_->Distance(p.get());
}

template <class T, WkbKind R>
template <class T2, WkbKind R2>
bool Geometry<T, R>::disjoint(const Geometry<T2, R2>& geom) const {
  return data_->Disjoint(geom.get());
}

template <class T, WkbKind R>
template <class T2, WkbKind R2>
bool Geometry<T, R>::crosses(const Geometry<T2, R2>& geom) const {
  return data_->Crosses(geom.get());
}

template <class T, WkbKind R>
template <class T2, WkbKind R2>
bool Geometry<T, R>::touches(const Geometry<T2, R2>& geom) const {
  return data_->Touches(geom.get());
}

template <class T, WkbKind R>
template <class T2, WkbKind R2>
bool Geometry<T, R>::within(const Geometry<T2, R2>& geom) const {
  return data_->Within(geom.get());
}

template <class T, WkbKind R>
template <class T2, WkbKind R2>
bool Geometry<T, R>::contains(const Geometry<T2, R2>& geom) const {
  return data_->Contains(geom.get());
}

template <class T, WkbKind R>
template <class T2, WkbKind R2>
bool Geometry<T, R>::intersects(const Geometry<T2, R2>& geom) const {
  return data_->Intersects(geom.get());
}

template <class T, WkbKind R>
double Geometry<T, R>::Mod2pi(double val) {
  return autonomy::common::math::WrapAngle(val);
}

template <class T, WkbKind R>
double Geometry<T, R>::GetAngContinuity(double prev_val, double val) {
  const auto two_pi = boost::math::constants::two_pi<double>();
  auto val_id = round(prev_val / two_pi);
  auto GetNearVal =
    [two_pi] (int i, double value) {return i * two_pi + Mod2pi(value);};
  for (int i = -2; i <= 0; ++i) {
    if (fabs(GetNearVal(val_id + i + 1, val) - prev_val) >
      fabs(GetNearVal(val_id + i, val) - prev_val)) {
      return GetNearVal(val_id + i, val);
    }
  }
  return GetNearVal(val_id + 1, val);
}

template <class T, WkbKind R>
std::vector<double> Geometry<T, R>::GetAngContinuity(
    const std::vector<double>& val) {
  std::vector<double> res {val[0]};
  for (size_t i = 1; i < val.size(); ++i) {
    res.emplace_back(GetAngContinuity(res[i - 1], val[i]));
  }
  return res;
}

template <class T, WkbKind R>
double Geometry<T, R>::GetAngleDiffAbs(double a, double b) {
  const auto pi = boost::math::constants::pi<double>();
  return pi - fabs(Mod2pi(b - a) - pi);
}

template <class T, WkbKind R>
double Geometry<T, R>::GetAngleAvg(double a, double b) {
  const auto pi = boost::math::constants::pi<double>();
  double ma = Mod2pi(a);
  double mb = Mod2pi(b);
  if (abs(mb - ma) < pi) {
    return 0.5 * (ma + mb);
  } else {
    return 0.5 * (ma + mb) + pi;
  }
}

template <class T, WkbKind R>
bool Geometry<T, R>::IsEmpty() const {
  return (!data_ || data_->IsEmpty());
}

template <class T, WkbKind R>
std::string Geometry<T, R>::ExportToWkt() const {
  char *PszWKT = nullptr;
  data_->exportToWkt(&PszWKT);
  std::string result{PszWKT ? PszWKT : ""};
  std::free(PszWKT);
  return result;
}

template <class T, WkbKind R>
std::string Geometry<T, R>::ExportToGML() const {
  char *s = data_->exportToGML();
  std::string str(s ? s : "");
  std::free(s);
  return str;
}

template <class T, WkbKind R>
std::string Geometry<T, R>::ExportToKML() const {
  char *s = data_->exportToKML();
  std::string str(s ? s : "");
  std::free(s);
  return str;
}

template <class T, WkbKind R>
std::string Geometry<T, R>::ExportToJson() const {
  char *s = data_->exportToJson();
  std::string str(s ? s : "");
  std::free(s);
  return str;
}

template <class T, WkbKind R>
void Geometry<T, R>::ImportFromWkt(const std::string& text) {
  auto char_text = text.c_str();
  data_->importFromWkt(&char_text);
}

template <class T, WkbKind R>
template <typename To, typename From>
inline To Geometry<T, R>::DownCast(From *f) const {
  return static_cast<To>(f);
}

template <class T, WkbKind R>
template<typename RetType>
RetType Geometry<T, R>::DestroyResGeom(NativeGeom* g) {
  RetType res(g);
  GeomFactory::destroyGeometry(g);
  return res;
}

template <class T, WkbKind R>
NativeGeom* Geometry<T, R>::BufferNative(double DfDist, int side) const {
  return data_->Buffer(DfDist, side);
}

template <class T, WkbKind R>
double Geometry<T, R>::mod(double a, double b) {
  return fmod(fmod(a, b) + b, b);
}


}  // namespace autonomy::planning::planner::fields2cover::types
