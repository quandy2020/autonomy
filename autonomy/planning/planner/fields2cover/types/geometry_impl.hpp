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


#include <cpl_conv.h>
#include <geos_c.h>
#include <memory>
#include <vector>
#include <string>


namespace autonomy::planning::planner::fields2cover::types {

template <class T, OGRwkbGeometryType R>
Geometry<T, R>::Geometry() : data_(
  DownCast<T*>(OGRGeometryFactory::createGeometry(R)),
  [](T* f) {OGRGeometryFactory::destroyGeometry(f);}) {}

template <class T, OGRwkbGeometryType R>
Geometry<T, R>::Geometry(const T& g) : data_(DownCast<T*>(g.clone()),
  [](T* f) {OGRGeometryFactory::destroyGeometry(f);}) {}

template <class T, OGRwkbGeometryType R>
Geometry<T, R>::Geometry(std::shared_ptr<T> g) : data_(g) {}

template <class T, OGRwkbGeometryType R>
Geometry<T, R>::Geometry(T* g, EmptyDestructor) : data_(g, [](T* f) {}) {}

template <class T, OGRwkbGeometryType R>
Geometry<T, R>::Geometry(const T* g) : data_(
  DownCast<T*>(g ? g->clone() : OGRGeometryFactory::createGeometry(R)),
  [](T* f) {OGRGeometryFactory::destroyGeometry(f);}) {}

template <class T, OGRwkbGeometryType R>
Geometry<T, R>::Geometry(OGRGeometry* g, EmptyDestructor) :
  data_(DownCast<T*>(g), [](T* f) {}) {}

template <class T, OGRwkbGeometryType R>
Geometry<T, R>::Geometry(const OGRGeometry* g) : data_(
  DownCast<T*>(g ? g->clone() : OGRGeometryFactory::createGeometry(R)),
  [](T* f) {OGRGeometryFactory::destroyGeometry(f);}) {}

template <class T, OGRwkbGeometryType R>
Geometry<T, R>::~Geometry() = default;

template <class T, OGRwkbGeometryType R>
Geometry<T, R>::Geometry(const Geometry& g) = default;

template <class T, OGRwkbGeometryType R>
Geometry<T, R>::Geometry(Geometry&& g) = default;

template <class T, OGRwkbGeometryType R>
Geometry<T, R>& Geometry<T, R>::operator=(
    Geometry&& g) = default;

template <class T, OGRwkbGeometryType R>
Geometry<T, R>& Geometry<T, R>::operator=(
    const Geometry& g) = default;

template <class T, OGRwkbGeometryType R>
std::shared_ptr<T> Geometry<T, R>::operator->() {return data_;}

template <class T, OGRwkbGeometryType R>
std::shared_ptr<const T> Geometry<T, R>::operator->() const {return data_;}

template <class T, OGRwkbGeometryType R>
T* Geometry<T, R>::get() {return data_.get();}

template <class T, OGRwkbGeometryType R>
const T* Geometry<T, R>::get() const {return data_.get();}

template <class T, OGRwkbGeometryType R>
bool Geometry<T, R>::operator!=(const Geometry<T, R>& geom2) const {
  return !geom2->Equals(data_.get());
}

template <class T, OGRwkbGeometryType R>
bool Geometry<T, R>::operator==(const Geometry<T, R>& geom2) const {
  return geom2->Equals(data_.get());
}


template <class T, OGRwkbGeometryType R>
double Geometry<T, R>::GetDimMinX() const {
  OGREnvelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MinX;
}

template <class T, OGRwkbGeometryType R>
double Geometry<T, R>::GetDimMaxX() const {
  OGREnvelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MaxX;
}

template <class T, OGRwkbGeometryType R>
double Geometry<T, R>::GetDimMinY() const {
  OGREnvelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MinY;
}

template <class T, OGRwkbGeometryType R>
double Geometry<T, R>::GetDimMaxY() const {
  OGREnvelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MaxY;
}

template <class T, OGRwkbGeometryType R>
double Geometry<T, R>::GetHeight() const {
  OGREnvelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MaxY - envelope.MinY;
}

template <class T, OGRwkbGeometryType R>
double Geometry<T, R>::GetWidth() const {
  OGREnvelope envelope;
  data_->getEnvelope(&envelope);
  return envelope.MaxX - envelope.MinX;
}

template <class T, OGRwkbGeometryType R>
double Geometry<T, R>::GetMinSafeLength() const {
  OGREnvelope envelope;
  data_->getEnvelope(&envelope);
  return (envelope.MaxX - envelope.MinX) +
    (envelope.MaxY - envelope.MinY);
}

template <class T, OGRwkbGeometryType R>
template <class T2, OGRwkbGeometryType R2>
double Geometry<T, R>::Distance(const Geometry<T2, R2>& p) const {
  return data_->Distance(p.get());
}

template <class T, OGRwkbGeometryType R>
template <class T2, OGRwkbGeometryType R2>
bool Geometry<T, R>::disjoint(const Geometry<T2, R2>& geom) const {
  return data_->Disjoint(geom.get());
}

template <class T, OGRwkbGeometryType R>
template <class T2, OGRwkbGeometryType R2>
bool Geometry<T, R>::crosses(const Geometry<T2, R2>& geom) const {
  return data_->Crosses(geom.get());
}

template <class T, OGRwkbGeometryType R>
template <class T2, OGRwkbGeometryType R2>
bool Geometry<T, R>::touches(const Geometry<T2, R2>& geom) const {
  return data_->Touches(geom.get());
}

template <class T, OGRwkbGeometryType R>
template <class T2, OGRwkbGeometryType R2>
bool Geometry<T, R>::within(const Geometry<T2, R2>& geom) const {
  return data_->Within(geom.get());
}

template <class T, OGRwkbGeometryType R>
template <class T2, OGRwkbGeometryType R2>
bool Geometry<T, R>::contains(const Geometry<T2, R2>& geom) const {
  return data_->Contains(geom.get());
}

template <class T, OGRwkbGeometryType R>
template <class T2, OGRwkbGeometryType R2>
bool Geometry<T, R>::intersects(const Geometry<T2, R2>& geom) const {
  return data_->Intersects(geom.get());
}

template <class T, OGRwkbGeometryType R>
double Geometry<T, R>::Mod2pi(double val) {
  return autonomy::common::math::WrapAngle(val);
}

template <class T, OGRwkbGeometryType R>
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

template <class T, OGRwkbGeometryType R>
std::vector<double> Geometry<T, R>::GetAngContinuity(
    const std::vector<double>& val) {
  std::vector<double> res {val[0]};
  for (size_t i = 1; i < val.size(); ++i) {
    res.emplace_back(GetAngContinuity(res[i - 1], val[i]));
  }
  return res;
}

template <class T, OGRwkbGeometryType R>
double Geometry<T, R>::GetAngleDiffAbs(double a, double b) {
  const auto pi = boost::math::constants::pi<double>();
  return pi - fabs(Mod2pi(b - a) - pi);
}

template <class T, OGRwkbGeometryType R>
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

template <class T, OGRwkbGeometryType R>
bool Geometry<T, R>::IsEmpty() const {
  return (!data_ || data_->IsEmpty());
}

template <class T, OGRwkbGeometryType R>
std::string Geometry<T, R>::ExportToWkt() const {
  char *PszWKT = nullptr;
  data_->exportToWkt(&PszWKT);
  std::string result{PszWKT};
  CPLFree(PszWKT);
  return result;
}

template <class T, OGRwkbGeometryType R>
std::string Geometry<T, R>::ExportToGML() const {
  char *s = data_->exportToGML();
  std::string str(s);
  CPLFree(s);
  return str;
}

template <class T, OGRwkbGeometryType R>
std::string Geometry<T, R>::ExportToKML() const {
  char *s = data_->exportToKML();
  std::string str(s);
  CPLFree(s);
  return str;
}

template <class T, OGRwkbGeometryType R>
std::string Geometry<T, R>::ExportToJson() const {
  char *s = data_->exportToJson();
  std::string str(s);
  CPLFree(s);
  return str;
}

template <class T, OGRwkbGeometryType R>
void Geometry<T, R>::ImportFromWkt(const std::string& text) {
  auto char_text = text.c_str();
  data_->importFromWkt(&char_text);
}

// ###############################
// Code extracted from:
// https://github.com/OSGeo/gdal/blob/b0aa6065a39b252cb8306e9c2e2535d6dda0fb55/port/cpl_conv.h#L397
template <class T, OGRwkbGeometryType R>
template <typename To, typename From>
inline To Geometry<T, R>::DownCast(From *f) const {
  static_assert(
    (std::is_base_of<From, typename std::remove_pointer<To>::type>::value),
    "target type not derived from source type");
  CPLAssert(f == nullptr || dynamic_cast<To>(f) != nullptr);
  return static_cast<To>(f);
}
// ###############################

template <class T, OGRwkbGeometryType R>
template<typename RetType>
RetType Geometry<T, R>::DestroyResGeom(OGRGeometry* g) {
  RetType res(g);
  OGRGeometryFactory::destroyGeometry(g);
  return res;
}

// ###############################
// Code extracted from:
// https://github.com/OSGeo/gdal/blob/717dcc0eed252e2f78c142b1f7866e49c5511224/ogr/ogrgeometry.cpp#L4309
template <class T, OGRwkbGeometryType R>
OGRGeometry* Geometry<T, R>::OGRBuffer(double DfDist, int side) const {
  OGRGeometry *PoOGRProduct = nullptr;

  GEOSContextHandle_t HGEOSCtxt = OGRGeometry::createGEOSContext();
  GEOSGeom HGeosGeom = this->data_->exportToGEOS(HGEOSCtxt);
  if (HGeosGeom != nullptr) {
    GEOSBufferParams* HBufParams = GEOSBufferParams_create_r(HGEOSCtxt);
    GEOSBufferParams_setEndCapStyle_r(HGEOSCtxt, HBufParams,
        GEOSBufCapStyles::GEOSBUF_CAP_FLAT);
    GEOSBufferParams_setJoinStyle_r(HGEOSCtxt, HBufParams,
        GEOSBufJoinStyles::GEOSBUF_JOIN_MITRE);
    GEOSBufferParams_setSingleSided_r(HGEOSCtxt, HBufParams, side != 0);

    GEOSGeom HGeosProduct = GEOSBufferWithParams_r(HGEOSCtxt, HGeosGeom,
        HBufParams, side >= 0 ? DfDist : -DfDist);
    GEOSGeom_destroy_r(HGEOSCtxt, HGeosGeom);
    GEOSBufferParams_destroy_r(HGEOSCtxt, HBufParams);

    PoOGRProduct = BuildGeometryFromGEOS(HGEOSCtxt, HGeosProduct,
        this->data_.get(), nullptr);
  }
  OGRGeometry::freeGEOSContext(HGEOSCtxt);
  return PoOGRProduct;
}
// ---------------------------------------------------------------------------

// ###############################
// Code extracted from:
// https://github.com/OSGeo/gdal/blob/717dcc0eed252e2f78c142b1f7866e49c5511224/ogr/ogrgeometry.cpp#L4309
template <class T, OGRwkbGeometryType R>
OGRGeometry* Geometry<T, R>::OGRGeometryRebuildCurves(
    const OGRGeometry *PoGeom, const OGRGeometry *PoOtherGeom,
    OGRGeometry *PoOGRProduct) const {
  if (PoOGRProduct != nullptr &&
      wkbFlatten(PoOGRProduct->getGeometryType()) != wkbPoint &&
      (PoGeom->hasCurveGeometry(true) ||
      (PoOtherGeom && PoOtherGeom->hasCurveGeometry(true)))) {
    OGRGeometry *PoCurveGeom = PoOGRProduct->getCurveGeometry();
    delete PoOGRProduct;
    return PoCurveGeom;
  }
  return PoOGRProduct;
}
template <class T, OGRwkbGeometryType R>
OGRGeometry* Geometry<T, R>::BuildGeometryFromGEOS(
    GEOSContextHandle_t HGEOSCtxt, GEOSGeom HGeosProduct,
    const OGRGeometry *PoSelf, const OGRGeometry *PoOtherGeom) const {
  OGRGeometry *PoOGRProduct = nullptr;
  if (HGeosProduct != nullptr) {
    PoOGRProduct = OGRGeometryFactory::createFromGEOS(HGEOSCtxt, HGeosProduct);
    if (PoOGRProduct != nullptr &&
        PoSelf->getSpatialReference() != nullptr &&
        (PoOtherGeom == nullptr ||
          (PoOtherGeom->getSpatialReference() != nullptr &&
        PoOtherGeom->getSpatialReference()->IsSame(
          PoSelf->getSpatialReference())))) {
      PoOGRProduct->assignSpatialReference(PoSelf->getSpatialReference());
    }
    PoOGRProduct = OGRGeometryRebuildCurves(PoSelf, PoOtherGeom, PoOGRProduct);
    GEOSGeom_destroy_r(HGEOSCtxt, HGeosProduct);
  }
  return PoOGRProduct;
}
// ###############################


template <class T, OGRwkbGeometryType R>
double Geometry<T, R>::mod(double a, double b) {
  return fmod(fmod(a, b) + b, b);
}


}  // namespace autonomy::planning::planner::fields2cover::types

