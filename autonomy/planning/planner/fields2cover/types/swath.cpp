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

/**
 * @file swath.cpp
 * @brief Fields2Cover Swath implementation.
 *
 * Ported from Fields2Cover (BSD-3, Wageningen University).
 */



#include "autonomy/planning/planner/fields2cover/types/swath.hpp"

namespace autonomy::planning::planner::fields2cover::types {

Swath::Swath() = default;
Swath::Swath(const LineString& _path) : path_(_path) {}

Swath::Swath(double _width) {
  this->SetWidth(_width);
}

Swath::Swath(const LineString& _path, double _width, int _id, SwathType _type) :
    id_(_id), path_(_path), type_(_type) {
  this->SetWidth(_width);
}

Swath::Swath(const Swath&) = default;
Swath::~Swath() = default;
Swath& Swath::operator=(Swath&&) = default;
Swath& Swath::operator=(const Swath&) = default;

bool Swath::operator!=(const Swath& s) const {
  if (s.GetWidth() != this->GetWidth()) {
    return true;
  }
  Point this_p {this->StartPoint()};
  Point s_p = (this->HasSameDir(s) ? s.StartPoint() : s.EndPoint());
  return (this_p.GetX() != s_p.GetX() || this_p.GetY() != s_p.GetY()) ||
    s.Length() != this->Length();
}

bool Swath::operator==(const Swath& s) const {
  return !(s != *this);
}

bool Swath::operator<(const Swath& s) const {
  return this->GetId() < s.GetId();
}

bool Swath::operator>(const Swath& s) const {
  return this->GetId() > s.GetId();
}

bool Swath::operator>=(const Swath& s) const {
  return (this->GetId() > s.GetId() ||
      (this->GetId() == s.GetId() && *this == s));
}

bool Swath::operator<=(const Swath& s) const {
  return (this->GetId() < s.GetId() ||
      (this->GetId() == s.GetId() && *this == s));
}

double Swath::Length() const {
  return this->path_.Length();
}

double Swath::Area() const {
  return this->AreaCovered().Area();
}

double Swath::Area(const Cells& polys) const {
  return this->AreaCovered(polys).Area();
}

void Swath::Reverse() {
  this->path_.ReversePoints();
  this->creation_dir_ = !this->creation_dir_;
}

double Swath::GetInAngle() const {
  if (this->path_.IsEmpty()) {
    return -1.0;
  }
  return (path_.GetGeometry(1) - path_.GetGeometry(0)).GetAngleFromPoint();
}

double Swath::GetOutAngle() const {
  if (this->path_.IsEmpty()) {
    return -1.0;
  }
  auto n = this->NumPoints() - 1;
  return (path_.GetGeometry(n) - path_.GetGeometry(n-1)).GetAngleFromPoint();
}

Point Swath::StartPoint() const {
  if (this->path_.IsEmpty()) {
    return {};
  }
  return this->path_.StartPoint();
}

Point Swath::EndPoint() const {
  if (this->path_.IsEmpty()) {
    return {};
  }
  return this->path_.EndPoint();
}

Cells Swath::AreaCovered() const {
  if (this->path_.IsEmpty()) {
    return Cells();
  }
  return Cells::buffer(this->path_, 0.5 * this->width_);
}

Cells Swath::AreaCovered(const Cells& polys) const {
  return polys.GetCellsInside(this->AreaCovered());
}

Swath Swath::Clone() const {
  Swath new_s {this->path_.Clone(), this->width_, this->id_};
  new_s.SetCreationDir(this->GetCreationDir());
  return new_s;
}

bool Swath::HasSameDir(const Swath& s) const {
  return s.GetCreationDir() == this->GetCreationDir();
}

void Swath::TargetSameDirAs(const Swath& s) {
  if (!this->HasSameDir(s)) {
    this->Reverse();
  }
}

void Swath::TargetOppositeDirAs(const Swath& s) {
  if (this->HasSameDir(s)) {
    this->Reverse();
  }
}

size_t Swath::NumPoints() const {
  return this->path_->getNumPoints();
}

Point Swath::GetPoint(int i) {
  return this->path_.GetGeometry(i);
}

const Point Swath::GetPoint(int i) const {
  return this->path_.GetGeometry(i);
}

int Swath::GetId() const {
  return this->id_;
}

void Swath::SetId(int _id) {
  this->id_ = _id;
}

LineString Swath::GetPath() const {
  return this->path_;
}

void Swath::SetPath(const LineString& _path) {
  this->path_ = _path;
}

double Swath::GetWidth() const {
  return this->width_;
}

void Swath::SetWidth(double _width) {
  if (0 >= _width) {
    throw std::invalid_argument("Width needs to be positive");
  }
  this->width_ = _width;
}

bool Swath::GetCreationDir() const {
  return this->creation_dir_;
}

void Swath::SetCreationDir(bool _creation_dir) {
  this->creation_dir_ = _creation_dir;
}

SwathType Swath::GetType() const {
  return this->type_;
}

void Swath::SetType(SwathType _type) {
  this->type_ = _type;
}

void Swath::MoveTo(const Point& ref_pt) {
  for (auto&& s : this->path_) {
    s = s + ref_pt;
  }
}

}  // namespace autonomy::planning::planner::fields2cover::types

