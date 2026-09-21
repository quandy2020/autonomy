/*
 * Copyright 2026 The Openbot Authors
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

#include <string>
#include <unordered_map>

#include "autonomy/localization/atla2/common/types.hpp"

namespace autonomy::localization::atla2 {

//! Extrinsic tree: each edge is T_parent_child (pose of child in parent).
class TransformTree {
 public:
  void Set(const std::string& parent, const std::string& child, const SE3& T_parent_child);

  //! Lookup T_a_b by composing along the tree (parent->child edges).
  //! Returns false if no path.
  bool Lookup(const std::string& a, const std::string& b, SE3* T_a_b) const;

  bool Has(const std::string& parent, const std::string& child) const;

  void Clear();

 private:
  struct Edge {
    std::string parent;
    SE3 T_parent_child = Se3Identity();
  };
  std::unordered_map<std::string, Edge> child_to_parent_;
};

}  // namespace autonomy::localization::atla2
