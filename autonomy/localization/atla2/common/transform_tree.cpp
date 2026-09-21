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

#include "autonomy/localization/atla2/common/transform_tree.hpp"

namespace autonomy::localization::atla2 {

void TransformTree::Set(const std::string& parent, const std::string& child,
                        const SE3& T_parent_child) {
  child_to_parent_[child] = Edge{parent, T_parent_child};
}

bool TransformTree::Has(const std::string& parent, const std::string& child) const {
  const auto it = child_to_parent_.find(child);
  return it != child_to_parent_.end() && it->second.parent == parent;
}

void TransformTree::Clear() { child_to_parent_.clear(); }

bool TransformTree::Lookup(const std::string& a, const std::string& b, SE3* T_a_b) const {
  if (!T_a_b) {
    return false;
  }
  if (a == b) {
    *T_a_b = Se3Identity();
    return true;
  }

  SE3 T_root_b = Se3Identity();
  std::string cur = b;
  for (int i = 0; i < 32; ++i) {
    if (cur == a) {
      *T_a_b = T_root_b;
      return true;
    }
    const auto it = child_to_parent_.find(cur);
    if (it == child_to_parent_.end()) {
      break;
    }
    T_root_b = it->second.T_parent_child * T_root_b;
    cur = it->second.parent;
  }

  SE3 T_root_a = Se3Identity();
  cur = a;
  for (int i = 0; i < 32; ++i) {
    if (cur == b) {
      *T_a_b = Se3Inverse(T_root_a);
      return true;
    }
    const auto it = child_to_parent_.find(cur);
    if (it == child_to_parent_.end()) {
      break;
    }
    T_root_a = it->second.T_parent_child * T_root_a;
    cur = it->second.parent;
  }

  const auto ia = child_to_parent_.find(a);
  const auto ib = child_to_parent_.find(b);
  if (ia != child_to_parent_.end() && ib != child_to_parent_.end() &&
      ia->second.parent == ib->second.parent) {
    *T_a_b = Se3Inverse(ia->second.T_parent_child) * ib->second.T_parent_child;
    return true;
  }
  return false;
}

}  // namespace autonomy::localization::atla2
