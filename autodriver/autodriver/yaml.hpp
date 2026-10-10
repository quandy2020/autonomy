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
 * @file yaml.hpp
 * @brief yaml-cpp-shaped facade over header-only fkYAML (autolink/thirdparty).
 *
 * Keeps existing call sites (`YAML::Node`, `LoadFile`, `IsMap`, `as<T>`, …)
 * while depending only on the in-tree fkYAML headers.
 */

#pragma once

#include <fstream>
#include <string>
#include <utility>

#include <fkYAML/node.hpp>

namespace YAML {

using Exception = fkyaml::exception;

struct iterator_value;

class Node {
 public:
  Node() = default;

  explicit Node(fkyaml::node node) : node_(std::move(node)), defined_(true) {}

  static Node Undefined() { return Node(); }

  explicit operator bool() const { return defined_ && !node_.is_null(); }

  bool IsDefined() const { return defined_; }
  bool IsNull() const { return !defined_ || node_.is_null(); }
  bool IsScalar() const {
    return defined_ && !node_.is_null() && node_.is_scalar();
  }
  bool IsMap() const { return defined_ && node_.is_mapping(); }
  bool IsSequence() const { return defined_ && node_.is_sequence(); }

  std::size_t size() const {
    if (!defined_ || (!node_.is_mapping() && !node_.is_sequence())) {
      return 0;
    }
    return node_.size();
  }

  Node operator[](const char* key) const { return Lookup(std::string(key)); }

  Node operator[](const std::string& key) const { return Lookup(key); }

  Node operator[](int index) const {
    if (!defined_ || !node_.is_sequence()) {
      return Undefined();
    }
    if (index < 0 || static_cast<std::size_t>(index) >= node_.size()) {
      return Undefined();
    }
    return Node(node_[index]);
  }

  template <typename T>
  T as() const {
    if (!defined_) {
      throw Exception("YAML::Node::as: undefined node");
    }
    return node_.template get_value<T>();
  }

  template <typename T>
  T as(const T& default_value) const {
    if (!defined_ || node_.is_null()) {
      return default_value;
    }
    return node_.template get_value_or<T>(default_value);
  }

  class const_iterator {
   public:
    const_iterator() = default;

    const_iterator(fkyaml::node::const_iterator it, bool is_map)
        : it_(it), is_map_(is_map), valid_(true) {}

    iterator_value operator*() const;

    const_iterator& operator++() {
      ++it_;
      return *this;
    }

    bool operator==(const const_iterator& other) const {
      if (!valid_ || !other.valid_) {
        return valid_ == other.valid_;
      }
      return it_ == other.it_;
    }

    bool operator!=(const const_iterator& other) const {
      return !(*this == other);
    }

   private:
    fkyaml::node::const_iterator it_{};
    bool is_map_{false};
    bool valid_{false};
  };

  const_iterator begin() const {
    if (!defined_ || (!node_.is_mapping() && !node_.is_sequence())) {
      return {};
    }
    return {node_.begin(), node_.is_mapping()};
  }

  const_iterator end() const {
    if (!defined_ || (!node_.is_mapping() && !node_.is_sequence())) {
      return {};
    }
    return {node_.end(), node_.is_mapping()};
  }

 private:
  Node Lookup(const std::string& key) const {
    if (!defined_ || !node_.is_mapping() || !node_.contains(key)) {
      return Undefined();
    }
    return Node(node_[key]);
  }

  fkyaml::node node_{};
  bool defined_{false};
};

/** Range-for element: `.first`/`.second` for maps; acts as Node for sequences. */
struct iterator_value {
  Node first;
  Node second;

  explicit operator bool() const { return static_cast<bool>(second); }
  bool IsDefined() const { return second.IsDefined(); }
  bool IsNull() const { return second.IsNull(); }
  bool IsScalar() const { return second.IsScalar(); }
  bool IsMap() const { return second.IsMap(); }
  bool IsSequence() const { return second.IsSequence(); }

  Node operator[](const char* key) const { return second[key]; }
  Node operator[](const std::string& key) const { return second[key]; }
  Node operator[](int index) const { return second[index]; }

  template <typename T>
  T as() const {
    return second.as<T>();
  }

  template <typename T>
  T as(const T& default_value) const {
    return second.as<T>(default_value);
  }

  operator const Node&() const { return second; }
};

inline iterator_value Node::const_iterator::operator*() const {
  iterator_value value;
  if (is_map_) {
    value.first = Node(it_.key());
    value.second = Node(*it_);
  } else {
    value.second = Node(*it_);
  }
  return value;
}

inline Node LoadFile(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    const std::string msg = "failed to open YAML file: " + path;
    throw Exception(msg.c_str());
  }
  return Node(fkyaml::node::deserialize(input));
}

}  // namespace YAML
