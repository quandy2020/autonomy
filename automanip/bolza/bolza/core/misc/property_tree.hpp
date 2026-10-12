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
 * @file property_tree.hpp
 * @brief Minimal property tree and INFO-format file reader.
 */

#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace bolza {
namespace property_tree {

/** Thrown when @ref ptree::get is called for a missing path. */
class PtreeBadPath : public std::runtime_error {
 public:
  explicit PtreeBadPath(const std::string& path)
      : std::runtime_error("bolza::property_tree: path not found: \"" + path + "\"") {}
};

template <typename T>
struct Translator {
  static T get(const std::string& s) {
    T value{};
    std::istringstream iss(s);
    iss >> value;
    if (iss.fail() && !s.empty()) {
      throw std::runtime_error("bolza::property_tree: failed to convert \"" + s + "\" to requested type");
    }
    return value;
  }
};

template <>
struct Translator<std::string> {
  static std::string get(const std::string& s) { return s; }
};

template <>
struct Translator<bool> {
  static bool get(const std::string& s) {
    if (s == "true" || s == "1") {
      return true;
    }
    if (s == "false" || s == "0") {
      return false;
    }
    throw std::runtime_error("bolza::property_tree: failed to convert \"" + s + "\" to bool");
  }
};

template <>
struct Translator<int> {
  static int get(const std::string& s) { return std::stoi(s); }
};

template <>
struct Translator<long> {
  static long get(const std::string& s) { return std::stol(s); }
};

template <>
struct Translator<unsigned long> {
  static unsigned long get(const std::string& s) { return std::stoul(s); }
};

template <>
struct Translator<float> {
  static float get(const std::string& s) { return std::stof(s); }
};

template <>
struct Translator<double> {
  static double get(const std::string& s) { return std::stod(s); }
};

template <>
struct Translator<std::size_t> {
  static std::size_t get(const std::string& s) { return static_cast<std::size_t>(std::stoull(s)); }
};

class ptree {
 public:
  ptree() = default;

  template <typename T>
  T get(const std::string& path) const {
    const std::string* data = ResolveData(path);
    if (data == nullptr) {
      throw PtreeBadPath(path);
    }
    return Translator<T>::get(*data);
  }

  template <typename T>
  T get(const std::string& path, const T& default_value) const {
    const std::string* data = ResolveData(path);
    if (data == nullptr) {
      return default_value;
    }
    return Translator<T>::get(*data);
  }

 private:
  struct Node {
    std::string data;
    bool has_data = false;
    std::unordered_map<std::string, Node> children;
  };

  Node root_;

  const std::string* ResolveData(const std::string& path) const;
  static std::vector<std::string> SplitPath(const std::string& path);

  friend void read_info(const std::string& filename, ptree& pt);
};

void read_info(const std::string& filename, ptree& pt);

}  // namespace property_tree
}  // namespace bolza
