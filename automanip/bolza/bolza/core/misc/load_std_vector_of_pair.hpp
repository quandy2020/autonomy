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
 * @file load_std_vector_of_pair.hpp
 * @brief Load vectors of comma-separated pairs from INFO config files.
 */

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <bolza/core/misc/load_data.hpp>
#include <bolza/core/misc/property_tree.hpp>
#include <bolza/core/types.hpp>

namespace bolza {
namespace loadData {
namespace detail {

template <typename T>
T FromString(const std::string& str);

template <>
inline std::string FromString<std::string>(const std::string& str) {
  return str;
}

template <>
inline std::size_t FromString<std::size_t>(const std::string& str) {
  return std::stoul(str);
}

template <>
inline int FromString<int>(const std::string& str) {
  return std::stoi(str);
}

template <>
inline double FromString<double>(const std::string& str) {
  return std::stod(str);
}

template <>
inline float FromString<float>(const std::string& str) {
  return std::stof(str);
}

inline std::string TrimAscii(const std::string& s) {
  size_t begin = 0;
  while (begin < s.size() && std::isspace(static_cast<unsigned char>(s[begin])) != 0) {
    ++begin;
  }
  size_t end = s.size();
  while (end > begin && std::isspace(static_cast<unsigned char>(s[end - 1])) != 0) {
    --end;
  }
  return s.substr(begin, end - begin);
}

inline std::string StripOuterQuotes(const std::string& s) {
  if (s.size() >= 2 && s.front() == '"' && s.back() == '"') {
    return s.substr(1, s.size() - 2);
  }
  return s;
}

/** Split "first, second" (optional quotes and spaces around comma). */
inline std::pair<std::string, std::string> SplitPairTokens(const std::string& str) {
  const size_t comma = str.find(',');
  if (comma == std::string::npos) {
    throw std::runtime_error("Failed parsing pair: \"" + str + R"(". Expected: x,x or "x, x")");
  }
  std::string first = StripOuterQuotes(TrimAscii(str.substr(0, comma)));
  std::string second = StripOuterQuotes(TrimAscii(str.substr(comma + 1)));
  if (first.empty() || second.empty()) {
    throw std::runtime_error("Failed parsing pair: \"" + str + R"(". Expected: x,x or "x, x")");
  }
  return {first, second};
}

template <typename T1, typename T2>
struct ExtendedPair {
  T1 first;
  T2 second;

  using stdPair = std::pair<T1, T2>;

  ExtendedPair(T1 firstIn, T2 secondIn) : first(std::move(firstIn)), second(std::move(secondIn)) {}

  explicit ExtendedPair(const std::string& str) : first(), second() {
    const auto [firstStr, secondStr] = SplitPairTokens(str);
    first = FromString<T1>(firstStr);
    second = FromString<T2>(secondStr);
  }

  explicit operator stdPair() const { return std::pair<T1, T2>(first, second); }

  friend std::ostream& operator<<(std::ostream& stream, const ExtendedPair& pair) {
    stream << "[" << pair.first << ", " << pair.second << "]";
    return stream;
  }

  std::string ToString() const {
    std::ostringstream stream;
    stream << *this;
    return stream.str();
  }
};

}  // namespace detail
}  // namespace loadData
}  // namespace bolza

namespace bolza {
namespace property_tree {

template <typename T1, typename T2>
struct Translator<loadData::detail::ExtendedPair<T1, T2>> {
  static loadData::detail::ExtendedPair<T1, T2> get(const std::string& s) {
    return loadData::detail::ExtendedPair<T1, T2>(s);
  }
};

}  // namespace property_tree
}  // namespace bolza

namespace bolza {
namespace loadData {

/**
 * Loads a vector of comma-separated pairs from an INFO file.
 *
 * Format:
 * @code
 * topicName
 * {
 *   [0] "value1, value2"
 *   [1] "value1, value2"
 * }
 * @endcode
 */
template <typename T1, typename T2>
void LoadStdVectorOfPair(const std::string& filename, const std::string& topicName, std::vector<std::pair<T1, T2>>& loadVector,
                         bool verbose = true) {
  std::vector<detail::ExtendedPair<T1, T2>> extendedPairVector;
  LoadStdVector(filename, topicName, extendedPairVector, verbose);
  if (!extendedPairVector.empty()) {
    loadVector.clear();
    auto toStdPair = [](const detail::ExtendedPair<T1, T2>& p) { return std::pair<T1, T2>(p); };
    std::transform(extendedPairVector.begin(), extendedPairVector.end(), std::back_inserter(loadVector), toStdPair);
  }
}

}  // namespace loadData
}  // namespace bolza
