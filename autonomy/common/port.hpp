/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#ifndef AUTONOMY_COMMON_PORT_HPP_
#define AUTONOMY_COMMON_PORT_HPP_

#ifdef _MSC_VER
#if _MSC_VER >= 1600
#include <cstdint>
#else
typedef __int8 int8_t;
typedef __int16 int16_t;
typedef __int32 int32_t;
typedef __int64 int64_t;
typedef unsigned __int8 uint8_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
#endif
#elif __GNUC__ >= 3
#include <cstdint>
#endif

// Define non-copyable or non-movable classes.
// NOLINTNEXTLINE(bugprone-macro-parentheses)
#define NON_COPYABLE(class_name)          \
  class_name(class_name const&) = delete; \
  void operator=(class_name const& obj) = delete;
// NOLINTNEXTLINE(bugprone-macro-parentheses)
#define NON_MOVABLE(class_name) class_name(class_name&&) = delete;

#include "autonomy/common/eigen_alignment.hpp"

#include <Eigen/Core>

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <cinttypes>
#include <cmath>
#include <string>

namespace Eigen {

using Matrix3x4f = Matrix<float, 3, 4>;
using Matrix3x4d = Matrix<double, 3, 4>;
using Matrix6d = Matrix<double, 6, 6>;
using Vector3ub = Matrix<uint8_t, 3, 1>;
using Vector4ub = Matrix<uint8_t, 4, 1>;
using Vector6d = Matrix<double, 6, 1>;
using RowMajorMatrixXi = Matrix<int, Dynamic, Dynamic, RowMajor>;

}  // namespace Eigen

namespace autonomy {

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;
using float64 = double;

namespace common {

inline int RoundToInt(const float x) { return std::lround(x); }

inline int RoundToInt(const double x) { return std::lround(x); }

inline int64 RoundToInt64(const float x) { return std::lround(x); }

inline int64 RoundToInt64(const double x) { return std::lround(x); }

inline void FastGzipString(const std::string& uncompressed, std::string* compressed) 
{
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));
  out.push(boost::iostreams::back_inserter(*compressed));
  boost::iostreams::write(out, reinterpret_cast<const char*>(uncompressed.data()), uncompressed.size());
}

inline void FastGunzipString(const std::string& compressed, std::string* decompressed) 
{
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::gzip_decompressor());
  out.push(boost::iostreams::back_inserter(*decompressed));
  boost::iostreams::write(out, reinterpret_cast<const char*>(compressed.data()), compressed.size());
}

}  // namespace common
}  // namespace autonomy

// This file provides specializations of the templated hash function for
// custom types. These are used for comparison in unordered sets/maps.
namespace std {

// Hash function specialization for uint32_t pairs, e.g., image_t or camera_t.
template <>
struct hash<std::pair<uint32_t, uint32_t>> {
  std::size_t operator()(const std::pair<uint32_t, uint32_t>& p) const {
    const uint64_t s = (static_cast<uint64_t>(p.first) << 32) +
                       static_cast<uint64_t>(p.second);
    return std::hash<uint64_t>()(s);
  }
};

}  // namespace std

#endif  // AUTONOMY_COMMON_PORT_HPP_
