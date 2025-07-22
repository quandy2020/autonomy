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

#pragma once

#include <Eigen/Core>

// namespace autonomy {
// namespace common {
// namespace math {
  
#ifdef _MSVC_LANG
#define CPP_VERSION _MSVC_LANG
#else
#define CPP_VERSION __cplusplus
#endif

#if !EIGEN_VERSION_AT_LEAST(3, 4, 0) || CPP_VERSION < 201703L

#include <initializer_list>
#include <memory>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#ifndef EIGEN_ALIGNED_ALLOCATOR
#define EIGEN_ALIGNED_ALLOCATOR Eigen::aligned_allocator
#endif

// Equivalent to EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION but with support for
// initializer lists, which is a C++11 feature and not supported by the Eigen.
// The initializer list extension is inspired by Theia and StackOverflow code.
#define EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(...)                 \
  namespace std {                                                          \
  template <>                                                              \
  class vector<__VA_ARGS__, std::allocator<__VA_ARGS__>>                   \
      : public vector<__VA_ARGS__, EIGEN_ALIGNED_ALLOCATOR<__VA_ARGS__>> { \
    typedef vector<__VA_ARGS__, EIGEN_ALIGNED_ALLOCATOR<__VA_ARGS__>>      \
        vector_base;                                                       \
                                                                           \
   public:                                                                 \
    typedef __VA_ARGS__ value_type;                                        \
    typedef vector_base::allocator_type allocator_type;                    \
    typedef vector_base::size_type size_type;                              \
    typedef vector_base::iterator iterator;                                \
    explicit vector(const allocator_type& a = allocator_type())            \
        : vector_base(a) {}                                                \
    template <typename InputIterator>                                      \
    vector(InputIterator first,                                            \
           InputIterator last,                                             \
           const allocator_type& a = allocator_type())                     \
        : vector_base(first, last, a) {}                                   \
    vector(const vector& c) : vector_base(c) {}                            \
    explicit vector(size_type num, const value_type& val = value_type())   \
        : vector_base(num, val) {}                                         \
    vector(iterator start, iterator end) : vector_base(start, end) {}      \
    vector& operator=(const vector& x) {                                   \
      vector_base::operator=(x);                                           \
      return *this;                                                        \
    }                                                                      \
    vector(initializer_list<__VA_ARGS__> list)                             \
        : vector_base(list.begin(), list.end()) {}                         \
  };                                                                       \
  }  // namespace std

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Vector2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Vector4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Vector4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix4d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix4f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Affine3d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Affine3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Quaterniond)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Quaternionf)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix<float, 3, 4>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION_CUSTOM(Eigen::Matrix<double, 3, 4>)

#endif

#undef CPP_VERSION

// }  // namespace math
// }  // namespace common
// }  // namespace autonomy

