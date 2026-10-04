/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans
http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use
of this software. Permission is granted to anyone to use this software for any
purpose, including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim
that you wrote the original software. If you use this software in a product, an
acknowledgment in the product documentation would be appreciated but is not
required.
2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

/**
 * @file MinMax.h
 * @brief TF2 / Bullet min/max/clamp helpers for scalar and vector types.
 *
 * @see Scalar.h
 * @see Vector3
 */

#ifndef GEN_MINMAX_H
#define GEN_MINMAX_H

#include "autoviz/transform/tf2/LinearMath/Scalar.h"

namespace autoviz {
namespace transform {
namespace tf2 {

/**
 * @brief Returns the lesser of @p a and @p b.
 * @tparam T Comparable type.
 * @param a Left value.
 * @param b Right value.
 * @return Const reference to the minimum.
 */
template <class T>
TF2SIMD_FORCE_INLINE const T& tf2Min(const T& a, const T& b) {
    return a < b ? a : b;
}

/**
 * @brief Returns the greater of @p a and @p b.
 * @tparam T Comparable type.
 * @param a Left value.
 * @param b Right value.
 * @return Const reference to the maximum.
 */
template <class T>
TF2SIMD_FORCE_INLINE const T& tf2Max(const T& a, const T& b) {
    return a > b ? a : b;
}

/**
 * @brief Clamps @p a into [@p lb, @p ub].
 * @tparam T Comparable type.
 * @param a Value to clamp.
 * @param lb Lower bound.
 * @param ub Upper bound.
 * @return Clamped value.
 */
template <class T>
TF2SIMD_FORCE_INLINE const T& GEN_clamped(const T& a, const T& lb,
                                          const T& ub) {
    return a < lb ? lb : (ub < a ? ub : a);
}

/**
 * @brief Sets @p a to @c min(a, b).
 * @tparam T Comparable type.
 * @param[in,out] a Value updated if @p b is smaller.
 * @param b Candidate minimum.
 */
template <class T>
TF2SIMD_FORCE_INLINE void tf2SetMin(T& a, const T& b) {
    if (b < a) {
        a = b;
    }
}

/**
 * @brief Sets @p a to @c max(a, b).
 * @tparam T Comparable type.
 * @param[in,out] a Value updated if @p b is larger.
 * @param b Candidate maximum.
 */
template <class T>
TF2SIMD_FORCE_INLINE void tf2SetMax(T& a, const T& b) {
    if (a < b) {
        a = b;
    }
}

template <class T>
TF2SIMD_FORCE_INLINE void GEN_clamp(T& a, const T& lb, const T& ub) {
    if (a < lb) {
        a = lb;
    } else if (ub < a) {
        a = ub;
    }
}

}  // namespace tf2
}  // namespace transform
}  // namespace autoviz

#endif
