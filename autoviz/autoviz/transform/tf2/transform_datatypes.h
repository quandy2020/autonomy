/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file transform_datatypes.h
 * @brief TF2 @c Stamped&lt;T&gt; wrapper — data plus stamp and frame_id.
 *
 * Cross-compatible with stamped geometry message patterns used throughout
 * tf2 convert / doTransform APIs.
 *
 * @see convert.h
 * @see Time
 */

#ifndef TF2_TRANSFORM_DATATYPES_H
#define TF2_TRANSFORM_DATATYPES_H

#include <string>

#include "autoviz/transform/tf2/time.h"

namespace autoviz {
namespace transform {
namespace tf2 {

/**
 * @class Stamped
 * @brief Timestamped, framed wrapper around an arbitrary data type @c T.
 *
 * TF2 equivalent of a stamped message: inherits @c T and adds @c stamp_ /
 * @c frame_id_.
 *
 * @tparam T Underlying data type (e.g. pose, transform, quaternion).
 */
template <typename T>
class Stamped : public T
{
public:
    /** Timestamp associated with this data (@ref Time nanoseconds). */
    Time stamp_;
    /** Frame id in which @c T is expressed. */
    std::string frame_id_;

    /**
     * @brief Default constructor (preallocation); frame id is a sentinel.
     */
    Stamped()
        : frame_id_(
              "NO_ID_STAMPED_DEFAULT_CONSTRUCTION"){};  // Default constructor
                                                        // used only for
                                                        // preallocation

    /**
     * @brief Full constructor from data, stamp, and frame.
     *
     * @param input Underlying data copied into the base @c T.
     * @param timestamp Nanosecond stamp.
     * @param frame_id Frame id string.
     */
    Stamped(const T& input, const Time& timestamp, const std::string& frame_id)
        : T(input), stamp_(timestamp), frame_id_(frame_id){};

    /**
     * @brief Copy constructor.
     * @param s Source stamped value.
     */
    Stamped(const Stamped<T>& s)
        : T(s), stamp_(s.stamp_), frame_id_(s.frame_id_) {}

    /**
     * @brief Replaces the underlying @c T data without changing stamp/frame.
     * @param input New data assigned into the base subobject.
     */
    void setData(const T& input) {
        *static_cast<T*>(this) = input;
    };
};

/**
 * @brief Equality for @ref Stamped: frame, stamp, and base @c T must match.
 *
 * @tparam T Underlying data type.
 * @param a Left operand.
 * @param b Right operand.
 * @return @c true if frame_id, stamp, and @c T compare equal.
 */
template <typename T>
bool operator==(const Stamped<T>& a, const Stamped<T>& b) {
    return a.frame_id_ == b.frame_id_ && a.stamp_ == b.stamp_ &&
           static_cast<const T&>(a) == static_cast<const T&>(b);
};

}  // namespace tf2

}  // namespace transform
}  // namespace autoviz

#endif  // TF2_TRANSFORM_DATATYPES_H
