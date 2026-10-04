/*
 * Copyright (c) 2013, Open Source Robotics Foundation
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
 * @file convert.h
 * @brief TF2 type conversion and transform-application templates.
 *
 * Client libraries specialize @ref doTransform, @ref toMsg, and @ref fromMsg
 * for their datatypes. @ref convert dispatches via @ref impl::Converter.
 *
 * @see impl/convert.h
 * @see Stamped
 */

#ifndef TF2_CONVERT_H
#define TF2_CONVERT_H

#include <autoviz/transform/geometry_msgs/transform_stamped.h>
#include <autoviz/transform/tf2/exceptions.h>
#include <autoviz/transform/tf2/transform_datatypes.h>
// #include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <autoviz/transform/tf2/impl/convert.h>
#include <autoviz/transform/tf2/time.h>

namespace autoviz {
namespace transform {
namespace tf2 {

/**
 * @brief Applies @p transform to @p data_in, writing the result to @p data_out.
 *
 * Must be specialized by client datatype libraries. May alias in/out.
 *
 * @tparam T Datatype being transformed.
 * @param data_in Input value.
 * @param[out] data_out Transformed value (may reference @p data_in).
 * @param transform Stamped transform to apply.
 */
template <class T>
void doTransform(const T& data_in, T& data_out,
                 const geometry_msgs::TransformStamped& transform);

/**
 * @brief Returns the timestamp associated with @p t.
 *
 * @tparam T Datatype providing a stamp (specialized for @ref Stamped).
 * @param t Input data.
 * @return Const reference to the timestamp (lifetime bound to @p t).
 */
template <class T>
const Time& getTimestamp(const T& t);

/**
 * @brief Returns the frame_id associated with @p t.
 *
 * @tparam T Datatype providing a frame id (specialized for @ref Stamped).
 * @param t Input data.
 * @return Const reference to the frame id (lifetime bound to @p t).
 */
template <class T>
const std::string& getFrameId(const T& t);

/**
 * @brief @ref Stamped specialization of @ref getTimestamp.
 * @tparam P Underlying stamped payload type.
 * @param t Stamped value.
 * @return Reference to @c t.stamp_.
 */
template <class P>
const Time& getTimestamp(const tf2::Stamped<P>& t) {
    return t.stamp_;
}

/**
 * @brief @ref Stamped specialization of @ref getFrameId.
 * @tparam P Underlying stamped payload type.
 * @param t Stamped value.
 * @return Reference to @c t.frame_id_.
 */
template <class P>
const std::string& getFrameId(const tf2::Stamped<P>& t) {
    return t.frame_id_;
}

/**
 * @brief Converts @p a to a message-like type @c B.
 *
 * Implemented per datatype in tf2_* packages (except pure message types).
 *
 * @tparam A Source type.
 * @tparam B Destination message type.
 * @param a Source object.
 * @return Converted message of type @c B.
 */
template <typename A, typename B>
B toMsg(const A& a);

/**
 * @brief Converts message-like @p a into non-message type @p b.
 *
 * @tparam A Source message type.
 * @tparam B Destination type.
 * @param a Source message (unnamed in declaration; see specializations).
 * @param[out] b Destination object.
 */
template <typename A, typename B>
void fromMsg(const A&, B& b);

/**
 * @brief Converts any type @c A to any type @c B when toMsg/fromMsg exist.
 *
 * Specialize when both types are messages or when the default path does not
 * apply. The generic body is intentionally empty in this Autoviz fork.
 *
 * @tparam A Source type.
 * @tparam B Destination type.
 * @param a Source object.
 * @param[out] b Destination object.
 */
template <class A, class B>
void convert(const A& a, B& b) {
    // printf("In double type convert\n");
    //  impl::Converter<ros::message_traits::IsMessage<A>::value,
    //  ros::message_traits::IsMessage<B>::value>::convert(a, b);
}

/**
 * @brief Same-type convert: assigns @p a1 to @p a2 unless they alias.
 *
 * @tparam A Value type.
 * @param a1 Source.
 * @param[out] a2 Destination (unchanged if @c &a1 == &a2).
 */
template <class A>
void convert(const A& a1, A& a2) {
    // printf("In single type convert\n");
    if (&a1 != &a2)
        a2 = a1;
}

}  // namespace tf2
}  // namespace transform
}  // namespace autoviz

#endif  // TF2_CONVERT_H
