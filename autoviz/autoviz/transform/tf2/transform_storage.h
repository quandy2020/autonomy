/*
 * Copyright (c) 2010, Willow Garage, Inc.
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
 * @file transform_storage.h
 * @brief Internal storage record for one stamped transform in a TimeCache.
 *
 * Holds rotation, translation, stamp, and compact parent/child frame ids used
 * by @ref TimeCache / @ref StaticCache inside @ref BufferCore.
 *
 * @see TimeCache
 * @see CompactFrameID
 */

#ifndef TF2_TRANSFORM_STORAGE_H
#define TF2_TRANSFORM_STORAGE_H

#include <autoviz/transform/geometry_msgs/transform_stamped.h>
#include <autoviz/transform/tf2/LinearMath/Quaternion.h>
#include <autoviz/transform/tf2/LinearMath/Vector3.h>
// #include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/transform/tf2/time.h"

namespace autoviz {
namespace transform {
namespace tf2 {

/**
 * @brief Compact integer frame id used inside BufferCore (not a string name).
 */
typedef uint32_t CompactFrameID;

/**
 * @class TransformStorage
 * @brief One buffered transform sample (pose + stamp + compact frame ids).
 */
class TransformStorage
{
public:
    /** @brief Default-constructs an empty / zero storage record. */
    TransformStorage();

    /**
     * @brief Constructs from a geometry_msgs stamped transform and frame ids.
     *
     * @param data Stamped transform (translation + rotation + stamp).
     * @param frame_id Compact parent frame id.
     * @param child_frame_id Compact child frame id.
     */
    TransformStorage(const geometry_msgs::TransformStamped& data,
                     CompactFrameID frame_id, CompactFrameID child_frame_id);

    /**
     * @brief Copy constructor.
     * @param rhs Source storage.
     */
    TransformStorage(const TransformStorage& rhs) {
        *this = rhs;
    }

    /**
     * @brief Copy assignment.
     * @param rhs Source storage.
     * @return @c *this.
     */
    TransformStorage& operator=(const TransformStorage& rhs) {
#if 01
        rotation_ = rhs.rotation_;
        translation_ = rhs.translation_;
        stamp_ = rhs.stamp_;
        frame_id_ = rhs.frame_id_;
        child_frame_id_ = rhs.child_frame_id_;
#endif
        return *this;
    }

    /** Rotation component (tf2 quaternion). */
    tf2::Quaternion rotation_;
    /** Translation component (tf2 vector). */
    tf2::Vector3 translation_;
    /** Sample timestamp (nanoseconds). */
    Time stamp_{0};
    /** Compact parent frame id. */
    CompactFrameID frame_id_{0};
    /** Compact child frame id. */
    CompactFrameID child_frame_id_{0};
};

}  // namespace tf2
}  // namespace transform
}  // namespace autoviz

#endif  // TF2_TRANSFORM_STORAGE_H
