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
 * @file transform_stamped.h
 * @brief Minimal geometry_msgs-compatible structs for Autoviz tf2 internals.
 *
 * Lightweight stand-ins for ROS geometry_msgs used by BufferCore /
 * TransformStorage without depending on full ROS message headers. Autoviz
 * public APIs prefer Automsgs protobuf types via @ref transform::Buffer.
 *
 * @see TransformStorage
 * @see BufferCore
 */

#pragma once

#include <stdint.h>

#include <iostream>

namespace geometry_msgs {

/**
 * @struct Header
 * @brief Minimal stamped-message header (seq, stamp, frame_id).
 */
struct Header {
    /** Sequence number (often unused in Autoviz). */
    uint32_t seq;
    /** Timestamp in nanoseconds (tf2 @ref Time compatible). */
    uint64_t stamp;
    /** Frame id string. */
    std::string frame_id;
    /** @brief Zero-initializes seq/stamp and empty frame_id. */
    Header() : seq(0), stamp(0), frame_id("") {}
};

/**
 * @struct Vector3
 * @brief 3D vector (x, y, z).
 */
struct Vector3 {
    double x;  /**< X component. */
    double y;  /**< Y component. */
    double z;  /**< Z component. */
    /** @brief Constructs a zero vector. */
    Vector3() : x(0.0), y(0.0), z(0.0) {}
};

/**
 * @struct Quaternion
 * @brief Unit quaternion (x, y, z, w).
 */
struct Quaternion {
    double x;  /**< Imaginary i / x. */
    double y;  /**< Imaginary j / y. */
    double z;  /**< Imaginary k / z. */
    double w;  /**< Real / w component. */
    /** @brief Constructs a zero quaternion (invalid until set). */
    Quaternion() : x(0.0), y(0.0), z(0.0), w(0.0) {}
};

/**
 * @struct QuaternionStamped
 * @brief Quaternion with a @ref Header.
 */
struct QuaternionStamped {
    Header header;          /**< Stamp and frame of the quaternion. */
    Quaternion quaternion;  /**< Orientation. */
};

/**
 * @struct Transform
 * @brief Rigid transform: translation + rotation.
 */
struct Transform {
    Vector3 translation;  /**< Translation component. */
    Quaternion rotation;  /**< Rotation component. */
};

/**
 * @struct TransformStamped
 * @brief Stamped transform from @c child_frame_id into @c header.frame_id.
 */
struct TransformStamped {
    Header header;              /**< Parent frame and stamp. */
    std::string child_frame_id; /**< Child frame id. */
    Transform transform;        /**< Rigid transform child → parent. */
};

}  // namespace geometry_msgs
