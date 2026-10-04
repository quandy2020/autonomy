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
 * @file exceptions.h
 * @brief TF2 exception hierarchy thrown by @ref BufferCore lookups.
 *
 * Vendored ROS tf2 exceptions under @c autoviz::transform::tf2. Used when
 * frames are missing, disconnected, extrapolated, or arguments are invalid.
 *
 * @see BufferCore
 * @see transform::Buffer
 */

#ifndef TF2_EXCEPTIONS_H
#define TF2_EXCEPTIONS_H

#include <stdexcept>
#include <string>

namespace autoviz {
namespace transform {
namespace tf2 {

/**
 * @class TransformException
 * @brief Base class for all tf2 transform errors (@c std::runtime_error).
 */
class TransformException : public std::runtime_error
{
public:
    /**
     * @brief Constructs with a human-readable description.
     * @param errorDescription Error message stored in @c what().
     */
    TransformException(const std::string errorDescription)
        : std::runtime_error(errorDescription) {
        ;
    };
};

/**
 * @class ConnectivityException
 * @brief Frames exist but the TF tree does not connect them.
 */
class ConnectivityException : public TransformException
{
public:
    /**
     * @brief Constructs with a connectivity failure description.
     * @param errorDescription Error message for @c what().
     */
    ConnectivityException(const std::string errorDescription)
        : tf2::TransformException(errorDescription) {
        ;
    };
};

/**
 * @class LookupException
 * @brief A requested frame id is not in the graph (unpublished / broken tree).
 */
class LookupException : public TransformException
{
public:
    /**
     * @brief Constructs with a lookup failure description.
     * @param errorDescription Error message for @c what().
     */
    LookupException(const std::string errorDescription)
        : tf2::TransformException(errorDescription) {
        ;
    };
};

/**
 * @class ExtrapolationException
 * @brief Requested time is outside the buffered history (would extrapolate).
 */
class ExtrapolationException : public TransformException
{
public:
    /**
     * @brief Constructs with an extrapolation failure description.
     * @param errorDescription Error message for @c what().
     */
    ExtrapolationException(const std::string errorDescription)
        : tf2::TransformException(errorDescription) {
        ;
    };
};

/**
 * @class InvalidArgumentException
 * @brief One or more arguments are invalid (e.g. zero quaternion).
 */
class InvalidArgumentException : public TransformException
{
public:
    /**
     * @brief Constructs with an invalid-argument description.
     * @param errorDescription Error message for @c what().
     */
    InvalidArgumentException(const std::string errorDescription)
        : tf2::TransformException(errorDescription) {
        ;
    };
};

/**
 * @class TimeoutException
 * @brief A timed lookup / wait exceeded its timeout.
 */
class TimeoutException : public TransformException
{
public:
    /**
     * @brief Constructs with a timeout description.
     * @param errorDescription Error message for @c what().
     */
    TimeoutException(const std::string errorDescription)
        : tf2::TransformException(errorDescription) {
        ;
    };
};

}  // namespace tf2
}  // namespace transform
}  // namespace autoviz

#endif  // TF2_EXCEPTIONS_H
