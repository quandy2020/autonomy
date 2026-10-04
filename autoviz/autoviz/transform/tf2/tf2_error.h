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
 * @file tf2_error.h
 * @brief Numeric TF2 error codes mirroring @c tf2_msgs/TF2Error.
 *
 * Used by BufferCore / canTransform paths that report status without throwing.
 *
 * @see TransformException
 * @see BufferCore
 */

#ifndef TF2_MSGS_TF2_ERROR_H
#define TF2_MSGS_TF2_ERROR_H

namespace autoviz {
namespace transform {
namespace tf2 {

namespace tf2_msgs {
/**
 * @brief Error code constants for TF2 lookup / wait results.
 */
namespace TF2Error {
/** @brief Success / no error. */
const uint8_t NO_ERROR = 0;
/** @brief Frame lookup failed (@ref LookupException). */
const uint8_t LOOKUP_ERROR = 1;
/** @brief Frames not connected (@ref ConnectivityException). */
const uint8_t CONNECTIVITY_ERROR = 2;
/** @brief Time outside buffer (@ref ExtrapolationException). */
const uint8_t EXTRAPOLATION_ERROR = 3;
/** @brief Invalid argument (@ref InvalidArgumentException). */
const uint8_t INVALID_ARGUMENT_ERROR = 4;
/** @brief Wait timed out (@ref TimeoutException). */
const uint8_t TIMEOUT_ERROR = 5;
/** @brief Generic transform failure. */
const uint8_t TRANSFORM_ERROR = 6;
}  // namespace TF2Error
}  // namespace tf2_msgs

}  // namespace tf2
}  // namespace transform
}  // namespace autoviz

#endif
