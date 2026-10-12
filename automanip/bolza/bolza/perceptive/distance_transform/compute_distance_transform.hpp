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
 * @file compute_distance_transform.hpp
 * @brief Computes one-dimensional distance transform for a sampled function based on lower envelope of parabolas method introduced by:.
 */


#include <cstddef>
#include <vector>

namespace bolza {

/**
 * Computes one-dimensional distance transform for a sampled function based on lower envelope of parabolas method introduced by:
 * P. F. Felzenszwalb and D. P. Huttenlocher. "Distance transforms of sampled functions." (2012).
 *
 * @param [in] numSamples: The size of the sampled function.
 * @param [in] getValue: Lambda function to get value from with signature: ScalarType(size_t index).
 * @param [in] setValue: Lambda function to set value to with signature: void(size_t index, ScalarType value).
 * @param [in] start: The start index for iteration over.
 * @param [in] end: The index of the element following the last element for iteration over.
 * @param [in] vBuffer: A buffered memory of unsigned integers with size "numSamples". This memory holds the information
 *                 about the locations of parabolas in lower envelope.
 * @param [in] zBuffer: A buffered memory of Scalar type with size "numSamples + 1". This memory holds the information
 *                 about the locations of boundaries between parabolas.
 *
 * @tparam GetValFunc: Template typename for inferring lambda expression with signature Scalar(size_t).
 * @tparam SetValFunc: Template typename for inferring lambda expression with signature void(size_t, Scalar).
 * @tparam Scalar: The Scalar type
 */
template <typename GetValFunc, typename SetValFunc, typename Scalar = float>
/**
 * @brief Compute Distance Transform.
 * @param [in] numSamples: Num Samples.
 * @param [in,out] getValue: Get Value.
 * @param [in,out] setValue: Set Value.
 * @param [in] start: Start.
 * @param [in] end: End.
 * @param [in,out] vBuffer: V Buffer.
 * @param [in,out] zBuffer: Z Buffer.
 */
void ComputeDistanceTransform(std::size_t numSamples, GetValFunc&& getValue, SetValFunc&& setValue, std::size_t start, std::size_t end,
                              std::vector<std::size_t>& vBuffer, std::vector<Scalar>& zBuffer);

/**
 * Computes one-dimensional distance transform for a sampled function based on lower envelope of parabolas method introduced by:
 * P. F. Felzenszwalb and D. P. Huttenlocher. "Distance transforms of sampled functions." (2012).
 *
 * @param [in] numSamples: The size of the sampled function.
 * @param [in] getValue: Lambda function to get value from with signature: ScalarType(size_t index).
 * @param [in] setValue: Lambda function to set value to with signature: void(size_t index, ScalarType value).
 * @param [in] setImageIndex: Lambda function to set mirror index with signature: void(size_t index, size_t mirrorIndex).
 * @param [in] start: The start index for iteration over.
 * @param [in] end: The index of the element following the last element for iteration over.
 * @param [in] vBuffer: A buffered memory of unsigned integers with size "numSamples". This memory holds the information
 *                 about the locations of parabolas in lower envelope.
 * @param [in] zBuffer: A buffered memory of Scalar type with size "numSamples + 1". This memory holds the information
 *                 about the locations of boundaries between parabolas.
 *
 * @tparam GetValFunc: Template typename for inferring lambda expression with signature Scalar(size_t).
 * @tparam SetValFunc: Template typename for inferring lambda expression with signature void(size_t, Scalar).
 * @tparam SetImageIndexFunc: Template typename for inferring lambda expression with signature void(size_t, size_t).
 * @tparam Scalar: The Scalar type
 */
template <typename GetValFunc, typename SetValFunc, typename SetImageIndexFunc, typename Scalar = float>
/**
 * @brief Compute Distance Transform.
 * @param [in] numSamples: Num Samples.
 * @param [in,out] getValue: Get Value.
 * @param [in,out] setValue: Set Value.
 * @param [in,out] setImageIndex: Set Image Index.
 * @param [in] start: Start.
 * @param [in] end: End.
 * @param [in,out] vBuffer: V Buffer.
 * @param [in,out] zBuffer: Z Buffer.
 */
void ComputeDistanceTransform(std::size_t numSamples, GetValFunc&& getValue, SetValFunc&& setValue, SetImageIndexFunc&& setImageIndex,
                              std::size_t start, std::size_t end, std::vector<std::size_t>& vBuffer, std::vector<Scalar>& zBuffer);

}  // namespace bolza

#include "implementation/compute_distance_transform.hpp"
