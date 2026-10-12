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
 * @file linear_interpolation.hpp
 * @brief Get the interval index and interpolation coefficient alpha.
 */


#include <type_traits>
#include <utility>
#include <vector>

#include "bolza/core/types.hpp"

namespace bolza {
namespace LinearInterpolation {

using index_alpha_t = std::pair<int, scalar_t>;

template <class T>
using remove_cvref_t = typename std::remove_cv<typename std::remove_reference<T>::type>::type;

/**
 * Get the interval index and interpolation coefficient alpha.
 * Alpha = 1 at the start of the interval and alpha = 0 at the end.
 *
 * @param [in] enquiryTime: The enquiry time for interpolation.
 * @param [in] timeArray: interpolation time array.
 * @return {index, alpha}
 */
index_alpha_t TimeSegment(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray);

/**
 * Directly uses the index and interpolation coefficient provided by the user
 * @note If sizes in data array are not equal, the interpolation will snap to the data
 * point closest to the query time
 *
 *  - No data implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are used for linear interpolation and zero order extrapolation
 *
 * @param [in] indexAlpha: index and interpolation coefficient (alpha) pair
 * @param [in] dataArray: vector of data
 * @return The interpolation result
 *
 * @tparam Data: Data type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, class Alloc>
/**
 * @brief Interpolate.
 * @param [in] indexAlpha: Index Alpha.
 * @param [in] dataArray: Data Array.
 * @return Result value.
 */
Data Interpolate(index_alpha_t indexAlpha, const std::vector<Data, Alloc>& dataArray);

/**
 * Linearly interpolates at the given time. When duplicate values exist the lower range is selected s.t. ( ]
 * Example: t = [0.0, 1.0, 1.0, 2.0]
 * when querying tk = 1.0, the range (0.0, 1.0] is selected
 *
 *  - No data implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are used for linear interpolation and zero order extrapolation
 *
 * @param [in] enquiryTime: The enquiry time for interpolation.
 * @param [in] timeArray: Times vector
 * @param [in] dataArray: Data vector
 * @return The interpolation result
 *
 * @tparam Data: Data type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, class Alloc>
/**
 * @brief Interpolate.
 * @param [in] enquiryTime: Enquiry Time.
 * @param [in] timeArray: Time Array.
 * @param [in] dataArray: Data Array.
 * @return Result value.
 */
Data Interpolate(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray, const std::vector<Data, Alloc>& dataArray);

/**
 * Directly uses the index and interpolation coefficient provided by the user
 * @note If sizes in data array are not equal, the interpolation will snap to the data
 * point closest to the query time
 *
 *  - No data implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are used for linear interpolation and zero order extrapolation
 *
 * @param [in] indexAlpha: index and interpolation coefficient (alpha) pair
 * @param [in] dataArray: vector of data
 * @param [in] accessFun: Method to access the subfield of Data in array. The signature of the accessFun
 *                        should be equivalent to the following where Field is any subfield of Data:
 *                        const Field& AccessFun(const std::vector<Data, Alloc>& array, size_t index)
 * @return The interpolation result
 *
 * @tparam Data: Data type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, class Alloc, class AccessFun>
auto Interpolate(index_alpha_t indexAlpha, const std::vector<Data, Alloc>& dataArray, AccessFun accessFun)
    -> remove_cvref_t<typename std::result_of<AccessFun(const std::vector<Data, Alloc>&, size_t)>::type>;

/**
 * Linearly interpolates at the given time. When duplicate values exist the lower range is selected s.t. ( ]
 * Example: t = [0.0, 1.0, 1.0, 2.0]
 * when querying tk = 1.0, the range (0.0, 1.0] is selected
 *
 *  - No data implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are used for linear interpolation and zero order extrapolation
 *
 * @param [in] enquiryTime: The enquiry time for interpolation.
 * @param [in] timeArray: Times vector
 * @param [in] dataArray: Data vector
 * @param [in] accessFun: Method to access the subfield of Data in array. The signature of the accessFun
 *                        should be equivalent to the following where Field is any subfield of Data:
 *                        const Field& AccessFun(const std::vector<Data, Alloc>& array, size_t index)
 * @return The interpolation result
 *
 * @tparam Data: Data type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data, class Alloc, class AccessFun>
auto Interpolate(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray, const std::vector<Data, Alloc>& dataArray,
                 AccessFun accessFun) -> remove_cvref_t<typename std::result_of<AccessFun(const std::vector<Data, Alloc>&, size_t)>::type>;

}  // namespace LinearInterpolation
}  // namespace bolza

#include "implementation/linear_interpolation.hpp"
