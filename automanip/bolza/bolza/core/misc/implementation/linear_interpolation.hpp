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

#include <algorithm>
#include <functional>

#include "bolza/core/numeric_traits.hpp"
#include "bolza/core/misc/lookup.hpp"

namespace bolza {
namespace LinearInterpolation {

/******************************************************************************************************/

/**
 * @file linear_interpolation.hpp
 * @brief Helper comparison function for non-Eigen types.
 */

/******************************************************************************************************/
/******************************************************************************************************/
/**
 * Helper comparison function for non-Eigen types.
 */
template <typename T>
/**
 * @brief Are Same Size.
 * @param [in] lhs: Lhs.
 * @param [in] rhs: Rhs.
 * @return Result value.
 */
constexpr typename std::enable_if<!std::is_base_of<Eigen::EigenBase<T>, T>::Value, bool>::type AreSameSize(const T& lhs, const T& rhs) {
  return true;
}

/**
 * Helper comparison function for Eigen-Type.
 */
template <typename Derived>
/**
 * @brief Are Same Size.
 * @param [in] lhs: Lhs.
 * @param [in] rhs: Rhs.
 * @return True on success.
 */
bool AreSameSize(const Eigen::EigenBase<Derived>& lhs, const Eigen::EigenBase<Derived>& rhs) {
  /**
   * @brief Rows.
   * @return Result value.
   */
  return lhs.rows() == rhs.rows() && lhs.cols() == rhs.cols();
}

/**
 * Helper access function for std::vector<Data, Alloc> where Data a simple data structure such as double, Eigen-Type.
 */
template <typename Data, class Alloc>
/**
 * @brief Std Access Fun.
 * @param [in] vec: Vec.
 * @param [in] ind: Ind.
 * @return Result value.
 */
const Data& StdAccessFun(const std::vector<Data, Alloc>& vec, size_t ind) {
  return vec[ind];
}

/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief /******************************************************************************************************/.
 * @param [in] enquiryTime: Enquiry Time.
 * @param [in] timeArray: Time Array.
 * @return Result value.
 */
inline index_alpha_t TimeSegment(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray) {
  // corner cases (no time set OR single time element)
  if (timeArray.size() <= 1) {
    return {0, scalar_t(1.0)};
  }

  /**
   * @brief Find Interval In Time Array.
   * @param [in] timeArray: Time Array.
   * @param [in] enquiryTime: Enquiry Time.
   * @return Result value.
   */
  const int index = lookup::FindIntervalInTimeArray(timeArray, enquiryTime);
  /**
   * @brief Size.
   * @return Result value.
   */
  const auto lastInterval = static_cast<int>(timeArray.size() - 1);
  if (index >= 0) {
    if (index < lastInterval) {
      // interpolation : 0 <= index < lastInterval
      /**
       * @brief Assert.
       */
      assert(enquiryTime <= timeArray[index + 1]);  // assert upper bound of lookup
      /**
       * @brief Assert.
       * @param [in] enquiryTime: Enquiry Time.
       */
      assert(timeArray[index] <= enquiryTime);      // assert lower bound of lookup
      const scalar_t intervalLength = timeArray[index + 1] - timeArray[index];
      const scalar_t timeTillNext = timeArray[index + 1] - enquiryTime;

      // Normal case: interval is large enough for normal interpolation
      constexpr scalar_t minIntervalTime = 2.0 * numeric_traits::WeakEpsilon<scalar_t>();
      if (intervalLength > minIntervalTime) {
        return {index, (timeTillNext / intervalLength)};
      }

      // Take closes point for small time intervals
      if (timeTillNext < 0.5 * intervalLength) {  // short interval, closest to time[index + 1]
        return {index, scalar_t(0.0)};
      } else {  // short interval, closest to time[index]
        return {index, scalar_t(1.0)};
      }
    } else {
      // upper bound : index >= lastInterval
      return {std::max(lastInterval - 1, 0), scalar_t(0.0)};
    }
  } else {
    // lower bound : index < 0
    return {0, scalar_t(1.0)};
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Data, class Alloc>
/**
 * @brief Interpolate.
 * @param [in] indexAlpha: Index Alpha.
 * @param [in] dataArray: Data Array.
 * @return Result value.
 */
Data Interpolate(index_alpha_t indexAlpha, const std::vector<Data, Alloc>& dataArray) {
  /**
   * @brief Interpolate.
   * @param [in] indexAlpha: Index Alpha.
   * @param [in] dataArray: Data Array.
   * @return Result value.
   */
  return Interpolate(indexAlpha, dataArray, StdAccessFun<Data, Alloc>);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Data, class Alloc>
/**
 * @brief Interpolate.
 * @param [in] enquiryTime: Enquiry Time.
 * @param [in] timeArray: Time Array.
 * @param [in] dataArray: Data Array.
 * @return Result value.
 */
Data Interpolate(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray, const std::vector<Data, Alloc>& dataArray) {
  /**
   * @brief Interpolate.
   * @param [in] enquiryTime: Enquiry Time.
   * @param [in] timeArray: Time Array.
   * @param [in] dataArray: Data Array.
   * @return Result value.
   */
  return Interpolate(enquiryTime, timeArray, dataArray, StdAccessFun<Data, Alloc>);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Data, class Alloc, class AccessFun>
auto Interpolate(index_alpha_t indexAlpha, const std::vector<Data, Alloc>& dataArray, AccessFun accessFun)
    -> remove_cvref_t<typename std::result_of<AccessFun(const std::vector<Data, Alloc>&, size_t)>::type> {
  /**
   * @brief Assert.
   */
  assert(dataArray.size() > 0);
  if (dataArray.size() > 1) {
    // Normal interpolation case
    int index = indexAlpha.first;
    scalar_t alpha = indexAlpha.second;
    /**
     * @brief Access Fun.
     * @param [in] dataArray: Data Array.
     * @param [in] index: Index.
     * @return Result value.
     */
    auto& lhs = AccessFun(dataArray, index);
    /**
     * @brief Access Fun.
     * @param [in] dataArray: Data Array.
     * @return Result value.
     */
    auto& rhs = AccessFun(dataArray, index + 1);
    if (AreSameSize(rhs, lhs)) {
      return alpha * lhs + (scalar_t(1.0) - alpha) * rhs;
    } else {
      return (alpha > 0.5) ? lhs : rhs;
    }
  } else {  // dataArray.size() == 1
    // Time vector has only 1 element -> Constant function
    /**
     * @brief Access Fun.
     * @param [in] dataArray: Data Array.
     * @return Result value.
     */
    return AccessFun(dataArray, 0);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename Data, class Alloc, class AccessFun>
auto Interpolate(scalar_t enquiryTime, const std::vector<scalar_t>& timeArray, const std::vector<Data, Alloc>& dataArray,
                 AccessFun accessFun) -> remove_cvref_t<typename std::result_of<AccessFun(const std::vector<Data, Alloc>&, size_t)>::type> {
  /**
   * @brief Interpolate.
   * @param [in] dataArray: Data Array.
   * @param [in] accessFun: Access Fun.
   * @return Result value.
   */
  return Interpolate(TimeSegment(enquiryTime, timeArray), dataArray, accessFun);
}

}  // namespace LinearInterpolation
}  // namespace bolza
