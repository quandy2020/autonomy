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

#include <cstddef>
#include <limits>
#include <utility>

namespace bolza {

/******************************************************************************************************/

/**
 * @file compute_distance_transform.hpp
 * @brief compute distance transform (bolza).
 */

/******************************************************************************************************/
/******************************************************************************************************/
template <typename GetValFunc, typename SetValFunc, typename Scalar>
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
                              std::vector<std::size_t>& vBuffer, std::vector<Scalar>& zBuffer) {
  /**
   * @brief Constructs a ComputeDistanceTransform.
   * @param [in] numSamples: Num Samples.
   * @param [in] start: Start.
   * @param [in] end: End.
   * @param [in] vBuffer: V Buffer.
   * @param [in] zBuffer: Z Buffer.
   */
  ComputeDistanceTransform(
      numSamples, std::forward<GetValFunc>(getValue), std::forward<SetValFunc>(setValue), [&](std::size_t x, std::size_t ind) {}, start, end, vBuffer,
      zBuffer);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename GetValFunc, typename SetValFunc, typename SetImageIndexFunc, typename Scalar>
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
                              std::size_t start, std::size_t end, std::vector<std::size_t>& vBuffer, std::vector<Scalar>& zBuffer) {
  constexpr auto PlusInfinity = std::numeric_limits<Scalar>::max();
  constexpr auto MinusInfinity = std::numeric_limits<Scalar>::lowest();

  // make sure the buffer memories has the right size
  if (vBuffer.size() < numSamples) {
    /**
     * @brief Resize.
     * @param [in] numSamples: Num Samples.
     * @return Result value.
     */
    vBuffer.resize(numSamples);  // locations of parabolas in lower envelope
  }
  if (zBuffer.size() < numSamples + 1) {
    /**
     * @brief Resize.
     * @return Result value.
     */
    zBuffer.resize(numSamples + 1);  // locations of boundaries between parabolas
  }

  // initialization
  vBuffer[start] = start;
  zBuffer[start] = MinusInfinity;
  zBuffer[start + 1] = PlusInfinity;
  std::size_t k = start;  // index of rightmost parabola in lower envelope

  // compute lower envelope
  for (std::size_t q = start + 1; q < end; q++) {
    k++;  // compensates for first line of next do-while block

    Scalar s = 0;
    do {
      k--;
      // compute horizontal position of intersection between the parabola from q & the current lowest parabola
      s = ((GetValue(q) + q * q) - (GetValue(vBuffer[k]) + vBuffer[k] * vBuffer[k])) / (2 * q - 2 * vBuffer[k]);
    } while (s <= zBuffer[k]);

    k++;
    vBuffer[k] = q;
    zBuffer[k] = s;
    zBuffer[k + 1] = PlusInfinity;
  }  // end of for loop

  // fill in values of distance transform
  k = start;
  for (std::size_t q = start; q < end; q++) {
    while (zBuffer[k + 1] < q) {
      k++;
    }
    const std::size_t ind = vBuffer[k];
    const Scalar val = (q - ind) * (q - ind) + GetValue(ind);
    /**
     * @brief Constructs a SetValue.
     * @param [in] q: Q.
     * @param [in] val: Val.
     */
    SetValue(q, val);
    /**
     * @brief Constructs a SetImageIndex.
     * @param [in] q: Q.
     * @param [in] ind: Ind.
     */
    SetImageIndex(q, ind);
  }  // end of for loop
}

}  // namespace bolza
