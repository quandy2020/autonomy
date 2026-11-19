// Copyright 2026 The Openbot Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ORBRANDOM_H
#define ORBRANDOM_H

namespace ORB_SLAM3
{

/**
 * @brief Random source used by the ORB-SLAM3 RANSAC loops.
 *
 * The upstream code called DBoW2's DUtils generator. Argus builds the visual
 * frontend against FBoW, so the sampling contract lives here instead: seedable,
 * inclusive integer draws backed by the C++ standard library.
 */
class OrbRandom
{
public:
    OrbRandom() = delete;

    /**
     * @brief Reseeds the per-thread generator.
     * @param seed Deterministic seed value.
     */
    static void Seed(unsigned int seed);

    /**
     * @brief Draws a uniformly distributed integer.
     * @param min Inclusive lower bound.
     * @param max Inclusive upper bound.
     * @return Value in [min, max].
     */
    static int RandomInt(int min, int max);
};

} //namespace ORB_SLAM3

#endif // ORBRANDOM_H
