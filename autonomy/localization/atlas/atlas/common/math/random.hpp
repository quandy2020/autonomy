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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_ATLAS_COMMON_MATH_RANDOM_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_ATLAS_COMMON_MATH_RANDOM_HPP_

#include <cstddef>
#include <cstdint>
#include <random>
#include <stdexcept>

namespace autonomy::localization::atlas {

// Small project-owned random source used by RANSAC samplers. Keeping the
// generator in the caller avoids shared mutable random state.
class RandomGenerator
{
public:
    RandomGenerator() : generator_(std::random_device{}()) {}
    explicit RandomGenerator(std::uint32_t seed) : generator_(seed) {}

    std::size_t UniformIndex(std::size_t count) {
        if (count == 0) {
            throw std::invalid_argument("cannot sample an empty index range");
        }
        std::uniform_int_distribution<std::size_t> distribution(0, count - 1);
        return distribution(generator_);
    }

private:
    std::mt19937 generator_;
};

}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_ATLAS_COMMON_MATH_RANDOM_HPP_
