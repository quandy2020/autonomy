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

#include "OrbRandom.h"

#include <random>

namespace ORB_SLAM3
{

namespace
{

std::mt19937& Generator()
{
    static thread_local std::mt19937 generator(std::random_device{}());
    return generator;
}

} // namespace

void OrbRandom::Seed(unsigned int seed)
{
    Generator().seed(seed);
}

int OrbRandom::RandomInt(int min, int max)
{
    std::uniform_int_distribution<int> distribution(min, max);
    return distribution(Generator());
}

} //namespace ORB_SLAM3
