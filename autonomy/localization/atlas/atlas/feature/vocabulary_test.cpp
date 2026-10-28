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

#include "atlas/feature/vocabulary.hpp"

#include <limits>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas {
namespace {

TEST(VocabularyTest, ScoresPartiallyOverlappingL1Vectors) {
    const BowVector lhs{{1, 0.5}, {2, 0.5}};
    const BowVector rhs{{1, 0.5}, {3, 0.5}};

    EXPECT_DOUBLE_EQ(Vocabulary::Score(lhs, rhs), 0.5);
}

TEST(VocabularyTest, ScoresIdenticalL1Vectors) {
    const BowVector lhs{{1, 0.25}, {2, 0.75}};

    EXPECT_DOUBLE_EQ(Vocabulary::Score(lhs, lhs), 1.0);
}

TEST(VocabularyTest, ScoresDisjointL1VectorsAsZero) {
    const BowVector lhs{{1, 0.5}, {2, 0.5}};
    const BowVector rhs{{3, 0.5}, {4, 0.5}};

    EXPECT_DOUBLE_EQ(Vocabulary::Score(lhs, rhs), 0.0);
}

TEST(VocabularyTest, RejectsNonFiniteValues) {
    const BowVector lhs{{1, std::numeric_limits<double>::infinity()}};
    const BowVector rhs{{1, 1.0}};

    EXPECT_DOUBLE_EQ(Vocabulary::Score(lhs, rhs), 0.0);
}

}  // namespace
}  // namespace autonomy::localization::atlas
