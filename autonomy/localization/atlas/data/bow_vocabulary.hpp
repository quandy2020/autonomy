/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_DATA_BOW_VOCABULARY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_DATA_BOW_VOCABULARY_HPP_

#include "autonomy/localization/atlas/data/bow_vocabulary_fwd.hpp"

#ifdef USE_DBOW2
#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedVocabulary.h>
#else
#include <fbow/vocabulary.h>
#endif // USE_DBOW2

namespace autonomy::localization::atlas {
namespace data {
namespace bow_vocabulary_util {

float score(bow_vocabulary* bow_vocab, const bow_vector& bow_vec1, const bow_vector& bow_vec2);
void compute_bow(bow_vocabulary* bow_vocab, const cv::Mat& descriptors, bow_vector& bow_vec,
                 bow_feature_vector& bow_feat_vec);
bow_vocabulary* load(std::string path);

} // namespace bow_vocabulary_util
} // namespace data
} // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_DATA_BOW_VOCABULARY_HPP_
