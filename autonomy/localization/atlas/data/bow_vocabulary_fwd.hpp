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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_DATA_BOW_VOCABULARY_FWD_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_DATA_BOW_VOCABULARY_FWD_HPP_

#ifdef USE_DBOW2
#include <opencv2/core/mat.hpp>

namespace DBoW2 {
class FORB;
// class FORB::TDescriptor; // can't do forward declaration for inner class.
template<class TDescriptor, class F>
class TemplatedVocabulary;
class BowVector;
class FeatureVector;
} // namespace DBoW2
#else
namespace fbow {
class Vocabulary;
struct BoWVector;
struct BoWFeatVector;
} // namespace fbow
#endif // USE_DBOW2

namespace autonomy::localization::atlas {
namespace data {

#ifdef USE_DBOW2

typedef DBoW2::TemplatedVocabulary<cv::Mat, DBoW2::FORB> bow_vocabulary;
typedef DBoW2::BowVector bow_vector;
typedef DBoW2::FeatureVector bow_feature_vector;

#else

typedef fbow::Vocabulary bow_vocabulary;
typedef fbow::BoWVector bow_vector;
typedef fbow::BoWFeatVector bow_feature_vector;

#endif // USE_DBOW2

} // namespace data
} // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_DATA_BOW_VOCABULARY_FWD_HPP_
