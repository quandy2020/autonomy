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
 * @file nlp_settings.hpp
 * @brief This structure contains the settings for the gradient-descent algorithm.
 */


#include <iostream>

namespace bolza {

/**
 * This structure contains the settings for the gradient-descent algorithm.
 */
class NLPSettings {
 public:
  NLPSettings()
      /**
       * @brief Display Info.
       * @param [in] true: True.
       * @return Result value.
       */
      : displayInfo_(true),
        maxIterations_(1000),
        minRelCost_(1e-6),
        maxLearningRate_(1.0),
        minLearningRate_(0.05),
        useAscendingLineSearchNLP_(true) {}

  /** This value determines to display the log output.*/
  bool displayInfo_;
  /** This value determines the maximum number of algorithm iterations.*/
  size_t maxIterations_;
  /** This value determines the termination condition based on the minimum relative changes of the cost.*/
  double minRelCost_;
  /** This value determines the maximum step size for the line search scheme.*/
  double maxLearningRate_;
  /** This value determines the minimum step size for the line search scheme.*/
  double minLearningRate_;
  /**
   * This value determines the line search scheme to be used. \n
   * - \b Ascending: The step size eventually increases from the minimum value to the maximum. \n
   * - \b Descending: The step size eventually decreases from the minimum value to the maximum.
   * */
  bool useAscendingLineSearchNLP_;
};

}  // namespace bolza
