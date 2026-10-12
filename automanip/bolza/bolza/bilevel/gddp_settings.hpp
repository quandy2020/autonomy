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
 * @file gddp_settings.hpp
 * @brief This structure contains the settings for the GDDP algorithm.
 */


#include <iostream>
#include <string>

#include <bolza/core/integration/integrator.hpp>
#include <bolza/core/misc/load_data.hpp>

namespace bolza {

/**
 * This structure contains the settings for the GDDP algorithm.
 */
class GDDPSettings {
 public:
  /**
   * Default constructor.
   */
  GDDPSettings()
      /**
       * @brief Display Info.
       * @return Result value.
       */
      : displayInfo_(false),
        checkNumericalStability_(true),
        warmStart_(false),
        useLQForDerivatives_(false),
        maxNumIterationForLQ_(10),
        tolGradientDescent_(1e-2),
        acceptableTolGradientDescent_(1e-1),
        maxIterationGradientDescent_(20),
        minLearningRateNLP_(0.05),
        maxLearningRateNLP_(1.0),
        minEventTimeDifference_(0.0),
        nThreads_(4),
        riccatiIntegratorType_(IntegratorType::ODE45),
        absTolODE_(1e-9),
        relTolODE_(1e-6),
        maxNumStepsPerSecond_(5000),
        timeStep_(1e-3) {}

  /**
   * This function loads the "GDDPSettings" variables from a config file. This file contains the settings for the SQL and OCS2 algorithms.
   * Here, we use the INFO format which was created specifically for the property tree library (refer to www.goo.gl/fV3yWA).
   *
   * It has the following format:	<br>
   * slq	<br>
   * {	<br>
   *   maxIterationGDDP        Value		<br>
   *   minLearningRateGDDP     Value		<br>
   *   maxLearningRateGDDP     Value		<br>
   *   minRelCostGDDP          Value		<br>
   *   (and so on for the other fields)	<br>
   * }	<br>
   *
   * If a value for a specific field is not defined it will set to the default value defined in "GDDPSettings".
   *
   * @param [in] filename: File name which contains the configuration data.
   * @param [in] fieldName: Field name which contains the configuration data (the default is slq).
   * @param [in] verbose: Flag to determine whether to print out the loaded settings or not (the default is true).
   */
  void LoadSettings(const std::string& filename, const std::string& fieldName = "gddp", bool verbose = true);

 public:
  /****************
   *** Variables **
   ****************/

  /** This value determines to display the log output DDP. */
  bool displayInfo_;

  /** Check the numerical stability of the algorithms for debugging purpose. */
  bool checkNumericalStability_;

  /** This value determines to use a warm starting scheme for calculating cost gradients w.r.t. switching times. */
  bool warmStart_;
  /** This value determines to use LQ-based method or sweeping method for calculating cost gradients w.r.t. switching times. */
  bool useLQForDerivatives_;
  /** Maximum number of iterations for LQ-based method  */
  size_t maxNumIterationForLQ_;

  double tolGradientDescent_;
  /** This value determines the termination condition for OCS2 based on the minimum relative changes of the cost.*/
  double acceptableTolGradientDescent_;
  /** This value determines the maximum number of iterations in OCS2 algorithm.*/
  size_t maxIterationGradientDescent_;
  /** This value determines the minimum step size for the line search scheme in OCS2 algorithm.*/
  double minLearningRateNLP_;
  /** This value determines the maximum step size for the line search scheme in OCS2 algorithm.*/
  double maxLearningRateNLP_;
  /** This value determines the minimum accepted difference between to consecutive events times.*/
  double minEventTimeDifference_;

  /** Number of threads used in the multi-threading scheme. */
  size_t nThreads_;

  /** Riccati integrator type. */
  IntegratorType riccatiIntegratorType_;
  /** This value determines the absolute tolerance error for ode solvers. */
  double absTolODE_;
  /** This value determines the relative tolerance error for ode solvers. */
  double relTolODE_;
  /** This value determines the maximum number of integration points per a second for ode solvers. */
  size_t maxNumStepsPerSecond_;
  /** The minimum integration time step */
  double timeStep_;

};  // end of GDDPSettings class

/**
 * @brief Load Settings.
 * @param [in] filename: Filename.
 * @return Result value.
 */
inline void GDDPSettings::LoadSettings(const std::string& filename, const std::string& fieldName /*= ilqr*/, bool verbose /*= true*/) {
  bolza::property_tree::ptree pt;
  /**
   * @brief Read info.
   * @param [in] filename: Filename.
   * @param [in] pt: Pt.
   * @return Result value.
   */
  bolza::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << std::endl << " #### GDDP Settings: " << std::endl;
    std::cerr << " #### =============================================================================" << std::endl;
  }

  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] displayInfo_: Display Info.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, displayInfo_, fieldName + ".displayInfo", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] checkNumericalStability_: Check Numerical Stability.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, checkNumericalStability_, fieldName + ".checkNumericalStability", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] warmStart_: Warm Start.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, warmStart_, fieldName + ".warmStart", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] useLQForDerivatives_: Use LQFor Derivatives.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, useLQForDerivatives_, fieldName + ".useLQForDerivatives", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] maxNumIterationForLQ_: Max Num Iteration For LQ.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, maxNumIterationForLQ_, fieldName + ".maxNumIterationForLQ", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] tolGradientDescent_: Tol Gradient Descent.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, tolGradientDescent_, fieldName + ".tolGradientDescent", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] acceptableTolGradientDescent_: Acceptable Tol Gradient Descent.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, acceptableTolGradientDescent_, fieldName + ".acceptableTolGradientDescent", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] maxIterationGradientDescent_: Max Iteration Gradient Descent.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, maxIterationGradientDescent_, fieldName + ".maxIterationGradientDescent", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] minLearningRateNLP_: Min Learning Rate NLP.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, minLearningRateNLP_, fieldName + ".minLearningRateNLP", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] maxLearningRateNLP_: Max Learning Rate NLP.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, maxLearningRateNLP_, fieldName + ".maxLearningRateNLP", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] minEventTimeDifference_: Min Event Time Difference.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, minEventTimeDifference_, fieldName + ".minEventTimeDifference", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] nThreads_: N Threads.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, nThreads_, fieldName + ".nThreads", verbose);

  /**
   * @brief To String.
   * @param [in] riccatiIntegratorType_: Riccati Integrator Type.
   * @return Result value.
   */
  std::string integratorName = integrator_type::ToString(riccatiIntegratorType_);  // keep default
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] integratorName: Integrator Name.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, integratorName, fieldName + ".RiccatiIntegratorType", verbose);
  /**
   * @brief From String.
   * @param [in] integratorName: Integrator Name.
   * @return Result value.
   */
  riccatiIntegratorType_ = integrator_type::FromString(integratorName);

  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] absTolODE_: Abs Tol ODE.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, absTolODE_, fieldName + ".AbsTolODE", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] relTolODE_: Rel Tol ODE.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, relTolODE_, fieldName + ".RelTolODE", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] maxNumStepsPerSecond_: Max Num Steps Per Second.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, maxNumStepsPerSecond_, fieldName + ".maxNumStepsPerSecond", verbose);
  /**
   * @brief Load Ptree Value.
   * @param [in] pt: Pt.
   * @param [in] timeStep_: Time Step.
   * @param [in] verbose: Verbose.
   * @return Result value.
   */
  loadData::LoadPtreeValue(pt, timeStep_, fieldName + ".timeStep", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }
}

}  // namespace bolza
