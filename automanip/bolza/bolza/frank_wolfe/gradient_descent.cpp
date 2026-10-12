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

#include <bolza/frank_wolfe/gradient_descent.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GradientDescent::GradientDescent(const NLPSettings& nlpSettings)

    : nlpSettings_(nlpSettings), frankWolfeDescentDirectionPtr_(new FrankWolfeDescentDirection(nlpSettings.displayInfo_)) {
  CleanFmtDisplay_ = Eigen::IOFormat(3, 0, ", ", "\n", "[", "]");
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::GetCost(scalar_t& cost) const {
  cost = optimizedCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::GetParameters(vector_t& parameters) const {
  parameters = optimizedParameters_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::GetIterationsLog(scalar_array_t& iterationCost) const {
  iterationCost = iterationCost_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::OptimalSolutionID(size_t& optimizedID) const {
  optimizedID = optimizedID_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
NLPSettings& GradientDescent::NlpSettings() {
  return nlpSettings_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::LineSearch(const vector_t& parameters, const vector_t& gradient, NLPCost* costPtr, NLPConstraints* constraintsPtr,
                                 vector_t& optimizedParameters, scalar_t& optimizedCost, size_t& optimizedID,
                                 scalar_t& optimizedLearningRate) {
  scalar_t learningRate, contractionRate;
  if (nlpSettings_.useAscendingLineSearchNLP_ == true) {
    learningRate = nlpSettings_.minLearningRate_;
    contractionRate = 2.0;
  } else {
    learningRate = nlpSettings_.maxLearningRate_;
    contractionRate = 0.5;
  }

  // if no better learning rate is found
  optimizedLearningRate = 0.0;

  // return if gradient is zero
  if (gradient.isZero()) {
    if (nlpSettings_.displayInfo_) {
      std::cerr << "Line search is skipped since the gradient was zero." << std::endl;
    }
    return;
  }

  while (learningRate >= nlpSettings_.minLearningRate_) {
    // lineSerach parameter
    vector_t lsParameters = parameters - learningRate * gradient;

    // set the parameter
    size_t lsID = costPtr->SetCurrentParameter(lsParameters);

    // calculate cost function
    scalar_t lsCost;
    bool status = costPtr->GetCost(lsID, lsCost);

    // increment the number of function calls
    numFuntionCall_++;

    // skip it if status is not OK
    if (status == false) {
      // display
      if (nlpSettings_.displayInfo_) {
        std::cerr << "\t learningRate: " << learningRate;
        std::cerr << "\t cost: " << lsCost << " (rejected)" << std::endl;
      }
      learningRate *= contractionRate;
      continue;
    }

    // display
    if (nlpSettings_.displayInfo_) {
      scalar_t equalitySE(0.0), inequalitySE(0.0);
      if (constraintsPtr) {
        vector_t g, h;
        constraintsPtr->SetCurrentParameter(lsParameters);
        constraintsPtr->GetLinearEqualityConstraint(g);
        equalitySE = (g.size() > 0) ? g.squaredNorm() : 0.0;
        constraintsPtr->GetLinearInequalityConstraint(h);
        inequalitySE = (h.size() > 0) ? h.dot(h.cwiseMin(0.0)) : 0.0;
        std::cerr << "\t h: " << h.transpose().format(CleanFmtDisplay_) << std::endl;
      }
      std::cerr << "\t learningRate: " << learningRate;
      std::cerr << "\t cost: " << lsCost;
      std::cerr << "\t equality SE: " << equalitySE;
      std::cerr << "\t inequality SE: " << inequalitySE << std::endl;
    }

    // termination check
    if (nlpSettings_.useAscendingLineSearchNLP_ == true) {
      if (lsCost > optimizedCost_ * (1.0 - learningRate * 1e-3)) {
        break;
      }
      optimizedParameters = lsParameters;
      optimizedCost = lsCost;
      optimizedID = lsID;
      optimizedLearningRate = learningRate;

    } else {
      if (lsCost < optimizedCost_ * (1.0 - learningRate * 1e-3)) {
        optimizedParameters = lsParameters;
        optimizedCost = lsCost;
        optimizedID = lsID;
        optimizedLearningRate = learningRate;
        break;
      }
    }

    learningRate *= contractionRate;
  }  // end of while loop

  if (nlpSettings_.displayInfo_) {
    std::cerr << "Line search terminates with learning rate: " << optimizedLearningRate << std::endl;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GradientDescent::Run(const vector_t& initParameters, const vector_t& maxGradientInverse, NLPCost* costPtr,
                          NLPConstraints* constraintsPtr /* = nullptr*/) {
  // display
  if (nlpSettings_.displayInfo_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n+++++++++++++ NLP Solver is initialized ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
  }

  if (!costPtr) {
    throw std::runtime_error("Cost function pointer is null.");
  }

  numFuntionCall_ = 0;
  iterationCost_.clear();
  optimizedParameters_ = initParameters;

  // display
  if (nlpSettings_.displayInfo_) {
    std::cerr << "\n### Initial iteration\n";
  }

  // initial cost
  optimizedID_ = costPtr->SetCurrentParameter(optimizedParameters_);
  bool status = costPtr->GetCost(optimizedID_, optimizedCost_);
  iterationCost_.push_back(optimizedCost_);
  numFuntionCall_++;

  if (nlpSettings_.displayInfo_) {
    std::cerr << "cost:         " << optimizedCost_ << '\n';
    std::cerr << "parameters:   " << optimizedParameters_.transpose().format(CleanFmtDisplay_) << '\n';
  }

  size_t iteration = 0;
  bool isCostFunctionConverged = false;
  scalar_t relCost = std::numeric_limits<scalar_t>::max();
  scalar_t optimizedLearningRate = 0.0;

  while (iteration < nlpSettings_.maxIterations_ && isCostFunctionConverged == false) {
    // display
    if (nlpSettings_.displayInfo_) {
      std::cerr << "\n### Iteration " << iteration + 1 << '\n';
    }

    // compute the gradient
    scalar_t cachedCost = optimizedCost_;
    costPtr->GetCostDerivative(optimizedID_, optimizedGradient_);
    if (nlpSettings_.displayInfo_) {
      std::cerr << "Gradient:             " << optimizedGradient_.transpose().format(CleanFmtDisplay_) << '\n';
    }

    // compute the projected gradient
    if (constraintsPtr) {
      vector_t fwDescentDirection;
      frankWolfeDescentDirectionPtr_->Run(optimizedParameters_, optimizedGradient_, maxGradientInverse, constraintsPtr, fwDescentDirection);
      optimizedGradient_ = -fwDescentDirection;
      // display
      if (nlpSettings_.displayInfo_) {
        std::cerr << "Frank-Wolfe gradient: " << optimizedGradient_.transpose().format(CleanFmtDisplay_) << '\n';
      }
    }

    // line search
    LineSearch(optimizedParameters_, optimizedGradient_, costPtr, constraintsPtr, optimizedParameters_, optimizedCost_, optimizedID_,
               optimizedLearningRate);

    // loop variables
    relCost = std::fabs(optimizedCost_ - cachedCost);
    isCostFunctionConverged = (optimizedLearningRate == 0) || optimizedGradient_.isZero() || (relCost < nlpSettings_.minRelCost_);
    iterationCost_.push_back(optimizedCost_);
    iteration++;

    // display
    if (nlpSettings_.displayInfo_) {
      std::cerr << "cost:         " << optimizedCost_ << '\n';
      std::cerr << "parameters:   " << optimizedParameters_.transpose().format(CleanFmtDisplay_) << '\n';
      std::cerr << "solution ID:  " << optimizedID_ << '\n';
    }

  }  // end of while loop

  // display
  if (nlpSettings_.displayInfo_) {
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++";
    std::cerr << "\n++++++++++++++ NLP Solver is terminated ++++++++++++++";
    std::cerr << "\n++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
    if (isCostFunctionConverged) {
      if (optimizedGradient_.isZero()) {
        std::cerr << "NLP successfully terminates as gradient reduced to zero.\n";
      } else if (optimizedLearningRate == 0) {
        std::cerr << "NLP successfully terminates as learningRate reduced to zero.\n";
      } else {
        std::cerr << "NLP successfully terminates as cost relative change (relCost=" << relCost << ") reached to the minimum value.\n";
      }
    } else {
      std::cerr << "Maximum number of iterations has reached.\n";
    }
    std::cerr << "number of function calls:\t" << numFuntionCall_ << std::endl;
  }
}

}  // namespace bolza
