/*
 * SlidingWindowMathExpressionFilter.cpp
 *
 *  Created on: Aug 18, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "autonomy/common/logging.hpp"
#include "autonomy/map/grid_map/grid_map_filters/sliding_window_math_expression_filter.hpp"

using namespace filters;

namespace grid_map {

SlidingWindowMathExpressionFilter::SlidingWindowMathExpressionFilter()
    : windowSize_(3),
      useWindowLength_(false),
      windowLength_(0.0),
      isComputeEmptyCells_(true),
      edgeHandling_(SlidingWindowIterator::EdgeHandling::INSIDE) {}

SlidingWindowMathExpressionFilter::~SlidingWindowMathExpressionFilter() = default;

bool SlidingWindowMathExpressionFilter::configure() {
  if (!FilterBase::getParam(std::string("input_layer"), inputLayer_)) {
    AERROR << "SlidingWindowMathExpressionFilter did not find parameter 'input_layer'.";
    return false;
  }

  if (!FilterBase::getParam(std::string("output_layer"), outputLayer_)) {
    AERROR << "SlidingWindowMathExpressionFilter did not find parameter 'output_layer'.";
    return false;
  }

  if (!FilterBase::getParam(std::string("expression"), expression_)) {
    AERROR << "SlidingWindowMathExpressionFilter did not find parameter 'expression'.";
    return false;
  }

  if (!FilterBase::getParam(std::string("window_size"), windowSize_)) {
    if (FilterBase::getParam(std::string("window_length"), windowLength_)) {
      useWindowLength_ = true;
    }
  }

  if (!FilterBase::getParam(std::string("compute_empty_cells"), isComputeEmptyCells_)) {
    AERROR << "SlidingWindowMathExpressionFilter did not find parameter 'compute_empty_cells'.";
    return false;
  }

  std::string edgeHandlingMethod;
  if (!FilterBase::getParam(std::string("edge_handling"), edgeHandlingMethod)) {
    AERROR << "SlidingWindowMathExpressionFilter did not find parameter 'edge_handling'.";
    return false;
  }
  if (edgeHandlingMethod == "inside") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::INSIDE;
  } else if (edgeHandlingMethod == "crop") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::CROP;
  } else if (edgeHandlingMethod == "empty") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::EMPTY;
  } else if (edgeHandlingMethod == "mean") {
    edgeHandling_ = SlidingWindowIterator::EdgeHandling::MEAN;
  } else {
    AERROR << "SlidingWindowMathExpressionFilter did not find method '" << (edgeHandlingMethod.c_str()) << "' for edge handling.";
    return false;
  }

  // TODO(magnus): Can we make caching work with changing shared variable?
  //  parser_.setCacheExpressions(true);
  return true;
}

bool SlidingWindowMathExpressionFilter::update(const GridMap& mapIn, GridMap& mapOut) {
  mapOut = mapIn;
  mapOut.add(outputLayer_);
  Matrix& outputData = mapOut[outputLayer_];
  grid_map::SlidingWindowIterator iterator(mapIn, inputLayer_, edgeHandling_, windowSize_);
  if (useWindowLength_) {
    iterator.setWindowLength(mapIn, windowLength_);
  }
  for (; !iterator.isPastEnd(); ++iterator) {
    parser_.var(inputLayer_).setLocal(iterator.getData());
    EigenLab::Value<Eigen::MatrixXf> result(parser_.eval(expression_));
    if (result.matrix().cols() == 1 && result.matrix().rows() == 1) {
      outputData(iterator.getLinearIndex()) = result.matrix()(0);
    } else {
      AERROR << "SlidingWindowMathExpressionFilter could not apply filter because expression has to result in a scalar!";
    }
  }
  return true;
}

}  // namespace grid_map
