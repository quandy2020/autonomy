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

#include <iostream>
#include <utility>

#include <bolza/core/control/linear_controller.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearController::LinearController(const LinearController& other) : LinearController(other.timeStamp_, other.biasArray_, other.gainArray_) {
  deltaBiasArray_ = other.deltaBiasArray_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearController::LinearController(LinearController&& other) : ControllerBase(other) {
  swap(other, *this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearController& LinearController::operator=(LinearController rhs) {
  swap(rhs, *this);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearController* LinearController::Clone() const {
  return new LinearController(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearController::SetController(const scalar_array_t& controllerTime, const vector_array_t& controllerBias,
                                     const matrix_array_t& controllerGain) {
  timeStamp_ = controllerTime;
  biasArray_ = controllerBias;
  gainArray_ = controllerGain;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t LinearController::ComputeInput(scalar_t t, const vector_t& x) {
  const auto indexAlpha = LinearInterpolation::TimeSegment(t, timeStamp_);

  vector_t uff = LinearInterpolation::Interpolate(indexAlpha, biasArray_);
  const matrix_t k = LinearInterpolation::Interpolate(indexAlpha, gainArray_);

  uff.noalias() += k * x;
  return uff;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearController::Flatten(const scalar_array_t& timeArray, const std::vector<std::vector<float>*>& flatArray2) const {
  const auto timeSize = timeArray.size();
  const auto dataSize = flatArray2.size();

  if (timeSize != dataSize) {
    throw std::runtime_error("timeSize and dataSize must be equal in flatten method.");
  }

  for (size_t i = 0; i < timeSize; i++) {
    FlattenSingle(timeArray[i], *(flatArray2[i]));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearController::FlattenSingle(scalar_t time, std::vector<float>& flatArray) const {
  /* Serialized linear controller:
   * data = [
   *   // t0
   *   [
   *     uff[0], k[0, :],
   *     uff[1], k[1, :],
   *     ...
   *     uff[m], k[m, :]
   *   ],
   *
   *   ...
   *
   *   // tN
   *   [
   *     uff[0], k[0, :],
   *     uff[1], k[1, :],
   *     ...
   *     uff[m], k[m, :]
   *   ]
   * ]
   */

  const auto indexAlpha = LinearInterpolation::TimeSegment(time, timeStamp_);
  const vector_t uff = LinearInterpolation::Interpolate(indexAlpha, biasArray_);
  const matrix_t k = LinearInterpolation::Interpolate(indexAlpha, gainArray_);

  const size_t stateDim = k.cols();
  const size_t inputDim = k.rows();

  flatArray.clear();
  flatArray.resize(uff.size() + k.size());

  for (int i = 0; i < inputDim; i++) {  // i loops through rows of uff and k
    flatArray[i * (stateDim + 1) + 0] = static_cast<float>(uff(i));
    for (int j = 0; j < stateDim; j++) {  // j loops through cols of k
      flatArray[i * (stateDim + 1) + j + 1] = static_cast<float>(k(i, j));
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LinearController LinearController::UnFlatten(const size_array_t& stateDim, const size_array_t& inputDim, const scalar_array_t& timeArray,
                                             const std::vector<std::vector<float> const*>& flatArray2) {
  vector_array_t bias;
  matrix_array_t gain;

  bias.reserve(flatArray2.size());
  gain.reserve(flatArray2.size());

  for (int k = 0; k < timeArray.size(); k++) {  // loop through time
    if (flatArray2[k]->size() != inputDim[k] + inputDim[k] * stateDim[k]) {
      throw std::runtime_error("LinearController::UnFlatten received array of wrong length.");
    }

    bias.emplace_back(vector_t(inputDim[k]));
    gain.emplace_back(matrix_t(inputDim[k], stateDim[k]));

    const auto& arr = *flatArray2[k];
    for (int i = 0; i < inputDim[k]; i++) {  // loop through input dim
      bias.back()(i) = static_cast<scalar_t>(arr[i * (stateDim[k] + 1) + 0]);
      gain.back().row(i) = Eigen::Map<const Eigen::VectorXf>(&(arr[i * (stateDim[k] + 1) + 1]), stateDim[k]).cast<scalar_t>();
    }
  }
  return LinearController(timeArray, std::move(bias), std::move(gain));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearController::Concatenate(const ControllerBase* nextController, int index, int length) {
  if (const auto* nextLinCtrl = dynamic_cast<const LinearController*>(nextController)) {
    if (!timeStamp_.empty() && timeStamp_.back() > nextLinCtrl->timeStamp_.front()) {
      throw std::runtime_error("Concatenate requires that the nextController comes later in time.");
    }
    int last = index + length;
    timeStamp_.insert(timeStamp_.end(), nextLinCtrl->timeStamp_.begin() + index, nextLinCtrl->timeStamp_.begin() + last);
    biasArray_.insert(biasArray_.end(), nextLinCtrl->biasArray_.begin() + index, nextLinCtrl->biasArray_.begin() + last);
    gainArray_.insert(gainArray_.end(), nextLinCtrl->gainArray_.begin() + index, nextLinCtrl->gainArray_.begin() + last);

    // deltaBiasArray can be of different, incompatible size.
    if (last < nextLinCtrl->deltaBiasArray_.size()) {
      deltaBiasArray_.insert(deltaBiasArray_.end(), nextLinCtrl->deltaBiasArray_.begin() + index,
                             nextLinCtrl->deltaBiasArray_.begin() + last);
    } else {
      deltaBiasArray_.clear();
    }
  } else {
    throw std::runtime_error("Concatenate only works with controllers of the same type.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int LinearController::size() const {
  return timeStamp_.size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControllerType LinearController::GetType() const {
  return ControllerType::LINEAR;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearController::clear() {
  timeStamp_.clear();
  biasArray_.clear();
  deltaBiasArray_.clear();
  gainArray_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool LinearController::empty() const {
  return timeStamp_.empty();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearController::Display() const {
  std::cerr << *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearController::GetFeedbackGain(scalar_t time, matrix_t& gain) const {
  gain = LinearInterpolation::Interpolate(time, timeStamp_, gainArray_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LinearController::GetBias(scalar_t time, vector_t& bias) const {
  bias = LinearInterpolation::Interpolate(time, timeStamp_, biasArray_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_array_t LinearController::ControllerEventTimes() const {
  // simple controller case
  if (timeStamp_.size() < 2) {
    return scalar_array_t(0);
  }

  scalar_array_t eventTimes{0.0};
  scalar_t lastevent = timeStamp_.front();
  for (int i = 0; i < timeStamp_.size() - 1; i++) {
    bool eventDetected = timeStamp_[i + 1] - timeStamp_[i] < 2.0 * numeric_traits::WeakEpsilon<scalar_t>();
    const bool sufficientTimeSinceEvent = timeStamp_[i] - lastevent > 2.0 * numeric_traits::WeakEpsilon<scalar_t>();

    if (eventDetected && sufficientTimeSinceEvent) {  // push back event when event is detected
      eventTimes.push_back(timeStamp_[i]);
      lastevent = eventTimes.back();
    } else if (eventDetected) {
      // if event is detected to close to the last event, it is assumed that the earlier event was not an event
      // but was due to the refining steps taken in event detection
      // The last "detected event" is the time the event took place
      eventTimes.back() = timeStamp_[i];
      lastevent = eventTimes.back();
    }
  }
  return eventTimes;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void swap(LinearController& a, LinearController& b) noexcept {
  std::swap(a.timeStamp_, b.timeStamp_);
  std::swap(a.biasArray_, b.biasArray_);
  std::swap(a.deltaBiasArray_, b.deltaBiasArray_);
  std::swap(a.gainArray_, b.gainArray_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& out, const LinearController& controller) {
  for (size_t k = 0; k < controller.timeStamp_.size(); k++) {
    Out << "k: " << k << '\n';
    Out << "time: " << controller.timeStamp_[k] << '\n';
    Out << "bias: " << controller.biasArray_[k].transpose() << '\n';
    Out << "gain: " << controller.gainArray_[k] << '\n';
  }
  return out;
}

}  // namespace bolza
