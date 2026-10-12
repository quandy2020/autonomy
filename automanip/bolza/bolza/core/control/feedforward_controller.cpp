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

#include <bolza/core/control/feedforward_controller.hpp>

namespace bolza {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController::FeedforwardController(const scalar_array_t& controllerTime, const vector_array_t& stateTrajectory,
                                             ControllerBase* controller) {
  if (controllerTime.size() != stateTrajectory.size()) {
    throw std::runtime_error("FeedforwardController Constructor: controllerTime and stateTrajectory sizes mismatch.");
  }

  timeStamp_ = controllerTime;
  uffArray_.clear();
  uffArray_.reserve(controllerTime.size());

  auto iTime = controllerTime.cbegin();
  auto iState = stateTrajectory.cbegin();

  for (; iTime != controllerTime.end() and iState != stateTrajectory.end(); ++iTime, ++iState) {
    uffArray_.emplace_back(controller->ComputeInput(*iTime, *iState));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController::FeedforwardController(const FeedforwardController& other)
    : FeedforwardController(other.timeStamp_, other.uffArray_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController::FeedforwardController(FeedforwardController&& other) : ControllerBase(other) {
  swap(other, *this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController& FeedforwardController::operator=(FeedforwardController rhs) {
  swap(rhs, *this);
  return *this;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::SetController(const scalar_array_t& controllerTime, const vector_array_t& controllerFeedforward) {
  timeStamp_ = controllerTime;
  uffArray_ = controllerFeedforward;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t FeedforwardController::ComputeInput(scalar_t t, const vector_t& x) {
  return LinearInterpolation::Interpolate(t, timeStamp_, uffArray_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::Flatten(const scalar_array_t& timeArray, const std::vector<std::vector<float>*>& flatArray2) const {
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
void FeedforwardController::FlattenSingle(scalar_t time, std::vector<float>& flatArray) const {
  /* Serialized feedforward controller:
   * data = [
   *   [ uff(t0)[:] ],
   *   [ uff(t1)[:] ],
   *   ...
   *   [ uff(tN)[:] ]
   * ]
   */

  const vector_t uff = LinearInterpolation::Interpolate(time, timeStamp_, uffArray_);

  flatArray = std::vector<float>(uff.data(), uff.data() + uff.rows());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController FeedforwardController::UnFlatten(const scalar_array_t& timeArray,
                                                       const std::vector<std::vector<float> const*>& flatArray2) {
  vector_array_t uffArray;
  uffArray.reserve(flatArray2.size());

  for (const auto& arr : flatArray2) {  // loop through time
    uffArray.emplace_back(Eigen::Map<const Eigen::VectorXf>(arr->data(), arr->size()).cast<scalar_t>());
  }
  return FeedforwardController(timeArray, std::move(uffArray));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::Concatenate(const ControllerBase* nextController, int index, int length) {
  if (auto nextFfwdCtrl = dynamic_cast<const FeedforwardController*>(nextController)) {
    if (!timeStamp_.empty() && timeStamp_.back() > nextFfwdCtrl->timeStamp_.front()) {
      throw std::runtime_error("Concatenate requires that the nextController comes later in time.");
    }
    int last = index + length;
    timeStamp_.insert(timeStamp_.end(), nextFfwdCtrl->timeStamp_.begin() + index, nextFfwdCtrl->timeStamp_.begin() + last);
    uffArray_.insert(uffArray_.end(), nextFfwdCtrl->uffArray_.begin() + index, nextFfwdCtrl->uffArray_.begin() + last);
  } else {
    throw std::runtime_error("Concatenate only works with controllers of the same type.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int FeedforwardController::size() const {
  return timeStamp_.size();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ControllerType FeedforwardController::GetType() const {
  return ControllerType::FEEDFORWARD;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::clear() {
  timeStamp_.clear();
  uffArray_.clear();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool FeedforwardController::empty() const {
  return timeStamp_.empty();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
FeedforwardController* FeedforwardController::Clone() const {
  return new FeedforwardController(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void FeedforwardController::Display() const {
  std::cerr << *this << '\n';
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void swap(FeedforwardController& a, FeedforwardController& b) noexcept {
  std::swap(a.timeStamp_, b.timeStamp_);
  std::swap(a.uffArray_, b.uffArray_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& out, const FeedforwardController& controller) {
  for (int i = 0; i < controller.timeStamp_.size(); i++) {
    out << "t\t" << controller.timeStamp_[i] << "\tu\t" << controller.uffArray_[i].transpose() << '\n';
  }
  return out;
}

}  // namespace bolza
