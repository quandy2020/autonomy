//
// Created by rgrandia on 13.03.20.
//

#include "bolza/switched_model/foot_planner/spline_cpg.hpp"

namespace switched_model {

SplineCpg::SplineCpg(CubicSpline::Node liftOff, scalar_t midHeight, CubicSpline::Node touchDown)
    : SplineCpg(liftOff, CubicSpline::Node{(liftOff.time + touchDown.time) / 2.0, midHeight, 0.0}, touchDown) {}

SplineCpg::SplineCpg(CubicSpline::Node liftOff, CubicSpline::Node midNode, CubicSpline::Node touchDown)
    : midTime_(midNode.time), leftSpline_(liftOff, midNode), rightSpline_(midNode, touchDown) {}

scalar_t SplineCpg::Position(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.Position(time) : rightSpline_.Position(time);
}

scalar_t SplineCpg::Velocity(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.Velocity(time) : rightSpline_.Velocity(time);
}

scalar_t SplineCpg::Acceleration(scalar_t time) const {
  return (time < midTime_) ? leftSpline_.Acceleration(time) : rightSpline_.Acceleration(time);
}

scalar_t SplineCpg::StartTimeDerivative(scalar_t time) const {
  if (time <= midTime_) {
    return leftSpline_.StartTimeDerivative(time) + 0.5 * leftSpline_.StartTimeDerivative(time);
  } else {
    return 0.5 * rightSpline_.StartTimeDerivative(time);
  }
}

scalar_t SplineCpg::FinalTimeDerivative(scalar_t time) const {
  if (time <= midTime_) {
    return 0.5 * leftSpline_.FinalTimeDerivative(time);
  } else {
    return rightSpline_.FinalTimeDerivative(time) + 0.5 * rightSpline_.FinalTimeDerivative(time);
  }
}

}  // namespace switched_model
