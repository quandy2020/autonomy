/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/teleop/teleop_twist_utils.hpp"

namespace autoviz {
namespace teleop {
namespace {

void SetTwistField(automsgs::msgs::geometry_msgs::Twist* twist,
                   TeleopTwistField field, double value) {
  if (twist == nullptr) {
    return;
  }
  switch (field) {
    case TeleopTwistField::kLinearX:
      twist->mutable_linear()->set_x(value);
      break;
    case TeleopTwistField::kLinearY:
      twist->mutable_linear()->set_y(value);
      break;
    case TeleopTwistField::kLinearZ:
      twist->mutable_linear()->set_z(value);
      break;
    case TeleopTwistField::kAngularX:
      twist->mutable_angular()->set_x(value);
      break;
    case TeleopTwistField::kAngularY:
      twist->mutable_angular()->set_y(value);
      break;
    case TeleopTwistField::kAngularZ:
      twist->mutable_angular()->set_z(value);
      break;
  }
}

}  // namespace

automsgs::msgs::geometry_msgs::Twist ZeroTwist() {
  automsgs::msgs::geometry_msgs::Twist twist;
  twist.mutable_linear()->set_x(0.0);
  twist.mutable_linear()->set_y(0.0);
  twist.mutable_linear()->set_z(0.0);
  twist.mutable_angular()->set_x(0.0);
  twist.mutable_angular()->set_y(0.0);
  twist.mutable_angular()->set_z(0.0);
  return twist;
}

automsgs::msgs::geometry_msgs::Twist TwistFromButton(
    const TeleopButtonConfig& button) {
  automsgs::msgs::geometry_msgs::Twist twist = ZeroTwist();
  SetTwistField(&twist, button.field, button.value);
  return twist;
}

QString twistFieldLabel(TeleopTwistField field) {
  switch (field) {
    case TeleopTwistField::kLinearX:
      return QStringLiteral("linear.x");
    case TeleopTwistField::kLinearY:
      return QStringLiteral("linear.y");
    case TeleopTwistField::kLinearZ:
      return QStringLiteral("linear.z");
    case TeleopTwistField::kAngularX:
      return QStringLiteral("angular.x");
    case TeleopTwistField::kAngularY:
      return QStringLiteral("angular.y");
    case TeleopTwistField::kAngularZ:
      return QStringLiteral("angular.z");
  }
  return QStringLiteral("linear.x");
}

TeleopTwistField twistFieldFromIndex(int index) {
  switch (index) {
    case 1:
      return TeleopTwistField::kLinearY;
    case 2:
      return TeleopTwistField::kLinearZ;
    case 3:
      return TeleopTwistField::kAngularX;
    case 4:
      return TeleopTwistField::kAngularY;
    case 5:
      return TeleopTwistField::kAngularZ;
    default:
      return TeleopTwistField::kLinearX;
  }
}

}  // namespace teleop
}  // namespace autoviz
