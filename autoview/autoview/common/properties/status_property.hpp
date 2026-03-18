/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#pragma once

#include "autoview/common/properties/property.hpp"

namespace autoview {
namespace common {
namespace properties {

class StatusProperty : public Property
{
  Q_OBJECT
public:
  enum Level { Ok = 0, Warn = 1, Error = 2 };
};

class StatusList : public Property
{
  Q_OBJECT
};

}  // namespace properties
}  // namespace common
}  // namespace autoview
