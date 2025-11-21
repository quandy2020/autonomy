/**
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#include "autolink/examples/timer_component/timer_component_example.hpp"

#include "autolink/component/component.hpp"
#include "autolink/examples/proto/examples.pb.h"

bool TimerComponentSample::Init() {
  driver_writer_ = node_->CreateWriter<Driver>("/carstatus/channel");
  if (!driver_writer_) {
    AERROR << "Failed to create writer!";
    return false;
  }
  return true;
}

bool TimerComponentSample::Proc() {
  static int i = 0;
  auto out_msg = std::make_shared<Driver>();
  out_msg->set_msg_id(i++);
  if (driver_writer_->Write(out_msg)) {
    AINFO << "timer_component_example: Write drivermsg->"
          << out_msg->ShortDebugString();
  } else {
    AWARN << "Failed to write message";
  }
  return true;
}

AUTOLINK_REGISTER_COMPONENT(TimerComponentSample)
