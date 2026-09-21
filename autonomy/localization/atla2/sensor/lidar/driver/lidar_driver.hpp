/*
 * Copyright 2026 The Openbot Authors
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

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "autonomy/localization/atla2/sensor/interface/types.hpp"

namespace autonomy::localization::atla2 {

enum class LidarModel {
  kGeneric = 0,
  kLivox,
  kOuster,
  kVelodyne,
};

inline LidarModel ParseLidarModel(const std::string& s) {
  if (s == "livox" || s == "mid360" || s == "avia") {
    return LidarModel::kLivox;
  }
  if (s == "ouster") {
    return LidarModel::kOuster;
  }
  if (s == "velodyne" || s == "vlp16" || s == "vlp32") {
    return LidarModel::kVelodyne;
  }
  return LidarModel::kGeneric;
}

//! Vendor-agnostic lidar driver interface (SDK / ROS bridge injects scans).
class LidarDriverBase {
 public:
  using Callback = std::function<void(const LidarScan&)>;

  virtual ~LidarDriverBase() = default;
  virtual LidarModel Model() const = 0;
  virtual bool Open(const std::string& /*uri*/) { return true; }
  virtual void Close() {}
  virtual void SetCallback(Callback cb) { cb_ = std::move(cb); }
  //! Inject a raw scan (offline / bridge).
  virtual void Feed(const LidarScan& scan) {
    if (cb_) {
      cb_(scan);
    }
  }

 protected:
  Callback cb_;
};

class GenericLidarDriver : public LidarDriverBase {
 public:
  LidarModel Model() const override { return LidarModel::kGeneric; }
};

class LivoxDriver : public LidarDriverBase {
 public:
  LidarModel Model() const override { return LidarModel::kLivox; }
  //! Livox often needs custom packet→point decode; Feed() used until SDK lands.
};

class OusterDriver : public LidarDriverBase {
 public:
  LidarModel Model() const override { return LidarModel::kOuster; }
};

class VelodyneDriver : public LidarDriverBase {
 public:
  LidarModel Model() const override { return LidarModel::kVelodyne; }
};

std::unique_ptr<LidarDriverBase> CreateLidarDriver(LidarModel model);

}  // namespace autonomy::localization::atla2
