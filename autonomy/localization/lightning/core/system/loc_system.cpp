//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.hpp"

#include <glog/logging.h>

#include "common/options.hpp"
#include "core/localization/localization.hpp"
#include "io/yaml_io.hpp"

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {}

LocSystem::~LocSystem() {
    if (loc_) {
        loc_->Finish();
    }
}

bool LocSystem::Init(const std::string& yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);
    const std::string map_path = yaml.GetValue<std::string>("system", "map_path");
    const bool ret = loc_->Init(yaml_path, map_path);
    if (ret) {
        LOG(INFO) << "lightning loc system initialized, map=" << map_path;
    }
    return ret;
}

void LocSystem::SetInitPose(const SE3& pose) {
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();
    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr& imu) {
    if (loc_started_) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const CloudPtr& cloud) {
    if (loc_started_) {
        loc_->ProcessLidarMsg(cloud);
    }
}

}  // namespace lightning
