//
// Created by xiang on 25-9-8.
//

#ifndef LIGHTNING_LOC_SYSTEM_H
#define LIGHTNING_LOC_SYSTEM_H

#include <atomic>
#include <memory>
#include <string>

#include "common/eigen_types.hpp"
#include "common/imu.hpp"
#include "common/point_def.hpp"

namespace lightning {

namespace loc {
class Localization;
}

class LocSystem {
   public:
    struct Options {
        bool pub_tf_ = true;
    };

    explicit LocSystem(Options options);
    ~LocSystem();

    bool Init(const std::string& yaml_path);

    void SetInitPose(const SE3& pose);

    void ProcessIMU(const lightning::IMUPtr& imu);

    void ProcessLidar(const CloudPtr& cloud);

    loc::Localization* localization() const { return loc_.get(); }

   private:
    Options options_;

    std::shared_ptr<loc::Localization> loc_ = nullptr;

    std::atomic_bool loc_started_ = false;
};

}  // namespace lightning

#endif  // LIGHTNING_LOC_SYSTEM_H
