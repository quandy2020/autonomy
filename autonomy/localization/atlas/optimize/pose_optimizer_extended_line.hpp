#pragma once

#include "autonomy/localization/atlas/type.hpp"

#include <vector>

namespace autonomy::localization::atlas::data {
class frame;
}

namespace autonomy::localization::atlas::optimize {

class pose_optimizer_extended_line {
public:
    explicit pose_optimizer_extended_line(unsigned int num_trials = 4, unsigned int num_each_iter = 10);
    ~pose_optimizer_extended_line() = default;

    unsigned int optimize(data::frame& frm, Mat44_t& optimized_pose,
                          std::vector<bool>& outlier_flags,
                          std::vector<bool>& outlier_flags_line) const;

private:
    const unsigned int num_trials_;
    const unsigned int num_each_iter_;
};

}  // namespace autonomy::localization::atlas::optimize
