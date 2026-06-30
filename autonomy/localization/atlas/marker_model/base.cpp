#include "autonomy/localization/atlas/marker_model/base.hpp"

#include <iostream>

#include <nlohmann/json.hpp>
#include "autolink/common/log.hpp"
namespace autonomy::localization::atlas {
namespace marker_model {

base::base(double width)
    : width_(width) {
    ADEBUG << "CONSTRUCT: marker_model::base";
    corners_pos_.resize(4);
    corners_pos_.at(0) << -width / 2.0, width / 2.0, 0.0;
    corners_pos_.at(1) << width / 2.0, width / 2.0, 0.0;
    corners_pos_.at(2) << width / 2.0, -width / 2.0, 0.0;
    corners_pos_.at(3) << -width / 2.0, -width / 2.0, 0.0;
}

base::~base() {
    ADEBUG << "DESTRUCT: marker_model::base";
}

nlohmann::json base::to_json() const {
    return {
        {"width", width_},
    };
}

std::ostream& operator<<(std::ostream& os, const base& params) {
    os << "- width: " << params.width_ << std::endl;
    os << "- corners_pos: " << std::endl;
    for (auto& corner_pos : params.corners_pos_) {
        os << "- " << corner_pos << std::endl;
    }
    return os;
}

} // namespace marker_model
}  // namespace autonomy::localization::atlas
