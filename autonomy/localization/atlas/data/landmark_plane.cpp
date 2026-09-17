#include "autonomy/localization/atlas/data/landmark_plane.hpp"

#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"

#include <limits>

#include <opencv2/imgproc.hpp>

namespace autonomy::localization::atlas::data {

namespace {

void label_to_display_color(const long label, float& r, float& g, float& b) {
    const uint32_t id = label <= 0 ? 0u : static_cast<uint32_t>(label);
    if (id == 0) {
        r = g = b = 0.5f;
        return;
    }
    const int hue = static_cast<int>((id * 47u) % 180u);
    const cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 210, 230));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    const auto c = bgr.at<cv::Vec3b>(0, 0);
    b = static_cast<float>(c[0]) / 255.f;
    g = static_cast<float>(c[1]) / 255.f;
    r = static_cast<float>(c[2]) / 255.f;
}

}  // namespace

landmark_plane::landmark_plane(const std::shared_ptr<keyframe>& ref_keyfrm, map_database* map_db)
    : ref_keyfrm_(ref_keyfrm), map_db_(map_db), best_error_(std::numeric_limits<double>::max()) {}

void landmark_plane::add_landmark(const std::shared_ptr<landmark>& lm) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    if (!lm || lm->will_be_erased()) {
        return;
    }
    landmarks_[lm->id_] = lm;
    lm->set_owning_plane(shared_from_this());
}

std::vector<std::shared_ptr<landmark>> landmark_plane::get_landmarks() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    std::vector<std::shared_ptr<landmark>> out;
    out.reserve(landmarks_.size());
    for (const auto& pair : landmarks_) {
        out.push_back(pair.second);
    }
    return out;
}

void landmark_plane::set_landmarks(const std::vector<std::shared_ptr<landmark>>& lms) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    landmarks_.clear();
    for (const auto& lm : lms) {
        if (lm && !lm->will_be_erased()) {
            landmarks_[lm->id_] = lm;
        }
    }
}

unsigned int landmark_plane::num_landmarks() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return landmarks_.size();
}

void landmark_plane::add_fit_sample(const Vec3_t& pos_w) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    fit_samples_.push_back(pos_w);
}

void landmark_plane::clear_fit_samples() {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    fit_samples_.clear();
}

const std::vector<Vec3_t>& landmark_plane::get_fit_samples() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return fit_samples_;
}

void landmark_plane::set_equation(const double a, const double b, const double c, const double d) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    normal_ << a, b, c;
    offset_ = d;
    abs_normal_ = normal_.norm();
}

void landmark_plane::get_equation(double& a, double& b, double& c, double& d) const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    a = normal_(0);
    b = normal_(1);
    c = normal_(2);
    d = offset_;
}

bool landmark_plane::is_valid() const { return valid_; }
void landmark_plane::set_valid() { valid_ = true; }
void landmark_plane::set_invalid() { valid_ = false; }

Vec3_t landmark_plane::get_normal() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return normal_;
}

double landmark_plane::get_offset() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return offset_;
}

double landmark_plane::calculate_distance(const Vec3_t& pos_w) const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    if (abs_normal_ <= 0.0) {
        return std::numeric_limits<double>::max();
    }
    return std::abs(normal_.dot(pos_w) + offset_) / abs_normal_;
}

void landmark_plane::set_landmarks_ownership() {
    for (const auto& lm : get_landmarks()) {
        if (lm) {
            lm->set_owning_plane(shared_from_this());
        }
    }
}

void landmark_plane::remove_landmarks_ownership() {
    for (const auto& lm : get_landmarks()) {
        if (lm) {
            lm->remove_owning_plane();
        }
    }
}

void landmark_plane::set_best_error(const double error) {
    std::lock_guard<std::mutex> lock(mtx_position_);
    best_error_ = error;
}
double landmark_plane::get_best_error() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return best_error_;
}

double landmark_plane::get_normal_norm() const {
    std::lock_guard<std::mutex> lock(mtx_position_);
    return abs_normal_;
}

void landmark_plane::merge(const std::shared_ptr<landmark_plane>& other) {
    if (!other) {
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_observations_);

    const auto other_landmarks = other->get_landmarks();
    other->remove_landmarks_ownership();

    for (const auto& lm : other_landmarks) {
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        landmarks_[lm->id_] = lm;
        lm->set_owning_plane(shared_from_this());
    }

    needs_refinement_ = true;
    other->set_invalid();
}

bool landmark_plane::need_refinement() const { return needs_refinement_; }
void landmark_plane::set_need_refinement() { needs_refinement_ = true; }
void landmark_plane::set_refinement_is_done() { needs_refinement_ = false; }

void landmark_plane::set_seg_label(const long label) {
    seg_label_ = label;
    if (label > 0) {
        label_to_display_color(label, display_r_, display_g_, display_b_);
        has_display_color_ = true;
    }
}

long landmark_plane::get_seg_label() const { return seg_label_; }

void landmark_plane::get_display_color(float& r, float& g, float& b) const {
    if (has_display_color_) {
        r = display_r_;
        g = display_g_;
        b = display_b_;
        return;
    }
    label_to_display_color(seg_label_, r, g, b);
}

nlohmann::json landmark_plane::to_json() const {
    double a = 0, b = 0, c = 0, d = 0;
    get_equation(a, b, c, d);

    nlohmann::json json;
    json["id"] = id_;
    json["a"] = a;
    json["b"] = b;
    json["c"] = c;
    json["d"] = d;
    json["valid"] = valid_;
    json["best_error"] = get_best_error();

    nlohmann::json lm_ids = nlohmann::json::array();
    for (const auto& lm : get_landmarks()) {
        if (lm && !lm->will_be_erased()) {
            lm_ids.push_back(lm->id_);
        }
    }
    json["landmark_ids"] = lm_ids;
    return json;
}

std::shared_ptr<landmark_plane> landmark_plane::from_json(const nlohmann::json& json,
                                                         map_database* map_db,
                                                         const std::shared_ptr<keyframe>& ref_keyfrm) {
    auto plane = std::make_shared<landmark_plane>(ref_keyfrm, map_db);
    plane->id_ = json.at("id").get<unsigned int>();
    plane->set_equation(json.at("a").get<double>(), json.at("b").get<double>(), json.at("c").get<double>(),
                        json.at("d").get<double>());
    if (json.at("valid").get<bool>()) {
        plane->set_valid();
    } else {
        plane->set_invalid();
    }
    plane->set_best_error(json.value("best_error", std::numeric_limits<double>::max()));
    return plane;
}

}  // namespace autonomy::localization::atlas::data
