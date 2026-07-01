#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/publish/frame_publisher.hpp"

#include <iomanip>
#include <algorithm>
#include <map>

#include <opencv2/imgproc.hpp>
#include "autolink/common/log.hpp"

namespace {

cv::Scalar turbo_color(double score) {
    const int idx = static_cast<int>(std::clamp(score, 0.0, 1.0) * 255);
    cv::Mat gray(1, 1, CV_8UC1, cv::Scalar(idx));
    cv::Mat color;
    cv::applyColorMap(gray, color, cv::COLORMAP_TURBO);
    const auto& pixel = color.at<cv::Vec3b>(0, 0);
    return cv::Scalar(pixel[0], pixel[1], pixel[2]);
}

} // namespace

namespace autonomy::localization::atlas {
namespace publish {

frame_publisher::frame_publisher(const std::shared_ptr<config>& cfg, data::map_database* map_db,
                                 const unsigned int img_width)
    : cfg_(cfg), map_db_(map_db), img_width_(img_width),
      img_(cv::Mat(480, img_width_, CV_8UC3, cv::Scalar(0, 0, 0))) {
    ADEBUG << "CONSTRUCT: publish::frame_publisher";
}

frame_publisher::~frame_publisher() {
    ADEBUG << "DESTRUCT: publish::frame_publisher";
}

cv::Mat frame_publisher::draw_frame() {
    cv::Mat img;
    tracker_state_t tracking_state;
    std::vector<cv::KeyPoint> curr_keypts;
    std::vector<data::marker2d> curr_mkrs2d;
    bool mapping_is_enabled;
    std::vector<std::shared_ptr<data::landmark>> curr_lms;

    // copy to avoid memory access conflict
    {
        std::lock_guard<std::mutex> lock(mtx_);

        img_.copyTo(img);

        tracking_state = tracking_state_;

        // copy tracking information
        curr_keypts = curr_keypts_;

        curr_mkrs2d = curr_mkrs2d_;

        mapping_is_enabled = mapping_is_enabled_;

        curr_lms = curr_lms_;
    }

    // resize image
    const float mag = (img_width_ < img_.cols) ? static_cast<float>(img_width_) / img.cols : 1.0;
    if (mag != 1.0) {
        cv::resize(img, img, cv::Size(), mag, mag, cv::INTER_NEAREST);
    }

    // to draw COLOR information
    if (img.channels() < 3) {
        cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }

    // draw keypoints
    unsigned int num_tracked = 0;
    switch (tracking_state) {
        case tracker_state_t::Tracking: {
            num_tracked = draw_tracked_points(img, curr_keypts, curr_lms, mapping_is_enabled, mag);
            break;
        }
        default: {
            break;
        }
    }

    // draw detected markers
    draw_markers2d(img, curr_mkrs2d, mag);

    ADEBUG << "num_tracked: " << num_tracked;

    return img;
}

cv::Mat frame_publisher::draw_frame_matches() {
    cv::Mat prev_img;
    cv::Mat curr_img;
    std::vector<cv::KeyPoint> prev_keypts;
    std::vector<cv::KeyPoint> curr_keypts;
    std::vector<std::shared_ptr<data::landmark>> prev_lms;
    std::vector<std::shared_ptr<data::landmark>> curr_lms;
    bool has_prev = false;

    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!has_prev_frame_ || prev_img_.empty() || img_.empty()) {
            return cv::Mat();
        }
        prev_img_.copyTo(prev_img);
        img_.copyTo(curr_img);
        prev_keypts = prev_keypts_;
        curr_keypts = curr_keypts_;
        prev_lms = prev_lms_;
        curr_lms = curr_lms_;
        has_prev = true;
    }

    if (!has_prev) {
        return cv::Mat();
    }

    const float mag = (img_width_ < prev_img.cols * 2)
                          ? static_cast<float>(img_width_) / static_cast<float>(prev_img.cols * 2)
                          : 1.0f;
    if (mag != 1.0f) {
        cv::resize(prev_img, prev_img, cv::Size(), mag, mag, cv::INTER_NEAREST);
        cv::resize(curr_img, curr_img, cv::Size(), mag, mag, cv::INTER_NEAREST);
    }

    if (prev_img.channels() < 3) {
        cv::cvtColor(prev_img, prev_img, cv::COLOR_GRAY2BGR);
    }
    if (curr_img.channels() < 3) {
        cv::cvtColor(curr_img, curr_img, cv::COLOR_GRAY2BGR);
    }

    const int panel_h = std::max(prev_img.rows, curr_img.rows);
    const int panel_w = std::max(prev_img.cols, curr_img.cols);
    cv::Mat prev_panel = cv::Mat::zeros(panel_h, panel_w, CV_8UC3);
    cv::Mat curr_panel = cv::Mat::zeros(panel_h, panel_w, CV_8UC3);
    prev_img.copyTo(prev_panel(cv::Rect(0, 0, prev_img.cols, prev_img.rows)));
    curr_img.copyTo(curr_panel(cv::Rect(0, 0, curr_img.cols, curr_img.rows)));

    cv::Mat canvas(panel_h, panel_w * 2, CV_8UC3, cv::Scalar(0, 0, 0));
    prev_panel.copyTo(canvas(cv::Rect(0, 0, panel_w, panel_h)));
    curr_panel.copyTo(canvas(cv::Rect(panel_w, 0, panel_w, panel_h)));

    cv::putText(canvas, "prev", {10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(220, 220, 220), 2);
    cv::putText(canvas, "curr", {panel_w + 10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(220, 220, 220), 2);

    draw_match_lines(canvas, panel_w, prev_keypts, prev_lms, curr_keypts, curr_lms, mag);

    return canvas;
}

std::string frame_publisher::get_tracking_state() {
    std::lock_guard<std::mutex> lock(mtx_);
    std::string state_str;
    if (tracking_state_ == tracker_state_t::Initializing) {
        state_str = "Initializing";
    }
    else if (tracking_state_ == tracker_state_t::Lost) {
        state_str = "Lost";
    }
    else if (tracking_state_ == tracker_state_t::Tracking) {
        state_str = "Tracking";
    }
    return state_str;
}

std::pair<std::vector<cv::KeyPoint>, std::vector<std::shared_ptr<data::landmark>>> frame_publisher::get_keypoints_and_landmarks() {
    std::lock_guard<std::mutex> lock(mtx_);

    return std::make_pair(curr_keypts_, curr_lms_);
}

std::vector<cv::KeyPoint> frame_publisher::get_keypoints() {
    std::lock_guard<std::mutex> lock(mtx_);
    return curr_keypts_;
}

bool frame_publisher::get_mapping_is_enabled() {
    std::lock_guard<std::mutex> lock(mtx_);
    return mapping_is_enabled_;
}

std::vector<std::shared_ptr<data::landmark>> frame_publisher::get_landmarks() {
    std::lock_guard<std::mutex> lock(mtx_);
    return curr_lms_;
}

cv::Mat frame_publisher::get_image() {
    cv::Mat img;
    img_.copyTo(img);
    return img;
}

double frame_publisher::get_tracking_time_elapsed_ms() {
    return tracking_time_elapsed_ms_;
}

double frame_publisher::get_extraction_time_elapsed_ms() {
    return extraction_time_elapsed_ms_;
}

unsigned int frame_publisher::draw_tracked_points(cv::Mat& img, const std::vector<cv::KeyPoint>& curr_keypts,
                                                  const std::vector<std::shared_ptr<data::landmark>>& curr_lms,
                                                  const bool mapping_is_enabled,
                                                  const float mag) const {
    constexpr float radius = 5;

    unsigned int num_tracked = 0;

    for (unsigned int i = 0; i < curr_keypts.size(); ++i) {
        const auto& lm = curr_lms.at(i);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        const cv::Point2f pt_begin{curr_keypts.at(i).pt.x * mag - radius, curr_keypts.at(i).pt.y * mag - radius};
        const cv::Point2f pt_end{curr_keypts.at(i).pt.x * mag + radius, curr_keypts.at(i).pt.y * mag + radius};

        double score = lm->get_observed_ratio();
        if (mapping_is_enabled) {
            cv::circle(img, curr_keypts.at(i).pt * mag, 2, turbo_color(score), -1);
        }
        else {
            cv::circle(img, curr_keypts.at(i).pt * mag, 2, localization_color_, -1);
        }

        ++num_tracked;
    }

    return num_tracked;
}

void frame_publisher::draw_markers2d(cv::Mat& img, const std::vector<data::marker2d>& mkrs2d, const float mag) {
    for (auto& mkr : mkrs2d) {
        std::string id_str = std::to_string(mkr.id_);

        double x_min = mkr.dist_corners_[0].x * mag;
        double y_min = mkr.dist_corners_[0].y * mag;

        for (size_t i = 0; i < 4; i++) {
            size_t j = (i + 1) % 4;
            const cv::Point2f pt_begin{mkr.dist_corners_[i].x * mag, mkr.dist_corners_[i].y * mag};
            const cv::Point2f pt_end{mkr.dist_corners_[j].x * mag, mkr.dist_corners_[j].y * mag};
            cv::line(img, pt_begin, pt_end, marker_color_, 2);

            if (pt_begin.x < x_min)
                x_min = pt_begin.x;
            if (pt_begin.y < y_min)
                y_min = pt_begin.y;
        }

        cv::putText(img, id_str, {(int)std::round(x_min), (int)std::round(y_min) - 4}, cv::FONT_HERSHEY_SIMPLEX, 1 * mag, marker_color_, 2);
    }
}

void frame_publisher::draw_match_lines(cv::Mat& canvas, int prev_panel_width,
                                       const std::vector<cv::KeyPoint>& prev_keypts,
                                       const std::vector<std::shared_ptr<data::landmark>>& prev_lms,
                                       const std::vector<cv::KeyPoint>& curr_keypts,
                                       const std::vector<std::shared_ptr<data::landmark>>& curr_lms,
                                       float mag) const {
    std::map<unsigned int, cv::Point2f> prev_lm_pts;
    const auto prev_size = std::min(prev_keypts.size(), prev_lms.size());
    for (std::size_t i = 0; i < prev_size; ++i) {
        const auto& lm = prev_lms.at(i);
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        prev_lm_pts[lm->id_] = prev_keypts.at(i).pt * mag;
    }

    const auto curr_size = std::min(curr_keypts.size(), curr_lms.size());
    for (std::size_t i = 0; i < curr_size; ++i) {
        const auto& lm = curr_lms.at(i);
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        const auto it = prev_lm_pts.find(lm->id_);
        if (it == prev_lm_pts.end()) {
            continue;
        }
        const cv::Point2f pt_prev = it->second;
        const cv::Point2f pt_curr = curr_keypts.at(i).pt * mag + cv::Point2f(static_cast<float>(prev_panel_width), 0.0f);
        cv::line(canvas, pt_prev, pt_curr, match_line_color_, 1, cv::LINE_AA);
        cv::circle(canvas, pt_prev, 2, match_point_color_, -1, cv::LINE_AA);
        cv::circle(canvas, pt_curr, 2, match_point_color_, -1, cv::LINE_AA);
    }
}

void frame_publisher::update(const std::vector<std::shared_ptr<data::landmark>>& curr_lms,
                             bool mapping_is_enabled,
                             tracker_state_t tracking_state,
                             std::vector<cv::KeyPoint>& keypts,
                             std::vector<data::marker2d>& mkrs2d,
                             const cv::Mat& img,
                             double tracking_time_elapsed_ms,
                             double extraction_time_elapsed_ms) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!img_.empty()) {
        img_.copyTo(prev_img_);
        prev_keypts_ = curr_keypts_;
        prev_lms_ = curr_lms_;
        has_prev_frame_ = true;
    }

    img.copyTo(img_);

    assert(keypts.size() == curr_lms.size());
    curr_keypts_ = keypts;
    tracking_time_elapsed_ms_ = tracking_time_elapsed_ms;
    extraction_time_elapsed_ms_ = extraction_time_elapsed_ms;
    mapping_is_enabled_ = mapping_is_enabled;
    tracking_state_ = tracking_state;
    curr_lms_ = curr_lms;
    curr_mkrs2d_ = mkrs2d;
}

} // namespace publish
}  // namespace autonomy::localization::atlas
