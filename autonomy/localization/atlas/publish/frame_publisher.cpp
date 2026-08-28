#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/landmark_line.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/publish/frame_publisher.hpp"

#include <iomanip>
#include <algorithm>
#include <map>
#include <sstream>

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

cv::Vec3b SegIdToColor(uint32_t id) {
    if (id == 0) {
        return {0, 0, 0};
    }
    const int hue = static_cast<int>((id * 47u) % 180u);
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, 200, 220));
    cv::Mat bgr;
    cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
    return bgr.at<cv::Vec3b>(0, 0);
}

uint32_t SegPixelId(const cv::Mat& seg, int x, int y) {
    if (seg.type() == CV_16UC1) {
        return seg.at<uint16_t>(y, x);
    }
    if (seg.type() == CV_8UC1) {
        return seg.at<uint8_t>(y, x);
    }
    const auto c = seg.at<cv::Vec3b>(y, x);
    return static_cast<uint32_t>(c[0]) | (static_cast<uint32_t>(c[1]) << 8) |
           (static_cast<uint32_t>(c[2]) << 16);
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
    bool loop_ba_running = false;
    bool loop_closure_active = false;
    std::vector<std::shared_ptr<data::landmark>> curr_lms;
    std::vector<cv::line_descriptor::KeyLine> curr_keylines;
    std::vector<bool> is_tracked_line;
    cv::Mat curr_seg_mask;
    double elapsed_ms = 0.0;

    // copy to avoid memory access conflict
    {
        std::lock_guard<std::mutex> lock(mtx_);

        img_.copyTo(img);

        tracking_state = tracking_state_;
        elapsed_ms = tracking_time_elapsed_ms_;

        // copy tracking information
        curr_keypts = curr_keypts_;

        curr_mkrs2d = curr_mkrs2d_;

        mapping_is_enabled = mapping_is_enabled_;
        loop_ba_running = loop_ba_running_;
        loop_closure_active = loop_closure_active_;

        curr_lms = curr_lms_;
        curr_keylines = curr_keylines_;
        is_tracked_line = is_tracked_line_;
        if (!curr_seg_mask_.empty()) {
            curr_seg_mask_.copyTo(curr_seg_mask);
        }
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

    unsigned int num_tracked_line = 0;
    if (!curr_keylines.empty()) {
        num_tracked_line =
            draw_tracked_lines(img, curr_keylines, is_tracked_line, mapping_is_enabled, mag);
    }

    draw_info_text(img, tracking_state, num_tracked, num_tracked_line, elapsed_ms,
                   mapping_is_enabled, loop_ba_running, loop_closure_active);

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
        (void)score;
        if (mapping_is_enabled) {
            cv::circle(img, curr_keypts.at(i).pt * mag, 2, mapping_color_, -1);
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

unsigned int frame_publisher::draw_tracked_lines(
    cv::Mat& img, const std::vector<cv::line_descriptor::KeyLine>& keylines,
    const std::vector<bool>& is_tracked_line, const bool mapping_is_enabled,
    const float mag) const {
    if (keylines.empty()) {
        return 0;
    }

    unsigned int num_tracked_line = 0;
    for (size_t i = 0; i < keylines.size(); ++i) {
        const auto& line = keylines.at(i);
        const cv::Point start_point(static_cast<int>(line.startPointX * mag),
                                    static_cast<int>(line.startPointY * mag));
        const cv::Point end_point(static_cast<int>(line.endPointX * mag),
                                  static_cast<int>(line.endPointY * mag));
        const bool tracked =
            i < is_tracked_line.size() && is_tracked_line.at(i);
        if (tracked) {
            ++num_tracked_line;
        }
        const cv::Scalar color =
            tracked ? lsd_line_tracked_color_ : lsd_line_untracked_color_;
        const int thickness = (tracked && mapping_is_enabled) ? 3 : 1;
        cv::line(img, start_point, end_point, color, thickness, cv::LINE_AA);
    }
    return num_tracked_line;
}

void frame_publisher::draw_info_text(cv::Mat& img, tracker_state_t tracking_state,
                                     const unsigned int num_tracked,
                                     const unsigned int num_tracked_line,
                                     const double elapsed_ms,
                                     const bool mapping_is_enabled,
                                     const bool loop_ba_running,
                                     const bool loop_closure_active) const {
    if (img.empty()) {
        return;
    }

    std::ostringstream ss;
    cv::Scalar text_color(255, 255, 255);
    if (loop_ba_running) {
        ss << "LOOP BA | ";
        text_color = cv::Scalar(0, 140, 255);
    } else if (loop_closure_active) {
        ss << "LOOP CLOSURE | ";
        text_color = cv::Scalar(0, 215, 255);
    } else if (tracking_state == tracker_state_t::Initializing) {
        ss << "INITIALIZE | ";
    } else if (tracking_state == tracker_state_t::Lost) {
        ss << "LOST | ";
    } else if (tracking_state == tracker_state_t::Tracking) {
        ss << (mapping_is_enabled ? "MAPPING" : "LOCALIZATION") << " | ";
    } else {
        ss << "UNKNOWN | ";
    }

    if (map_db_ != nullptr) {
        ss << "KF: " << map_db_->get_num_keyframes() << ", ";
        ss << "LM: " << map_db_->get_num_landmarks() << ", ";
        ss << "L3d: " << map_db_->get_all_landmarks_line().size() << ", ";
    }
    ss << "KP: " << num_tracked << ", ";
    ss << "KL: " << num_tracked_line << ", ";
    ss << "track time: " << std::fixed << std::setprecision(0) << elapsed_ms << "ms";

    int baseline = 0;
    const cv::Size text_size =
        cv::getTextSize(ss.str(), cv::FONT_HERSHEY_PLAIN, 1.0, 1, &baseline);
    cv::Mat overlay = img.clone();
    cv::rectangle(overlay, cv::Point(0, img.rows - text_size.height - 10),
                  cv::Point(img.cols, img.rows), cv::Scalar(0, 0, 0), -1);
    constexpr double alpha = 0.6;
    cv::addWeighted(overlay, alpha, img, 1.0 - alpha, 0, img);
    cv::putText(img, ss.str(), cv::Point(5, img.rows - 5), cv::FONT_HERSHEY_PLAIN, 1.0,
                text_color, 1, cv::LINE_AA);
}

cv::Mat frame_publisher::render_seg_mask_panel(const cv::Mat& seg_mask,
                                               const cv::Size& out_size,
                                               const float mag) const {
    if (seg_mask.empty() || out_size.width <= 0 || out_size.height <= 0) {
        return cv::Mat(out_size.height, out_size.width, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    cv::Mat seg = seg_mask;
    if (mag != 1.0f) {
        cv::resize(seg, seg, cv::Size(), mag, mag, cv::INTER_NEAREST);
    }
    if (seg.size() != out_size) {
        cv::resize(seg, seg, out_size, 0, 0, cv::INTER_NEAREST);
    }

    // Autosim / Habitat publish rgb8 semantic_to_rgb — use colors as-is (PLP draw_seg_mask).
    if (seg.type() == CV_8UC3) {
        return seg.clone();
    }

    cv::Mat panel(out_size.height, out_size.width, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int y = 0; y < seg.rows; ++y) {
        auto* dst = panel.ptr<cv::Vec3b>(y);
        for (int x = 0; x < seg.cols; ++x) {
            const uint32_t id = SegPixelId(seg, x, y);
            if (id != 0) {
                dst[x] = SegIdToColor(id);
            }
        }
    }
    return panel;
}

cv::Mat frame_publisher::draw_seg_mask() {
    cv::Mat panel;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!seg_mask_panel_.empty()) {
            seg_mask_panel_.copyTo(panel);
        }
    }
    return panel;
}

void frame_publisher::update(const std::vector<std::shared_ptr<data::landmark>>& curr_lms,
                             bool mapping_is_enabled,
                             tracker_state_t tracking_state,
                             std::vector<cv::KeyPoint>& keypts,
                             std::vector<data::marker2d>& mkrs2d,
                             const cv::Mat& img,
                             double tracking_time_elapsed_ms,
                             double extraction_time_elapsed_ms,
                             const std::vector<cv::line_descriptor::KeyLine>& keylines,
                             const std::vector<std::shared_ptr<data::landmark_line>>& curr_lms_line,
                             const cv::Mat& seg_mask,
                             const bool loop_ba_running,
                             const bool loop_closure_active) {
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
    loop_ba_running_ = loop_ba_running;
    loop_closure_active_ = loop_closure_active;
    tracking_state_ = tracking_state;
    curr_lms_ = curr_lms;
    curr_mkrs2d_ = mkrs2d;
    curr_keylines_ = keylines;
    is_tracked_line_.assign(keylines.size(), false);
    for (size_t i = 0; i < keylines.size() && i < curr_lms_line.size(); ++i) {
        const auto& lm_line = curr_lms_line.at(i);
        if (lm_line && !lm_line->will_be_erased() && lm_line->num_observations() > 0) {
            is_tracked_line_.at(i) = true;
        }
    }
    if (!seg_mask.empty()) {
        curr_seg_mask_.release();
        seg_mask.copyTo(curr_seg_mask_);
        const float mag =
            (img_width_ < img.cols) ? static_cast<float>(img_width_) / static_cast<float>(img.cols) : 1.0f;
        const cv::Size panel_size(static_cast<int>(std::round(img.cols * mag)),
                                  static_cast<int>(std::round(img.rows * mag)));
        seg_mask_panel_ = render_seg_mask_panel(curr_seg_mask_, panel_size, mag);
    } else {
        curr_seg_mask_.release();
        seg_mask_panel_.release();
    }
}

} // namespace publish
}  // namespace autonomy::localization::atlas
