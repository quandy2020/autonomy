#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/atlas/config.hpp"
#include "autonomy/localization/atlas/tracking_module.hpp"
#include "autonomy/localization/atlas/mapping_module.hpp"
#include "autonomy/localization/atlas/global_optimization_module.hpp"
#include "autonomy/localization/atlas/camera/camera_factory.hpp"
#include "autonomy/localization/atlas/data/camera_database.hpp"
#include "autonomy/localization/atlas/data/common.hpp"
#include "autonomy/localization/atlas/data/frame_observation.hpp"
#include "autonomy/localization/atlas/data/orb_params_database.hpp"
#include "autonomy/localization/atlas/data/map_database.hpp"
#include "autonomy/localization/atlas/data/bow_database.hpp"
#include "autonomy/localization/atlas/data/bow_vocabulary.hpp"
#include "autonomy/localization/atlas/data/marker2d.hpp"
#include "autonomy/localization/atlas/match/stereo.hpp"
#include "autonomy/localization/atlas/feature/orb_extractor.hpp"
#include "autonomy/localization/atlas/feature/line_extractor.hpp"
#include "autonomy/localization/atlas/plp/planar_mapping_module.hpp"
#include "autonomy/localization/atlas/io/trajectory_io.hpp"
#include "autonomy/localization/atlas/io/map_database_io_factory.hpp"
#include "autonomy/localization/atlas/publish/map_publisher.hpp"
#include "autonomy/localization/atlas/publish/frame_publisher.hpp"
#include "autonomy/localization/atlas/util/converter.hpp"
#include "autonomy/localization/atlas/util/image_converter.hpp"
#include "autonomy/localization/atlas/util/yaml.hpp"

#include <thread>
#include <cmath>
#include "autolink/common/log.hpp"

namespace autonomy::localization::atlas {
namespace {

float sample_depth_bilinear(const cv::Mat& depth, float u, float v) {
    if (depth.empty() || depth.type() != CV_32FC1) {
        return -1.f;
    }
    if (u < 0.f || v < 0.f || u >= static_cast<float>(depth.cols - 1) ||
        v >= static_cast<float>(depth.rows - 1)) {
        return -1.f;
    }
    const int x0 = static_cast<int>(std::floor(u));
    const int y0 = static_cast<int>(std::floor(v));
    const float dx = u - static_cast<float>(x0);
    const float dy = v - static_cast<float>(y0);
    const float d00 = depth.at<float>(y0, x0);
    const float d10 = depth.at<float>(y0, x0 + 1);
    const float d01 = depth.at<float>(y0 + 1, x0);
    const float d11 = depth.at<float>(y0 + 1, x0 + 1);
    if (d00 <= 0.f || d10 <= 0.f || d01 <= 0.f || d11 <= 0.f) {
        return -1.f;
    }
    const float d0 = d00 * (1.f - dx) + d10 * dx;
    const float d1 = d01 * (1.f - dx) + d11 * dx;
    return d0 * (1.f - dy) + d1 * dy;
}

}  // namespace

system::system(const std::shared_ptr<config>& cfg, const std::string& vocab_file_path)
    : cfg_(cfg) {
    ADEBUG << "CONSTRUCT: system";
    print_info();

    // load ORB vocabulary
    if (!vocab_file_path.empty()) {
        AINFO << "loading ORB vocabulary: " << vocab_file_path;
        bow_vocab_ = data::bow_vocabulary_util::load(vocab_file_path);
    }
    else {
        ADEBUG << "Running without vocabulary";
    }

    const auto system_params = util::yaml_optional_ref(cfg->yaml_node_, "System");

    camera_ = camera::camera_factory::create(util::yaml_optional_ref(cfg->yaml_node_, "Camera"));
    orb_params_ = new feature::orb_params(util::yaml_optional_ref(cfg->yaml_node_, "Feature"));
    AINFO << "load orb_params "" << orb_params_->name_ << """;

    // database
    cam_db_ = new data::camera_database();
    cam_db_->add_camera(camera_);
    map_db_ = new data::map_database(system_params["min_num_shared_lms"].as<unsigned int>(15));
    if (bow_vocab_) {
        bow_db_ = new data::bow_database(bow_vocab_);
    }
    orb_params_db_ = new data::orb_params_database();
    orb_params_db_->add_orb_params(orb_params_);

    // frame and map publisher
    frame_publisher_ = std::shared_ptr<publish::frame_publisher>(new publish::frame_publisher(cfg_, map_db_));
    map_publisher_ = std::shared_ptr<publish::map_publisher>(new publish::map_publisher(cfg_, map_db_));

    // map I/O
    auto map_format = system_params["map_format"].as<std::string>("msgpack");
    map_database_io_ = io::map_database_io_factory::create(map_format);

    // tracking module
    tracker_ = new tracking_module(cfg_, camera_, map_db_, bow_vocab_, bow_db_);
    // mapping module
    mapper_ = new mapping_module(util::yaml_optional_ref(cfg->yaml_node_, "Mapping"), map_db_, bow_db_, bow_vocab_);
    // global optimization module
    if (bow_db_ && bow_vocab_) {
        global_optimizer_ = new global_optimization_module(map_db_, bow_db_, bow_vocab_, cfg_->yaml_node_, camera_->setup_type_ != camera::setup_type_t::Monocular);
    }

    // preprocessing modules
    const auto preprocessing_params = util::yaml_optional_ref(cfg->yaml_node_, "Preprocessing");
    if (camera_->setup_type_ == camera::setup_type_t::RGBD) {
        depthmap_factor_ = preprocessing_params["depthmap_factor"].as<double>(depthmap_factor_);
        if (depthmap_factor_ < 0.) {
            throw std::runtime_error("depthmap_factor must be greater than 0");
        }
    }
    auto mask_rectangles = util::get_rectangles(preprocessing_params["mask_rectangles"]);

    const auto min_size = preprocessing_params["min_size"].as<unsigned int>(800);
    const auto desc_type_str = preprocessing_params["descriptor_type"].as<std::string>("ORB");
    const auto desc_type = feature::descriptor_type_from_string(desc_type_str);
    extractor_left_ = new feature::orb_extractor(orb_params_, min_size, desc_type, mask_rectangles);
    if (camera_->setup_type_ == camera::setup_type_t::Stereo) {
        extractor_right_ = new feature::orb_extractor(orb_params_, min_size, desc_type, mask_rectangles);
    }

    num_grid_cols_ = preprocessing_params["num_grid_cols"].as<unsigned int>(64);
    num_grid_rows_ = preprocessing_params["num_grid_rows"].as<unsigned int>(48);

    plp_options_ = plp::Options::FromYaml(cfg->yaml_node_);
    if (plp_options_.enabled && plp_options_.use_line_tracking) {
        line_extractor_ = new feature::line_extractor(camera_, plp_options_);
        map_db_->set_use_line_tracking(true);
        tracker_->configure_plp_line_tracking();
        AINFO << "Atlas PLP: line tracking enabled (LSD/LBD)";
    }
    if (plp_options_.enabled && plp_options_.use_plane_mapping) {
        map_db_->set_use_plane_mapping(true);
        const bool is_monocular = (camera_->setup_type_ == camera::setup_type_t::Monocular);
        planar_mapper_ = std::make_unique<plp::planar_mapping_module>(map_db_, is_monocular, plp_options_);
        mapper_->set_planar_mapping_module(planar_mapper_.get());
        AINFO << "Atlas PLP: plane mapping enabled (requires instance seg mask)";
    }

    // connect modules each other
    tracker_->set_mapping_module(mapper_);
    mapper_->set_tracking_module(tracker_);
    if (global_optimizer_) {
        tracker_->set_global_optimization_module(global_optimizer_);
        mapper_->set_global_optimization_module(global_optimizer_);
        global_optimizer_->set_tracking_module(tracker_);
        global_optimizer_->set_mapping_module(mapper_);
    }
}

system::~system() {
    global_optimization_thread_.reset(nullptr);
    if (global_optimizer_) {
        delete global_optimizer_;
        global_optimizer_ = nullptr;
    }

    mapping_thread_.reset(nullptr);
    delete mapper_;
    mapper_ = nullptr;

    delete tracker_;
    tracker_ = nullptr;

    delete bow_db_;
    bow_db_ = nullptr;
    delete map_db_;
    map_db_ = nullptr;
    delete cam_db_;
    cam_db_ = nullptr;
    if (bow_vocab_) {
        delete bow_vocab_;
        bow_vocab_ = nullptr;
    }

    delete extractor_left_;
    extractor_left_ = nullptr;
    delete extractor_right_;
    extractor_right_ = nullptr;
    delete line_extractor_;
    line_extractor_ = nullptr;

    delete orb_params_db_;
    orb_params_db_ = nullptr;

    ADEBUG << "DESTRUCT: system";
}

void system::print_info() {
    std::ostringstream message_stream;

    message_stream << std::endl;
    message_stream << "original version of OpenVSLAM," << std::endl;
    message_stream << "Copyright (C) 2019," << std::endl;
    message_stream << "National Institute of Advanced Industrial Science and Technology (AIST)" << std::endl;
    message_stream << "All rights reserved." << std::endl;
    message_stream << "stella_vslam (the changes after forking from OpenVSLAM)," << std::endl;
    message_stream << "Copyright (C) 2022, stella-cv, All rights reserved." << std::endl;
    message_stream << std::endl;
    message_stream << "This is free software," << std::endl;
    message_stream << "and you are welcome to redistribute it under certain conditions." << std::endl;
    message_stream << "See the LICENSE file." << std::endl;
    message_stream << std::endl;

    // show configuration
    message_stream << *cfg_ << std::endl;

    AINFO << message_stream.str();
}

void system::startup(const bool need_initialize) {
    AINFO << "startup SLAM system";
    system_is_running_ = true;

    if (!need_initialize) {
        tracker_->tracking_state_ = tracker_state_t::Lost;
    }

    mapping_thread_ = std::unique_ptr<std::thread>(new std::thread(&autonomy::localization::atlas::mapping_module::run, mapper_));
    if (global_optimizer_) {
        global_optimization_thread_ = std::unique_ptr<std::thread>(new std::thread(&autonomy::localization::atlas::global_optimization_module::run, global_optimizer_));
    }
}

void system::shutdown() {
    if (planar_mapper_ && map_db_->use_plane_mapping()) {
        pause_other_threads();
        planar_mapper_->refinement();
        resume_other_threads();
    }

    // terminate the other threads
    if (global_optimizer_) {
        auto future_mapper_terminate = mapper_->async_terminate();
        auto future_global_optimizer_terminate = global_optimizer_->async_terminate();
        future_mapper_terminate.get();
        future_global_optimizer_terminate.get();
    }
    else {
        auto future_mapper_terminate = mapper_->async_terminate();
        future_mapper_terminate.get();
    }

    // wait until the threads stop
    mapping_thread_->join();
    if (global_optimization_thread_) {
        global_optimization_thread_->join();
    }

    AINFO << "shutdown SLAM system";
    system_is_running_ = false;
}

void system::save_frame_trajectory(const std::string& path, const std::string& format) const {
    pause_other_threads();
    io::trajectory_io trajectory_io(map_db_);
    trajectory_io.save_frame_trajectory(path, format);
    resume_other_threads();
}

void system::save_keyframe_trajectory(const std::string& path, const std::string& format) const {
    pause_other_threads();
    io::trajectory_io trajectory_io(map_db_);
    trajectory_io.save_keyframe_trajectory(path, format);
    resume_other_threads();
}

bool system::load_map_database(const std::string& path) const {
    pause_other_threads();
    ADEBUG << "load_map_database: " << path;
    bool ok = map_database_io_->load(path, cam_db_, orb_params_db_, map_db_, bow_db_, bow_vocab_);
    auto keyfrms = map_db_->get_all_keyframes();

    for (const auto& keyfrm : keyfrms) {
        keyfrm->frm_obs_.num_grid_cols_ = num_grid_cols_;
        keyfrm->frm_obs_.num_grid_rows_ = num_grid_rows_;
        data::assign_keypoints_to_grid(keyfrm->camera_, keyfrm->frm_obs_.undist_keypts_, keyfrm->frm_obs_.keypt_indices_in_cells_,
                                       keyfrm->frm_obs_.num_grid_cols_, keyfrm->frm_obs_.num_grid_rows_);
    }

    resume_other_threads();
    return ok;
}

bool system::save_map_database(const std::string& path) const {
    pause_other_threads();
    ADEBUG << "save_map_database: " << path;
    bool ok = map_database_io_->save(path, cam_db_, orb_params_db_, map_db_);
    resume_other_threads();
    return ok;
}

const std::shared_ptr<publish::map_publisher> system::get_map_publisher() const {
    return map_publisher_;
}

const std::shared_ptr<publish::frame_publisher> system::get_frame_publisher() const {
    return frame_publisher_;
}

void system::enable_mapping_module() {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    if (!system_is_running_) {
        AFATAL << "please call system::enable_mapping_module() after system::startup()";
    }
    // resume the mapping module
    mapper_->resume();
}

void system::disable_mapping_module() {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    if (!system_is_running_) {
        AFATAL << "please call system::disable_mapping_module() after system::startup()";
    }
    // pause the mapping module
    auto future_pause = mapper_->async_pause();
    // wait until it stops
    future_pause.get();
}

bool system::mapping_module_is_enabled() const {
    return !mapper_->is_paused();
}

void system::enable_loop_detector() {
    std::lock_guard<std::mutex> lock(mtx_loop_detector_);
    if (global_optimizer_) {
        global_optimizer_->enable_loop_detector();
    }
}

void system::disable_loop_detector() {
    std::lock_guard<std::mutex> lock(mtx_loop_detector_);
    if (global_optimizer_) {
        global_optimizer_->disable_loop_detector();
    }
}

bool system::loop_detector_is_enabled() const {
    return global_optimizer_ && global_optimizer_->loop_detector_is_enabled();
}

bool system::request_loop_closure(int keyfrm1_id, int keyfrm2_id) {
    return global_optimizer_ && global_optimizer_->request_loop_closure(keyfrm1_id, keyfrm2_id);
}

bool system::loop_BA_is_running() const {
    return global_optimizer_ && global_optimizer_->loop_BA_is_running();
}

void system::abort_loop_BA() {
    if (global_optimizer_) {
        global_optimizer_->abort_loop_BA();
    }
}

void system::enable_temporal_mapping() {
    map_db_->set_fixed_keyframe_id_threshold();
}

data::frame system::create_monocular_frame(const cv::Mat& img, const double timestamp, const cv::Mat& mask) {
    // color conversion
    if (!camera_->is_valid_shape(img)) {
        AWARN << "preprocess: Input image size is invalid";
    }
    cv::Mat img_gray = img;
    util::convert_to_grayscale(img_gray, camera_->color_order_);

    data::frame_observation frm_obs;
    data::line_frame_observation line_obs;
    std::thread line_thread;

    keypts_.clear();
    if (line_extractor_) {
        line_thread = std::thread([this, &img_gray, &line_obs]() {
            line_extractor_->extract(img_gray, &line_obs);
        });
    }
    extractor_left_->extract(img_gray, mask, keypts_, frm_obs.descriptors_);
    if (line_thread.joinable()) {
        line_thread.join();
    }
    if (keypts_.empty()) {
        AWARN << "preprocess: cannot extract any keypoints";
    }

    // Undistort keypoints
    camera_->undistort_keypoints(keypts_, frm_obs.undist_keypts_);

    // Convert to bearing vector
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // Assign all the keypoints into grid
    frm_obs.num_grid_cols_ = num_grid_cols_;
    frm_obs.num_grid_rows_ = num_grid_rows_;
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_,
                                   frm_obs.num_grid_cols_, frm_obs.num_grid_rows_);

    data::frame frm(next_frame_id_++, timestamp, camera_, orb_params_, frm_obs, {});
    frm.line_obs_ = std::move(line_obs);
    return frm;
}

data::frame system::create_stereo_frame(const cv::Mat& left_img, const cv::Mat& right_img, const double timestamp, const cv::Mat& mask) {
    // color conversion
    if (!camera_->is_valid_shape(left_img)) {
        AWARN << "preprocess: Input image size is invalid";
    }
    if (!camera_->is_valid_shape(right_img)) {
        AWARN << "preprocess: Input image size is invalid";
    }
    cv::Mat img_gray = left_img;
    cv::Mat right_img_gray = right_img;
    util::convert_to_grayscale(img_gray, camera_->color_order_);
    util::convert_to_grayscale(right_img_gray, camera_->color_order_);

    data::frame_observation frm_obs;
    data::line_frame_observation line_obs;
    //! keypoints of stereo right image
    std::vector<cv::KeyPoint> keypts_right;
    //! ORB descriptors of stereo right image
    cv::Mat descriptors_right;

    keypts_.clear();
    std::thread thread_left([this, &frm_obs, &img_gray, &mask]() {
        extractor_left_->extract(img_gray, mask, keypts_, frm_obs.descriptors_);
    });
    std::thread thread_right([this, &frm_obs, &right_img_gray, &mask, &keypts_right, &descriptors_right]() {
        extractor_right_->extract(right_img_gray, mask, keypts_right, descriptors_right);
    });
    std::thread line_thread;
    if (line_extractor_) {
        line_thread = std::thread([this, &img_gray, &line_obs]() {
            line_extractor_->extract(img_gray, &line_obs);
        });
    }
    thread_left.join();
    thread_right.join();
    if (line_thread.joinable()) {
        line_thread.join();
    }
    if (keypts_.empty()) {
        AWARN << "preprocess: cannot extract any keypoints";
    }

    // Undistort keypoints
    camera_->undistort_keypoints(keypts_, frm_obs.undist_keypts_);

    // Estimate depth with stereo match
    match::stereo stereo_matcher(extractor_left_->image_pyramid_, extractor_right_->image_pyramid_,
                                 keypts_, keypts_right, frm_obs.descriptors_, descriptors_right,
                                 orb_params_->scale_factors_, orb_params_->inv_scale_factors_,
                                 camera_->focal_x_baseline_, camera_->true_baseline_);
    stereo_matcher.compute(frm_obs.stereo_x_right_, frm_obs.depths_);

    // Convert to bearing vector
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // Assign all the keypoints into grid
    frm_obs.num_grid_cols_ = num_grid_cols_;
    frm_obs.num_grid_rows_ = num_grid_rows_;
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_,
                                   frm_obs.num_grid_cols_, frm_obs.num_grid_rows_);

    data::frame frm(next_frame_id_++, timestamp, camera_, orb_params_, frm_obs, {});
    frm.line_obs_ = std::move(line_obs);
    return frm;
}

data::frame system::create_RGBD_frame(const cv::Mat& rgb_img, const cv::Mat& depthmap, const double timestamp,
                                      const cv::Mat& mask, const cv::Mat& seg_mask) {
    // color and depth scale conversion
    if (!camera_->is_valid_shape(rgb_img)) {
        AWARN << "preprocess: Input image size is invalid";
    }
    if (!camera_->is_valid_shape(depthmap)) {
        AWARN << "preprocess: Input image size is invalid";
    }
    cv::Mat img_gray = rgb_img;
    cv::Mat img_depth = depthmap;
    util::convert_to_grayscale(img_gray, camera_->color_order_);
    util::convert_to_true_depth(img_depth, depthmap_factor_);

    data::frame_observation frm_obs;
    data::line_frame_observation line_obs;
    std::thread line_thread;

    keypts_.clear();
    if (line_extractor_) {
        line_thread = std::thread([this, &img_gray, &line_obs]() {
            line_extractor_->extract(img_gray, &line_obs);
        });
    }
    extractor_left_->extract(img_gray, mask, keypts_, frm_obs.descriptors_);
    if (line_thread.joinable()) {
        line_thread.join();
    }
    if (keypts_.empty()) {
        AWARN << "preprocess: cannot extract any keypoints";
    }

    // Undistort keypoints
    camera_->undistort_keypoints(keypts_, frm_obs.undist_keypts_);

    // Calculate disparity from depth
    // Initialize with invalid value
    frm_obs.stereo_x_right_ = std::vector<float>(frm_obs.undist_keypts_.size(), -1);
    frm_obs.depths_ = std::vector<float>(frm_obs.undist_keypts_.size(), -1);

    for (unsigned int idx = 0; idx < frm_obs.undist_keypts_.size(); idx++) {
        const auto& keypt = keypts_.at(idx);
        const auto& undist_keypt = frm_obs.undist_keypts_.at(idx);

        const float x = keypt.pt.x;
        const float y = keypt.pt.y;

        const float depth = img_depth.at<float>(y, x);

        if (depth <= 0) {
            continue;
        }

        frm_obs.depths_.at(idx) = depth;
        frm_obs.stereo_x_right_.at(idx) = undist_keypt.pt.x - camera_->focal_x_baseline_ / depth;
    }

    // Convert to bearing vector
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // Assign all the keypoints into grid
    frm_obs.num_grid_cols_ = num_grid_cols_;
    frm_obs.num_grid_rows_ = num_grid_rows_;
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_,
                                   frm_obs.num_grid_cols_, frm_obs.num_grid_rows_);

    data::frame frm(next_frame_id_++, timestamp, camera_, orb_params_, frm_obs, {});
    frm.line_obs_ = std::move(line_obs);
    if (!frm.line_obs_.empty() && !img_depth.empty()) {
        frm.line_obs_.depths_start.assign(frm.line_obs_.keylines.size(), -1.f);
        frm.line_obs_.depths_end.assign(frm.line_obs_.keylines.size(), -1.f);
        frm.line_obs_.stereo_x_right.assign(frm.line_obs_.keylines.size(), -1.f);
        for (size_t idx_l = 0; idx_l < frm.line_obs_.keylines.size(); ++idx_l) {
            const auto& keyline = frm.line_obs_.keylines.at(idx_l);
            const auto& sp = keyline.getStartPoint();
            const auto& ep = keyline.getEndPoint();
            const float depth_sp = sample_depth_bilinear(img_depth, sp.x, sp.y);
            const float depth_ep = sample_depth_bilinear(img_depth, ep.x, ep.y);
            if (depth_sp <= 0.0f || depth_ep <= 0.0f) {
                continue;
            }
            frm.line_obs_.depths_start.at(idx_l) = depth_sp;
            frm.line_obs_.depths_end.at(idx_l) = depth_ep;
            frm.line_obs_.stereo_x_right.at(idx_l) =
                sp.x - camera_->focal_x_baseline_ / depth_sp;
        }
    }
    if (!seg_mask.empty()) {
        frm.img_seg_mask_ = seg_mask.clone();
    }
    frm.depth_map_ = img_depth.clone();
    return frm;
}

std::shared_ptr<Mat44_t> system::feed_monocular_frame(const cv::Mat& img, const double timestamp, const cv::Mat& mask) {
    check_reset_request();

    assert(camera_->setup_type_ == camera::setup_type_t::Monocular);
    if (img.empty()) {
        AWARN << "preprocess: empty image";
        return nullptr;
    }
    const auto start = std::chrono::system_clock::now();
    auto frm = create_monocular_frame(img, timestamp, mask);
    const auto end = std::chrono::system_clock::now();
    double extraction_time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    return feed_frame(frm, img, extraction_time_elapsed_ms);
}

std::shared_ptr<Mat44_t> system::feed_stereo_frame(const cv::Mat& left_img, const cv::Mat& right_img, const double timestamp, const cv::Mat& mask) {
    check_reset_request();

    assert(camera_->setup_type_ == camera::setup_type_t::Stereo);
    if (left_img.empty() || right_img.empty()) {
        AWARN << "preprocess: empty image";
        return nullptr;
    }
    const auto start = std::chrono::system_clock::now();
    auto frm = create_stereo_frame(left_img, right_img, timestamp, mask);
    const auto end = std::chrono::system_clock::now();
    double extraction_time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    return feed_frame(frm, left_img, extraction_time_elapsed_ms);
}

std::shared_ptr<Mat44_t> system::feed_RGBD_frame(const cv::Mat& rgb_img, const cv::Mat& depthmap, const double timestamp,
                                                 const cv::Mat& mask, const cv::Mat& seg_mask) {
    check_reset_request();

    assert(camera_->setup_type_ == camera::setup_type_t::RGBD);
    if (rgb_img.empty() || depthmap.empty()) {
        AWARN << "preprocess: empty image";
        return nullptr;
    }
    const auto start = std::chrono::system_clock::now();
    auto frm = create_RGBD_frame(rgb_img, depthmap, timestamp, mask, seg_mask);
    const auto end = std::chrono::system_clock::now();
    double extraction_time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    return feed_frame(frm, rgb_img, extraction_time_elapsed_ms);
}

std::shared_ptr<Mat44_t> system::feed_frame(const data::frame& frm, const cv::Mat& img, const double extraction_time_elapsed_ms) {
    const auto start = std::chrono::system_clock::now();

    const auto cam_pose_wc = tracker_->feed_frame(frm);

    const auto end = std::chrono::system_clock::now();
    double tracking_time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    std::vector<data::marker2d> mkrs2d;
    for (auto id_mkr : frm.markers_2d_)
        mkrs2d.push_back(id_mkr.second);

    frame_publisher_->update(tracker_->curr_frm_.get_landmarks(),
                             !mapper_->is_paused(),
                             tracker_->tracking_state_,
                             keypts_,
                             mkrs2d,
                             img,
                             tracking_time_elapsed_ms,
                             extraction_time_elapsed_ms,
                             tracker_->curr_frm_.line_obs_.keylines,
                             tracker_->curr_frm_.get_landmarks_line(),
                             tracker_->curr_frm_.img_seg_mask_,
                             loop_BA_is_running(),
                             mapper_->is_paused() && !loop_BA_is_running());
    if (tracker_->tracking_state_ == tracker_state_t::Tracking && cam_pose_wc) {
        map_publisher_->set_current_cam_pose(util::converter::inverse_pose(*cam_pose_wc));
    }

    return cam_pose_wc;
}

bool system::relocalize_by_pose(const Mat44_t& cam_pose_wc) {
    const Mat44_t cam_pose_cw = util::converter::inverse_pose(cam_pose_wc);
    bool status = tracker_->request_relocalize_by_pose(cam_pose_cw);
    if (status) {
        // Even if state will be lost, still update the pose in map_publisher_
        // to clearly show new camera position
        map_publisher_->set_current_cam_pose(cam_pose_cw);
    }
    return status;
}

bool system::relocalize_by_pose_2d(const Mat44_t& cam_pose_wc, const Vec3_t& normal_vector) {
    const Mat44_t cam_pose_cw = util::converter::inverse_pose(cam_pose_wc);
    bool status = tracker_->request_relocalize_by_pose_2d(cam_pose_cw, normal_vector);
    if (status) {
        // Even if state will be lost, still update the pose in map_publisher_
        // to clearly show new camera position
        map_publisher_->set_current_cam_pose(cam_pose_cw);
    }
    return status;
}

void system::pause_tracker() {
    auto future_pause = tracker_->async_pause();
    future_pause.get();
}

bool system::tracker_is_paused() const {
    return tracker_->is_paused();
}

void system::resume_tracker() {
    tracker_->resume();
}

void system::request_reset() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    reset_is_requested_ = true;
}

bool system::reset_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    return reset_is_requested_;
}

void system::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool system::terminate_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

camera::base* system::get_camera() const {
    return camera_;
}

void system::check_reset_request() {
    std::lock_guard<std::mutex> lock(mtx_reset_);
    if (reset_is_requested_) {
        tracker_->reset();
        reset_is_requested_ = false;
    }
}

void system::pause_other_threads() const {
    // pause the mapping module
    if (mapper_ && !mapper_->is_terminated()) {
        auto future_pause = mapper_->async_pause();
        while (future_pause.wait_for(std::chrono::milliseconds(5)) == std::future_status::timeout) {
            if (mapper_->is_terminated()) {
                break;
            }
        }
    }
    // pause the global optimization module
    if (global_optimizer_ && !global_optimizer_->is_terminated()) {
        auto future_pause = global_optimizer_->async_pause();
        while (future_pause.wait_for(std::chrono::milliseconds(5)) == std::future_status::timeout) {
            if (global_optimizer_->is_terminated()) {
                break;
            }
        }
    }
}

void system::resume_other_threads() const {
    // resume the global optimization module
    if (global_optimizer_) {
        global_optimizer_->resume();
    }
    // resume the mapping module
    if (mapper_) {
        mapper_->resume();
    }
}

}  // namespace autonomy::localization::atlas
