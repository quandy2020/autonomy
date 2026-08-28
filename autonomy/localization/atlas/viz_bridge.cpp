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

#include "autonomy/localization/atlas/viz_bridge.hpp"

#include <cmath>
#include <cstring>
#include <set>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "autonomy/common/logging.hpp"
#include "autonomy/localization/atlas/camera/base.hpp"
#include "autonomy/localization/atlas/camera/fisheye.hpp"
#include "autonomy/localization/atlas/camera/perspective.hpp"
#include "autonomy/localization/atlas/data/keyframe.hpp"
#include "autonomy/localization/atlas/data/landmark.hpp"
#include "autonomy/localization/atlas/data/landmark_line.hpp"
#include "autonomy/localization/atlas/data/landmark_plane.hpp"
#include "autonomy/localization/atlas/publish/frame_publisher.hpp"
#include "autonomy/localization/atlas/publish/map_publisher.hpp"
#include "autonomy/transform/buffer_utils.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

/**
 * Atlas/OpenVSLAM world uses OpenCV camera axes for the first keyframe:
 *   X right, Y down, Z forward.
 * Autoviz / ROS map is REP-103 FLU: X forward, Y left, Z up.
 * Publishing OpenCV poses into "map" makes optical +Z look like world +Z (up),
 * so the trajectory climbs vertically — convert before visualization.
 */
Mat33_t OpencvWorldToRosMapR() {
    Mat33_t R;
    // ROS_x =  CV_z,  ROS_y = -CV_x,  ROS_z = -CV_y
    R << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0;
    return R;
}

Mat44_t OpencvPoseToRosMap(const Mat44_t& T_wc_opencv) {
    Mat44_t T_cv_ros = Mat44_t::Identity();
    T_cv_ros.block<3, 3>(0, 0) = OpencvWorldToRosMapR();
    return T_cv_ros * T_wc_opencv;
}

Eigen::Vector3d OpencvPointToRosMap(const Eigen::Vector3d& p_opencv) {
    return OpencvWorldToRosMapR() * p_opencv;
}

Eigen::Vector3d OpencvNormalToRosMap(const Eigen::Vector3d& n_opencv) {
    return OpencvWorldToRosMapR() * n_opencv;
}

/** OpenCV optical → REP-103 camera_link (X forward, Y left, Z up). */
Mat44_t OpticalToCameraLink() {
    Mat44_t T = Mat44_t::Identity();
    T.block<3, 3>(0, 0) = OpencvWorldToRosMapR();
    return T;
}

Mat44_t OpencvCamPoseToRosCameraLink(const Mat44_t& T_wc_opencv) {
    // T_map_optical * T_optical_link^{-1}  wait:
    // T_map_optical transforms optical pts → map
    // p_map = T_map_optical * p_opt = T_map_optical * inv(T_link_opt) * p_link
    // T_link_opt = OpticalToCameraLink()  (p_link = R * p_opt)
    const Mat44_t T_map_optical = OpencvPoseToRosMap(T_wc_opencv);
    return T_map_optical * OpticalToCameraLink().inverse();
}

Mat44_t TransformMsgToMat44(
    const automsgs::msgs::geometry_msgs::Transform& tf) {
    Mat44_t T = Mat44_t::Identity();
    const Eigen::Quaterniond q(tf.rotation().w(), tf.rotation().x(),
                               tf.rotation().y(), tf.rotation().z());
    T.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    T(0, 3) = tf.translation().x();
    T(1, 3) = tf.translation().y();
    T(2, 3) = tf.translation().z();
    return T;
}

void Mat44ToTransformMsg(const Mat44_t& T,
                         automsgs::msgs::geometry_msgs::Transform* tf) {
    if (tf == nullptr) {
        return;
    }
    const Eigen::Quaterniond q(T.block<3, 3>(0, 0));
    tf->mutable_translation()->set_x(T(0, 3));
    tf->mutable_translation()->set_y(T(1, 3));
    tf->mutable_translation()->set_z(T(2, 3));
    tf->mutable_rotation()->set_w(q.w());
    tf->mutable_rotation()->set_x(q.x());
    tf->mutable_rotation()->set_y(q.y());
    tf->mutable_rotation()->set_z(q.z());
}

Eigen::Vector3d TransformPoint(const Mat44_t& T, const Eigen::Vector3d& p) {
    const Eigen::Vector4d h = T * Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
    return h.head<3>();
}

void AddPoint(google::protobuf::RepeatedPtrField<
                  automsgs::msgs::geometry_msgs::Point>* points,
              const Eigen::Vector3d& p) {
    auto* out = points->Add();
    out->set_x(p.x());
    out->set_y(p.y());
    out->set_z(p.z());
}

void AddLine(google::protobuf::RepeatedPtrField<
                 automsgs::msgs::geometry_msgs::Point>* points,
             const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    AddPoint(points, a);
    AddPoint(points, b);
}

bool GetPinholeIntrinsics(camera::base* cam, double* fx, double* fy, double* cx,
                          double* cy, unsigned int* cols, unsigned int* rows) {
    if (cam == nullptr || fx == nullptr || fy == nullptr || cx == nullptr ||
        cy == nullptr || cols == nullptr || rows == nullptr) {
        return false;
    }
    *cols = cam->cols_;
    *rows = cam->rows_;
    if (auto* p = dynamic_cast<camera::perspective*>(cam)) {
        *fx = p->fx_;
        *fy = p->fy_;
        *cx = p->cx_;
        *cy = p->cy_;
        return *fx > 1e-6 && *fy > 1e-6;
    }
    if (auto* p = dynamic_cast<camera::fisheye*>(cam)) {
        *fx = p->fx_;
        *fy = p->fy_;
        *cx = p->cx_;
        *cy = p->cy_;
        return *fx > 1e-6 && *fy > 1e-6;
    }
    return false;
}

}  // namespace

VizBridge::VizBridge(system* slam, Options options)
    : slam_(slam), options_(std::move(options)) {
    if (options_.trajectory_stride < 1) {
        options_.trajectory_stride = 1;
    }
    if (options_.keyframe_frustum_skip < 1) {
        options_.keyframe_frustum_skip = 1;
    }
    if (options_.map_points_skip < 1) {
        options_.map_points_skip = 1;
    }
    if (options_.frustum_depth <= 0.0) {
        options_.frustum_depth = 0.5;
    }
}

VizBridge::~VizBridge() { Stop(); }

bool VizBridge::Start(const std::shared_ptr<autolink::Node>& node) {
    if (!slam_ || !node) {
        AERROR << "Atlas VizBridge: missing system or autolink node.";
        return false;
    }
    if (running_) {
        return true;
    }
    node_ = node;

    if (options_.publish_tracking_image) {
        tracking_image_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::Image>(
                options_.tracking_image_topic);
    }
    if (options_.publish_frame_match_image) {
        frame_match_image_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::Image>(
                options_.frame_match_image_topic);
    }
    if (options_.publish_trajectory) {
        trajectory_writer_ =
            node_->CreateWriter<automsgs::msgs::nav_msgs::Path>(
                options_.trajectory_topic);
    }
    if (options_.publish_camera_pose) {
        camera_pose_writer_ =
            node_->CreateWriter<automsgs::msgs::nav_msgs::Odometry>(
                options_.camera_pose_topic);
    }
    if (options_.publish_current_frustum) {
        current_frustum_writer_ =
            node_->CreateWriter<automsgs::msgs::visualization_msgs::Marker>(
                options_.current_frustum_topic);
    }
    if (options_.publish_keyframe_frustums) {
        keyframe_frustums_writer_ = node_->CreateWriter<
            automsgs::msgs::visualization_msgs::MarkerArray>(
            options_.keyframe_frustums_topic);
    }
    if (options_.publish_map_points) {
        map_points_writer_ =
            node_->CreateWriter<automsgs::msgs::sensor_msgs::PointCloud2>(
                options_.map_points_topic);
    }
    if (options_.publish_map_planes) {
        map_planes_writer_ = node_->CreateWriter<
            automsgs::msgs::visualization_msgs::MarkerArray>(
            options_.map_planes_topic);
    }
    if (options_.publish_map_lines) {
        map_lines_writer_ = node_->CreateWriter<
            automsgs::msgs::visualization_msgs::MarkerArray>(
            options_.map_lines_topic);
    }
    if (options_.publish_loop_edges) {
        loop_edges_writer_ = node_->CreateWriter<
            automsgs::msgs::visualization_msgs::MarkerArray>(
            options_.loop_edges_topic);
    }

    if (options_.publish_map_odom_tf) {
        tf_buffer_ = transform::Buffer::Instance();
        if (tf_buffer_) {
            tf_buffer_->Init();
        }
        tf_writer_ = node_->CreateWriter<automsgs::msgs::tf2_msgs::TFMessage>(
            options_.tf_topic);
        VizBridge* self = this;
        node_->CreateReader<automsgs::msgs::tf2_msgs::TFMessage>(
            options_.tf_topic,
            [self](
                const std::shared_ptr<automsgs::msgs::tf2_msgs::TFMessage>& msg) {
                if (msg) {
                    self->IngestTfMessage(*msg, "atlas_tf");
                }
            });
        node_->CreateReader<automsgs::msgs::tf2_msgs::TFMessage>(
            options_.tf_static_topic,
            [self](
                const std::shared_ptr<automsgs::msgs::tf2_msgs::TFMessage>& msg) {
                if (msg) {
                    self->IngestTfMessage(*msg, "atlas_tf_static");
                }
            });
    }

    running_ = true;
    AINFO << "Atlas VizBridge started: tracking_image="
          << options_.tracking_image_topic
          << " frame_match=" << options_.frame_match_image_topic
          << " trajectory=" << options_.trajectory_topic
          << " frustum=" << options_.current_frustum_topic
          << " map_odom_tf="
          << (options_.publish_map_odom_tf ? "on" : "off");
    return true;
}

void VizBridge::Stop() {
    running_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    tracking_image_writer_.reset();
    frame_match_image_writer_.reset();
    trajectory_writer_.reset();
    camera_pose_writer_.reset();
    current_frustum_writer_.reset();
    keyframe_frustums_writer_.reset();
    map_points_writer_.reset();
    map_planes_writer_.reset();
    map_lines_writer_.reset();
    loop_edges_writer_.reset();
    tf_writer_.reset();
    tf_buffer_ = nullptr;
    node_.reset();
}

VizBridge::TimeMsg VizBridge::ToStamp(double timestamp_sec) const {
    TimeMsg stamp;
    if (!std::isfinite(timestamp_sec) || timestamp_sec < 0.0) {
        stamp.set_sec(0);
        stamp.set_nanosec(0);
        return stamp;
    }
    const auto sec = static_cast<int32_t>(timestamp_sec);
    auto nanosec = static_cast<uint32_t>(
        std::llround((timestamp_sec - static_cast<double>(sec)) * 1e9));
    if (nanosec >= 1000000000u) {
        stamp.set_sec(sec + 1);
        stamp.set_nanosec(nanosec - 1000000000u);
    } else {
        stamp.set_sec(sec);
        stamp.set_nanosec(nanosec);
    }
    return stamp;
}

void VizBridge::SetHeader(automsgs::msgs::std_msgs::Header* header,
                          double timestamp_sec,
                          const std::string& frame_id) const {
    if (header == nullptr) {
        return;
    }
    *header->mutable_stamp() = ToStamp(timestamp_sec);
    header->set_frame_id(frame_id);
}

void VizBridge::Mat44ToPose(const Mat44_t& T_wc,
                            automsgs::msgs::geometry_msgs::Pose* pose) const {
    if (pose == nullptr) {
        return;
    }
    // T_wc is OpenCV camera→world; convert into ROS map (Z-up) for Autoviz.
    const Mat44_t T_map = OpencvPoseToRosMap(T_wc);
    const Eigen::Matrix3d R = T_map.block<3, 3>(0, 0);
    const Eigen::Quaterniond q(R);
    pose->mutable_position()->set_x(T_map(0, 3));
    pose->mutable_position()->set_y(T_map(1, 3));
    pose->mutable_position()->set_z(T_map(2, 3));
    pose->mutable_orientation()->set_w(q.w());
    pose->mutable_orientation()->set_x(q.x());
    pose->mutable_orientation()->set_y(q.y());
    pose->mutable_orientation()->set_z(q.z());
}

bool VizBridge::CvMatToImageMsg(
    const cv::Mat& bgr, double timestamp_sec, const std::string& frame_id,
    automsgs::msgs::sensor_msgs::Image* msg) const {
    if (msg == nullptr || bgr.empty()) {
        return false;
    }
    cv::Mat continuous = bgr.isContinuous() ? bgr : bgr.clone();
    if (continuous.channels() == 1) {
        cv::cvtColor(continuous, continuous, cv::COLOR_GRAY2BGR);
    }
    if (continuous.type() != CV_8UC3) {
        return false;
    }
    SetHeader(msg->mutable_header(), timestamp_sec, frame_id);
    msg->set_height(static_cast<uint32_t>(continuous.rows));
    msg->set_width(static_cast<uint32_t>(continuous.cols));
    msg->set_encoding("bgr8");
    msg->set_is_bigendian(false);
    msg->set_step(static_cast<uint32_t>(continuous.step));
    msg->set_data(reinterpret_cast<const char*>(continuous.data),
                  static_cast<size_t>(continuous.rows) * continuous.step);
    return true;
}

void VizBridge::AppendFrustumEdges(
    const Mat44_t& T_wc,
    google::protobuf::RepeatedPtrField<automsgs::msgs::geometry_msgs::Point>*
        points) const {
    if (points == nullptr) {
        return;
    }
    const double z = options_.frustum_depth;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    unsigned int cols = 0;
    unsigned int rows = 0;
    Eigen::Vector3d c0;
    Eigen::Vector3d c1;
    Eigen::Vector3d c2;
    Eigen::Vector3d c3;
    if (GetPinholeIntrinsics(slam_->get_camera(), &fx, &fy, &cx, &cy, &cols,
                             &rows)) {
        // Image-plane corners at depth z (camera optical frame, +Z forward).
        const auto corner = [&](double u, double v) {
            return Eigen::Vector3d((u - cx) / fx * z, (v - cy) / fy * z, z);
        };
        c0 = corner(0.0, 0.0);
        c1 = corner(static_cast<double>(cols), 0.0);
        c2 = corner(static_cast<double>(cols), static_cast<double>(rows));
        c3 = corner(0.0, static_cast<double>(rows));
    } else {
        const double half_w = z * 0.5 * 2.0;
        const double half_h = z * 0.5;
        c0 = Eigen::Vector3d(-half_w, -half_h, z);
        c1 = Eigen::Vector3d(half_w, -half_h, z);
        c2 = Eigen::Vector3d(half_w, half_h, z);
        c3 = Eigen::Vector3d(-half_w, half_h, z);
    }

    // Corners are in OpenCV optical frame; T_map puts them into ROS map.
    const Mat44_t T_map = OpencvPoseToRosMap(T_wc);
    const Eigen::Vector3d apex = TransformPoint(T_map, Eigen::Vector3d::Zero());
    const Eigen::Vector3d w0 = TransformPoint(T_map, c0);
    const Eigen::Vector3d w1 = TransformPoint(T_map, c1);
    const Eigen::Vector3d w2 = TransformPoint(T_map, c2);
    const Eigen::Vector3d w3 = TransformPoint(T_map, c3);
    AddLine(points, apex, w0);
    AddLine(points, apex, w1);
    AddLine(points, apex, w2);
    AddLine(points, apex, w3);
    AddLine(points, w0, w1);
    AddLine(points, w1, w2);
    AddLine(points, w2, w3);
    AddLine(points, w3, w0);
}

void VizBridge::PublishTrackingImage(double timestamp_sec) {
    if (!tracking_image_writer_) {
        return;
    }
    const auto fp = slam_->get_frame_publisher();
    if (!fp) {
        return;
    }
    cv::Mat img = fp->draw_frame();
    const cv::Mat seg_panel = fp->draw_seg_mask();
    if (!seg_panel.empty()) {
        cv::Mat stacked;
        cv::vconcat(img, seg_panel, stacked);
        img = stacked;
    }
    automsgs::msgs::sensor_msgs::Image msg;
    if (!CvMatToImageMsg(img, timestamp_sec, options_.camera_frame, &msg)) {
        return;
    }
    tracking_image_writer_->Write(msg);
}

void VizBridge::PublishFrameMatchImage(double timestamp_sec) {
    if (!frame_match_image_writer_) {
        return;
    }
    const auto fp = slam_->get_frame_publisher();
    if (!fp) {
        return;
    }
    cv::Mat img = fp->draw_frame_matches();
    automsgs::msgs::sensor_msgs::Image msg;
    if (!CvMatToImageMsg(img, timestamp_sec, options_.camera_frame, &msg)) {
        return;
    }
    frame_match_image_writer_->Write(msg);
}

void VizBridge::IngestTfMessage(
    const automsgs::msgs::tf2_msgs::TFMessage& message,
    const std::string& authority) {
    if (!tf_buffer_) {
        return;
    }
    // Skip parent=map: Atlas owns map→odom (same as Cartographer).
    automsgs::msgs::tf2_msgs::TFMessage filtered;
    for (const auto& transform : message.transforms()) {
        if (transform.header().frame_id() == options_.map_frame) {
            continue;
        }
        *filtered.add_transforms() = transform;
    }
    const bool is_static = (authority.find("static") != std::string::npos);
    transform::ApplyTfMessageToBuffer(tf_buffer_, filtered, authority,
                                      is_static);
}

void VizBridge::PublishMapOdomTf(double timestamp_sec, const Mat44_t& T_wc) {
    if (!tf_writer_ || !tf_buffer_) {
        return;
    }

    // SLAM tracks OpenCV optical; autosim camera_link is REP-103 FLU.
    const Mat44_t T_map_cam = OpencvCamPoseToRosCameraLink(T_wc);

    automsgs::msgs::builtin_interfaces::Time stamp = ToStamp(timestamp_sec);
    // Prefer latest odom→camera if exact stamp is not yet in the buffer.
    automsgs::msgs::builtin_interfaces::Time lookup_stamp;
    lookup_stamp.set_sec(0);
    lookup_stamp.set_nanosec(0);

    std::string err;
    if (!tf_buffer_->canTransform(options_.odom_frame, options_.camera_frame,
                                  lookup_stamp, 0.05f, &err)) {
        ++map_odom_warn_count_;
        if (map_odom_warn_count_ == 1 || map_odom_warn_count_ % 50 == 0) {
            AWARN << "Atlas VizBridge: waiting for TF " << options_.odom_frame
                  << "→" << options_.camera_frame << " (" << err << ")";
        }
        return;
    }

    Mat44_t T_odom_cam = Mat44_t::Identity();
    try {
        const auto stamped = tf_buffer_->lookupTransform(
            options_.odom_frame, options_.camera_frame, lookup_stamp, 0.05f);
        T_odom_cam = TransformMsgToMat44(stamped.transform());
    } catch (const std::exception& ex) {
        ++map_odom_warn_count_;
        if (map_odom_warn_count_ == 1 || map_odom_warn_count_ % 50 == 0) {
            AWARN << "Atlas VizBridge: lookup " << options_.odom_frame << "→"
                  << options_.camera_frame << " failed: " << ex.what();
        }
        return;
    }

    const Mat44_t T_map_odom = T_map_cam * T_odom_cam.inverse();

    automsgs::msgs::geometry_msgs::TransformStamped map_odom;
    *map_odom.mutable_header()->mutable_stamp() = stamp;
    map_odom.mutable_header()->set_frame_id(options_.map_frame);
    map_odom.set_child_frame_id(options_.odom_frame);
    Mat44ToTransformMsg(T_map_odom, map_odom.mutable_transform());

    transform::ApplyTransformStampedToBuffer(tf_buffer_, map_odom, "atlas",
                                             false);

    automsgs::msgs::tf2_msgs::TFMessage tf_msg;
    *tf_msg.add_transforms() = map_odom;
    tf_writer_->Write(tf_msg);

    if (map_odom_warn_count_ > 0 || map_odom_pub_count_ == 0) {
        AINFO << "Atlas VizBridge: publishing " << options_.map_frame << "→"
              << options_.odom_frame;
    }
    ++map_odom_pub_count_;
}

void VizBridge::PublishPoseAndTrajectory(double timestamp_sec,
                                         const Mat44_t& T_wc) {
    automsgs::msgs::geometry_msgs::Pose pose;
    Mat44ToPose(T_wc, &pose);

    if (camera_pose_writer_) {
        automsgs::msgs::nav_msgs::Odometry odom;
        SetHeader(odom.mutable_header(), timestamp_sec, options_.map_frame);
        odom.set_child_frame_id(options_.camera_frame);
        // PoseWithCovariance.pose is PoseStamped in this automsgs schema.
        auto* pose_stamped = odom.mutable_pose()->mutable_pose();
        SetHeader(pose_stamped->mutable_header(), timestamp_sec,
                  options_.map_frame);
        *pose_stamped->mutable_pose() = pose;
        camera_pose_writer_->Write(odom);
    }

    if (!trajectory_writer_) {
        return;
    }
    ++trajectory_frame_counter_;
    if (trajectory_frame_counter_ % options_.trajectory_stride != 0) {
        return;
    }
    auto* stamped = trajectory_path_.add_poses();
    SetHeader(stamped->mutable_header(), timestamp_sec, options_.map_frame);
    *stamped->mutable_pose() = pose;
    SetHeader(trajectory_path_.mutable_header(), timestamp_sec,
              options_.map_frame);
    trajectory_writer_->Write(trajectory_path_);
}

void VizBridge::PublishCurrentFrustum(double timestamp_sec,
                                      const Mat44_t& T_wc) {
    if (!current_frustum_writer_) {
        return;
    }
    using Marker = automsgs::msgs::visualization_msgs::Marker;
    Marker marker;
    SetHeader(marker.mutable_header(), timestamp_sec, options_.map_frame);
    marker.set_ns("current_camera");
    marker.set_id(0);
    marker.set_type(Marker::LINE_LIST);
    marker.set_action(Marker::ADD);
    marker.mutable_pose()->mutable_orientation()->set_w(1.0);
    marker.mutable_scale()->set_x(0.008);
    marker.mutable_color()->set_r(0.35f);
    marker.mutable_color()->set_g(0.95f);
    marker.mutable_color()->set_b(1.0f);
    marker.mutable_color()->set_a(1.0f);
    AppendFrustumEdges(T_wc, marker.mutable_points());
    if (marker.points_size() > 0) {
        current_frustum_writer_->Write(marker);
    }
}

void VizBridge::PublishKeyframeFrustums(double timestamp_sec) {
    if (!keyframe_frustums_writer_) {
        return;
    }
    using Marker = automsgs::msgs::visualization_msgs::Marker;
    using MarkerArray = automsgs::msgs::visualization_msgs::MarkerArray;

    std::vector<std::shared_ptr<data::keyframe>> keyframes;
    const auto mp = slam_->get_map_publisher();
    if (!mp) {
        return;
    }
    const unsigned int count = mp->get_keyframes(keyframes);
    MarkerArray array;
    {
        auto* clear = array.add_markers();
        clear->set_action(Marker::DELETEALL);
    }
    if (count == 0) {
        keyframe_frustums_writer_->Write(array);
        last_keyframe_count_ = 0;
        return;
    }

    int marker_id = 0;
    for (const auto& keyfrm : keyframes) {
        if (!keyfrm || keyfrm->will_be_erased()) {
            continue;
        }
        if (options_.keyframe_frustum_skip > 1 &&
            (keyfrm->id_ %
             static_cast<unsigned int>(options_.keyframe_frustum_skip)) != 0) {
            continue;
        }
        auto* marker = array.add_markers();
        SetHeader(marker->mutable_header(), timestamp_sec, options_.map_frame);
        marker->set_ns("keyframe_frustums");
        marker->set_id(marker_id++);
        marker->set_type(Marker::LINE_LIST);
        marker->set_action(Marker::ADD);
        marker->mutable_pose()->mutable_orientation()->set_w(1.0);
        marker->mutable_scale()->set_x(0.006);
        marker->mutable_color()->set_r(0.2f);
        marker->mutable_color()->set_g(0.9f);
        marker->mutable_color()->set_b(0.3f);
        marker->mutable_color()->set_a(0.85f);
        AppendFrustumEdges(keyfrm->get_pose_wc(), marker->mutable_points());
    }
    keyframe_frustums_writer_->Write(array);
    last_keyframe_count_ = count;
}

void VizBridge::PublishMapPoints(double timestamp_sec) {
    if (!map_points_writer_) {
        return;
    }
    const auto mp = slam_->get_map_publisher();
    if (!mp) {
        return;
    }
    std::vector<std::shared_ptr<data::landmark>> landmarks;
    std::set<std::shared_ptr<data::landmark>> local_landmarks;
    mp->get_landmarks(landmarks, local_landmarks);

    std::vector<float> xyz;
    std::vector<uint32_t> rgb_packed;
    xyz.reserve(landmarks.size() * 3);
    rgb_packed.reserve(landmarks.size());
    constexpr uint32_t kWhiteRgb = 0x00FFFFFFu;
    for (std::size_t i = 0; i < landmarks.size();
         i += static_cast<std::size_t>(options_.map_points_skip)) {
        const auto& lm = landmarks[i];
        if (!lm || lm->will_be_erased()) {
            continue;
        }
        const Eigen::Vector3d pos =
            OpencvPointToRosMap(lm->get_pos_in_world());
        xyz.push_back(static_cast<float>(pos(0)));
        xyz.push_back(static_cast<float>(pos(1)));
        xyz.push_back(static_cast<float>(pos(2)));
        rgb_packed.push_back(kWhiteRgb);
    }

    automsgs::msgs::sensor_msgs::PointCloud2 cloud;
    SetHeader(cloud.mutable_header(), timestamp_sec, options_.map_frame);
    cloud.set_height(1);
    cloud.set_width(static_cast<uint32_t>(xyz.size() / 3));
    cloud.set_is_dense(true);
    cloud.set_is_bigendian(false);
    cloud.set_point_step(16);
    cloud.set_row_step(cloud.point_step() * cloud.width());

    const char* names[] = {"x", "y", "z", "rgb"};
    for (int i = 0; i < 4; ++i) {
        auto* field = cloud.add_fields();
        field->set_name(names[i]);
        field->set_offset(static_cast<uint32_t>(i * 4));
        field->set_datatype(automsgs::msgs::sensor_msgs::PointField::FLOAT32);
        field->set_count(1);
    }
    if (!xyz.empty()) {
        std::vector<float> data;
        data.reserve(xyz.size() + rgb_packed.size());
        for (std::size_t i = 0; i < rgb_packed.size(); ++i) {
            data.push_back(xyz[i * 3 + 0]);
            data.push_back(xyz[i * 3 + 1]);
            data.push_back(xyz[i * 3 + 2]);
            float rgb_as_float = 0.f;
            const uint32_t rgb = rgb_packed[i];
            std::memcpy(&rgb_as_float, &rgb, sizeof(float));
            data.push_back(rgb_as_float);
        }
        cloud.set_data(reinterpret_cast<const char*>(data.data()),
                       data.size() * sizeof(float));
    }
    map_points_writer_->Write(cloud);
}

void VizBridge::PublishMapPlanes(double timestamp_sec) {
    if (!map_planes_writer_) {
        return;
    }
    const auto mp = slam_->get_map_publisher();
    if (!mp) {
        return;
    }

    std::vector<std::shared_ptr<data::landmark_plane>> planes;
    mp->get_landmark_planes(planes);

    using Marker = automsgs::msgs::visualization_msgs::Marker;
    using MarkerArray = automsgs::msgs::visualization_msgs::MarkerArray;
    MarkerArray array;

    int marker_id = 0;
    constexpr double kSquareSize = 0.10;
    constexpr float kPlaneAlpha = 0.7f;

    for (const auto& plane : planes) {
        if (!plane || !plane->is_valid()) {
            continue;
        }
        const auto map_pts = plane->get_landmarks();
        if (map_pts.empty()) {
            continue;
        }

        const double err_thr = plane->get_best_error();
        const Vec3_t n_cv = plane->get_normal().normalized();
        Vec3_t base1_cv = Vec3_t::Zero();
        Vec3_t base2_cv = Vec3_t::Zero();
        bool found_basis = false;
        for (size_t i = 0; i < map_pts.size() && !found_basis; ++i) {
            if (!map_pts[i] || map_pts[i]->will_be_erased() ||
                !map_pts[i]->get_owning_plane() ||
                plane->calculate_distance(map_pts[i]->get_pos_in_world()) > err_thr) {
                continue;
            }
            for (size_t j = i + 1; j < map_pts.size(); ++j) {
                if (!map_pts[j] || map_pts[j]->will_be_erased() ||
                    !map_pts[j]->get_owning_plane() ||
                    plane->calculate_distance(map_pts[j]->get_pos_in_world()) > err_thr) {
                    continue;
                }
                const Vec3_t diff =
                    map_pts[i]->get_pos_in_world() - map_pts[j]->get_pos_in_world();
                if (diff.norm() < 1e-3) {
                    continue;
                }
                base1_cv = diff.normalized();
                base2_cv = base1_cv.cross(n_cv).normalized();
                if (base2_cv.norm() > 1e-6) {
                    found_basis = true;
                    break;
                }
            }
        }
        if (!found_basis) {
            continue;
        }

        auto* marker = array.add_markers();
        SetHeader(marker->mutable_header(), timestamp_sec, options_.map_frame);
        marker->set_ns("map_planes");
        marker->set_id(marker_id++);
        marker->set_type(Marker::TRIANGLE_LIST);
        marker->set_action(Marker::ADD);
        marker->mutable_pose()->mutable_orientation()->set_w(1.0);

        float pr = 0.5f;
        float pg = 0.5f;
        float pb = 0.5f;
        plane->get_display_color(pr, pg, pb);
        marker->mutable_color()->set_r(pr);
        marker->mutable_color()->set_g(pg);
        marker->mutable_color()->set_b(pb);
        marker->mutable_color()->set_a(kPlaneAlpha);

        auto add_point = [&](const Vec3_t& p_ros) {
            auto* pt = marker->add_points();
            pt->set_x(p_ros(0));
            pt->set_y(p_ros(1));
            pt->set_z(p_ros(2));
        };
        for (const auto& point : map_pts) {
            if (!point || point->will_be_erased() || !point->get_owning_plane()) {
                continue;
            }
            if (plane->calculate_distance(point->get_pos_in_world()) > err_thr) {
                continue;
            }
            const Vec3_t center = point->get_pos_in_world();
            const Vec3_t tr = center + kSquareSize * (base1_cv + base2_cv);
            const Vec3_t br = center + kSquareSize * (base1_cv - base2_cv);
            const Vec3_t bl = center - kSquareSize * (base1_cv + base2_cv);
            const Vec3_t tl = center + kSquareSize * (base2_cv - base1_cv);
            add_point(OpencvPointToRosMap(tr));
            add_point(OpencvPointToRosMap(br));
            add_point(OpencvPointToRosMap(bl));
            add_point(OpencvPointToRosMap(tr));
            add_point(OpencvPointToRosMap(bl));
            add_point(OpencvPointToRosMap(tl));
        }
    }

    map_planes_writer_->Write(array);
    last_plane_count_ = static_cast<unsigned int>(planes.size());
}

void VizBridge::PublishMapLines(double timestamp_sec) {
    if (!map_lines_writer_) {
        return;
    }
    const auto mp = slam_->get_map_publisher();
    if (!mp) {
        return;
    }

    std::vector<std::shared_ptr<data::landmark_line>> lines;
    mp->get_landmark_lines(lines);

    using Marker = automsgs::msgs::visualization_msgs::Marker;
    using MarkerArray = automsgs::msgs::visualization_msgs::MarkerArray;
    MarkerArray array;

    auto* marker = array.add_markers();
    SetHeader(marker->mutable_header(), timestamp_sec, options_.map_frame);
    marker->set_ns("map_lines");
    marker->set_id(0);
    marker->set_type(Marker::LINE_LIST);
    marker->set_action(Marker::ADD);
    marker->mutable_scale()->set_x(0.02);
    marker->mutable_color()->set_r(0.4f);
    marker->mutable_color()->set_g(0.35f);
    marker->mutable_color()->set_b(0.8f);
    marker->mutable_color()->set_a(0.95f);

    for (const auto& lm_line : lines) {
        if (!lm_line || lm_line->will_be_erased()) {
            continue;
        }
        const Vec6_t pos_w = lm_line->get_pos_in_world();
        const Vec3_t sp = OpencvPointToRosMap(pos_w.head<3>());
        const Vec3_t ep = OpencvPointToRosMap(pos_w.tail<3>());

        auto* pt_sp = marker->add_points();
        pt_sp->set_x(sp(0));
        pt_sp->set_y(sp(1));
        pt_sp->set_z(sp(2));
        auto* pt_ep = marker->add_points();
        pt_ep->set_x(ep(0));
        pt_ep->set_y(ep(1));
        pt_ep->set_z(ep(2));
    }

    map_lines_writer_->Write(array);
    last_line_count_ = static_cast<unsigned int>(lines.size());
}

void VizBridge::PublishLoopEdges(double timestamp_sec) {
    if (!loop_edges_writer_) {
        return;
    }
    using Marker = automsgs::msgs::visualization_msgs::Marker;
    using MarkerArray = automsgs::msgs::visualization_msgs::MarkerArray;

    std::vector<std::shared_ptr<data::keyframe>> keyframes;
    const auto mp = slam_->get_map_publisher();
    if (!mp) {
        return;
    }
    mp->get_keyframes(keyframes);

    MarkerArray array;
    {
        auto* clear = array.add_markers();
        clear->set_action(Marker::DELETEALL);
    }

    Marker* marker = array.add_markers();
    SetHeader(marker->mutable_header(), timestamp_sec, options_.map_frame);
    marker->set_ns("loop_edges");
    marker->set_id(0);
    marker->set_type(Marker::LINE_LIST);
    marker->set_action(Marker::ADD);
    marker->mutable_pose()->mutable_orientation()->set_w(1.0);
    marker->mutable_scale()->set_x(0.025);
    marker->mutable_color()->set_r(0.1f);
    marker->mutable_color()->set_g(1.0f);
    marker->mutable_color()->set_b(0.2f);
    marker->mutable_color()->set_a(0.95f);

    std::set<std::pair<unsigned int, unsigned int>> drawn;
    for (const auto& keyfrm : keyframes) {
        if (!keyfrm || keyfrm->will_be_erased()) {
            continue;
        }
        const Vec3_t p0 = OpencvPointToRosMap(keyfrm->get_trans_wc());
        for (const auto& connected : keyfrm->graph_node_->get_loop_edges()) {
            if (!connected || connected->will_be_erased()) {
                continue;
            }
            unsigned int id0 = keyfrm->id_;
            unsigned int id1 = connected->id_;
            if (id0 > id1) {
                std::swap(id0, id1);
            }
            if (!drawn.insert({id0, id1}).second) {
                continue;
            }
            const Vec3_t p1 = OpencvPointToRosMap(connected->get_trans_wc());
            AddLine(marker->mutable_points(), p0, p1);
        }
    }

    loop_edges_writer_->Write(array);
    last_loop_edge_count_ = static_cast<unsigned int>(drawn.size());
}

void VizBridge::PublishFrame(double timestamp_sec,
                             const std::shared_ptr<Mat44_t>& cam_pose_wc) {
    if (!running_ || !slam_) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!running_) {
        return;
    }

    const bool loop_ba_running = slam_->loop_BA_is_running();
    if (was_loop_ba_running_ && !loop_ba_running) {
        last_keyframe_count_ = 0;
        last_plane_count_ = 0;
        last_line_count_ = 0;
        last_loop_edge_count_ = 0;
    }
    was_loop_ba_running_ = loop_ba_running;

    if (options_.publish_tracking_image) {
        PublishTrackingImage(timestamp_sec);
    }
    if (options_.publish_frame_match_image) {
        PublishFrameMatchImage(timestamp_sec);
    }

    if (cam_pose_wc) {
        if (options_.publish_camera_pose || options_.publish_trajectory) {
            PublishPoseAndTrajectory(timestamp_sec, *cam_pose_wc);
        }
        if (options_.publish_map_odom_tf) {
            PublishMapOdomTf(timestamp_sec, *cam_pose_wc);
        }
        if (options_.publish_current_frustum) {
            PublishCurrentFrustum(timestamp_sec, *cam_pose_wc);
        }
    }

    // Do not read the map graph while loop BA holds mtx_database_ (avoids g2o / viz races).
    if (loop_ba_running) {
        return;
    }

    unsigned int kf_count = 0;
    if (const auto mp = slam_->get_map_publisher()) {
        std::vector<std::shared_ptr<data::keyframe>> tmp;
        kf_count = mp->get_keyframes(tmp);
    }
    const bool keyframes_changed = (kf_count != last_keyframe_count_);
    if (options_.publish_keyframe_frustums &&
        (keyframes_changed || last_keyframe_count_ == 0)) {
        PublishKeyframeFrustums(timestamp_sec);
    }
    if (options_.publish_map_points &&
        (keyframes_changed || last_keyframe_count_ == 0)) {
        PublishMapPoints(timestamp_sec);
    }
    if (options_.publish_map_planes) {
        unsigned int plane_count = 0;
        if (const auto mp = slam_->get_map_publisher()) {
            std::vector<std::shared_ptr<data::landmark_plane>> tmp;
            plane_count = mp->get_landmark_planes(tmp);
        }
        if (keyframes_changed || plane_count != last_plane_count_) {
            PublishMapPlanes(timestamp_sec);
        }
    }
    if (options_.publish_map_lines) {
        PublishMapLines(timestamp_sec);
    }
    if (options_.publish_loop_edges) {
        unsigned int loop_edge_count = last_loop_edge_count_;
        if (const auto mp = slam_->get_map_publisher()) {
            std::vector<std::shared_ptr<data::keyframe>> tmp;
            mp->get_keyframes(tmp);
            loop_edge_count = 0;
            std::set<std::pair<unsigned int, unsigned int>> counted;
            for (const auto& keyfrm : tmp) {
                if (!keyfrm) {
                    continue;
                }
                for (const auto& connected : keyfrm->graph_node_->get_loop_edges()) {
                    if (!connected) {
                        continue;
                    }
                    unsigned int id0 = keyfrm->id_;
                    unsigned int id1 = connected->id_;
                    if (id0 > id1) {
                        std::swap(id0, id1);
                    }
                    if (counted.insert({id0, id1}).second) {
                        ++loop_edge_count;
                    }
                }
            }
        }
        if (keyframes_changed || loop_edge_count != last_loop_edge_count_) {
            PublishLoopEdges(timestamp_sec);
        }
    }
    last_keyframe_count_ = kf_count;
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
