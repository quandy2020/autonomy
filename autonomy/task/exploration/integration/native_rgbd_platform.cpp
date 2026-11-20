/* Copyright 2026 The Openbot Authors. */
/** @file native_rgbd_platform.cpp
 * @brief Actual native RGB-D providers and PlannerServer source installed on TaskServer's task.
 */
#include "autonomy/task/exploration/integration/native_rgbd_platform.hpp"

#ifdef AUTONOMY_TASK_HAS_NATIVE_ATLAS_MAPPING
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <google/protobuf/util/json_util.h>
#include <Eigen/Geometry>
#include <autolink/action/client.hpp>
#include <automsgs/actions/nav_actions.pb.h>
#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include "autolink/autolink.hpp"
#include "autolink/croutine/croutine.hpp"
#include "autonomy/common/yaml.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller_server.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/footprint.hpp"
#include "autonomy/perception/observation/native_sensor_messages.hpp"
#include "autonomy/perception/observation/reader_retirement.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/planner/vista/vista_planner.hpp"
#include "autonomy/transform/buffer_utils.hpp"

namespace autonomy::task::exploration {
namespace {
using StartupSpinAction = ::automsgs::actions::SpinAction;  ///< Existing native recovery action reused for observation.
using StartupSpinClient = ::autolink::action::Client<StartupSpinAction>;  ///< Native action transport.
constexpr char kStartupSpinAction[] = "/spin";  ///< Existing ControllerServer action endpoint.
/** @brief Native-source clock explicitly selected by this platform configuration. */
double Now() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }
/** @brief Resolve actual calibration/persistence files relative to the reviewed configuration. */
std::filesystem::path Path(const std::filesystem::path& root,const std::string& name) {
    if (name.empty()) throw std::invalid_argument("native RGB-D configuration contains an empty file path");
    const std::filesystem::path path(name); return path.is_absolute() ? path : root/path;
}
/** @brief Read actual generated native messages, without constructing calibration or GT substitutes. */
template <typename Message> Message MessageFile(const std::filesystem::path& path) {
    std::ifstream input(path,std::ios::binary); Message message;
    if (!input || !message.ParseFromIstream(&input)) throw std::invalid_argument("invalid native calibration file: "+path.string());
    return message;
}
/** @brief Convert existing native TF messages into the existing tf2 calibration tree representation. */
::geometry_msgs::TransformStamped Internal(const ::automsgs::msgs::geometry_msgs::TransformStamped& input) {
    ::geometry_msgs::TransformStamped result;
    result.header.frame_id = input.header().frame_id(); result.child_frame_id = input.child_frame_id();
    result.header.stamp = static_cast<std::uint64_t>(input.header().stamp().sec())*1000000000ULL+input.header().stamp().nanosec();
    const auto& t = input.transform();
    result.transform.translation.x = t.translation().x(); result.transform.translation.y = t.translation().y();
    result.transform.translation.z = t.translation().z();
    result.transform.rotation.x = t.rotation().x(); result.transform.rotation.y = t.rotation().y();
    result.transform.rotation.z = t.rotation().z(); result.transform.rotation.w = t.rotation().w();
    return result;
}
/** @brief Convert a validated native pose/transform into Eigen's existing rigid transform implementation. */
template <typename Position,typename Quaternion>
Eigen::Isometry3d Rigid(const Position& position,const Quaternion& quaternion) {
    Eigen::Quaterniond rotation(quaternion.w(),quaternion.x(),quaternion.y(),quaternion.z());
    if (!rotation.coeffs().allFinite() || std::abs(rotation.norm()-1)>1e-3)
        throw std::invalid_argument("invalid actual native camera/installation rotation");
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity(); result.linear() = rotation.normalized().toRotationMatrix();
    result.translation() = Eigen::Vector3d{position.x(),position.y(),position.z()};
    if (!result.matrix().allFinite()) throw std::invalid_argument("invalid actual native camera/installation translation");
    return result;
}
/** @brief Encode a real composed alignment using the existing native TransformStamped message. */
void Encode(const Eigen::Isometry3d& pose,::automsgs::msgs::geometry_msgs::Transform* output) {
    const Eigen::Quaterniond rotation(pose.linear());
    output->mutable_translation()->set_x(pose.translation().x()); output->mutable_translation()->set_y(pose.translation().y());
    output->mutable_translation()->set_z(pose.translation().z());
    output->mutable_rotation()->set_w(rotation.w()); output->mutable_rotation()->set_x(rotation.x());
    output->mutable_rotation()->set_y(rotation.y()); output->mutable_rotation()->set_z(rotation.z());
}
}  // namespace

NativeRgbdPlatformOptions LoadNativeRgbdPlatformOptions(const std::filesystem::path& path) {
    const auto yaml = YAML::LoadFile(path.string()); const auto root = path.parent_path();
    if (yaml["clock_domain"].as<std::string>() != "system")
        throw std::invalid_argument("native RGB-D platform requires explicitly matching system-epoch sensor stamps");
    NativeRgbdPlatformOptions options;
    const auto camera = MessageFile<::automsgs::msgs::sensor_msgs::CameraInfo>(Path(root,yaml["camera_info_file"].as<std::string>()));
    const auto tf = MessageFile<::automsgs::msgs::tf2_msgs::TFMessage>(Path(root,yaml["tf_static_file"].as<std::string>()));
    auto& config = options.exploration;
    config.input.mode = config.session.mode = config.coverage.mode = perception::SensorMode::kRgbdOnly;
    config.input.depth_topic = yaml["depth_topic"].as<std::string>();
    config.input.camera_info_topic = yaml["camera_info_topic"].as<std::string>();
    config.input.depth_calibration_revision = yaml["calibration_revision"].as<std::uint64_t>();
    config.input.depth_rectified = true;
    config.input.minimum_depth = yaml["minimum_depth"].as<double>(); config.input.maximum_depth = yaml["maximum_depth"].as<double>();
    config.input.decoding.valid_for = .75; config.input.localization_wait_timeout = std::chrono::milliseconds(250);
    config.input.decoding.rgbd_pixel_stride = yaml["ray_pixel_stride"] ? yaml["ray_pixel_stride"].as<unsigned>() : 4;
    const auto calibration = perception::NativeDepthCalibration(camera,config.input.depth_calibration_revision,true,
        config.input.minimum_depth,config.input.maximum_depth);
    if (!calibration) throw std::invalid_argument("actual CameraInfo does not support the declared rectified RGB-D calibration");
    options.localization.backend_frame = yaml["atlas_frame"].as<std::string>();
    options.localization.navigation_frame = config.routes.frame_id = options.mapping.frame_id = yaml["navigation_frame"].as<std::string>();
    options.localization.camera_frame = camera.header().frame_id(); options.localization.base_frame = yaml["physical_base_frame"].as<std::string>();
    options.navigation_base_frame = yaml["navigation_base_frame"].as<std::string>();
    options.localization.odometry_topic = yaml["atlas_odometry_topic"].as<std::string>(); options.status_topic = yaml["atlas_status_topic"].as<std::string>();
    options.localization.calibration_revision = config.input.depth_calibration_revision; options.localization.evidence_lifetime = .75;
    ::autonomy::transform::tf2::BufferCore mounts;
    for (const auto& transform : tf.transforms()) {
        if (transform.header().frame_id() == "map" || transform.header().frame_id() == "odom" ||
            transform.header().frame_id() == options.localization.backend_frame ||
            transform.header().frame_id() == options.localization.navigation_frame) continue;
        mounts.setTransform(Internal(transform),"actual_rgbd_installation",true);
    }
    const auto mount = mounts.lookupTransform(options.localization.base_frame,options.localization.camera_frame,0);
    auto& installation = options.localization.base_from_camera;
    installation.mutable_header()->set_frame_id(options.localization.base_frame); installation.set_child_frame_id(options.localization.camera_frame);
    installation.mutable_transform()->mutable_translation()->set_x(mount.transform.translation.x);
    installation.mutable_transform()->mutable_translation()->set_y(mount.transform.translation.y);
    installation.mutable_transform()->mutable_translation()->set_z(mount.transform.translation.z);
    installation.mutable_transform()->mutable_rotation()->set_x(mount.transform.rotation.x);
    installation.mutable_transform()->mutable_rotation()->set_y(mount.transform.rotation.y);
    installation.mutable_transform()->mutable_rotation()->set_z(mount.transform.rotation.z);
    installation.mutable_transform()->mutable_rotation()->set_w(mount.transform.rotation.w);
    const auto actual_mount = Rigid(installation.transform().translation(),installation.transform().rotation());
    perception::RigidTransform base_from_camera;
    for (unsigned row = 0; row < 3; ++row) for (unsigned col = 0; col < 3; ++col)
        base_from_camera.rotation[3*row+col] = actual_mount.linear()(row,col);
    base_from_camera.translation = {actual_mount.translation().x(),actual_mount.translation().y(),actual_mount.translation().z()};
    config.camera = NativeCameraCoverage{calibration->camera,base_from_camera};
    config.routes.service_name = yaml["route_service"].as<std::string>(); config.routes.planner_id = yaml["planner_id"].as<std::string>();
    config.routes.navigation_tree = yaml["navigation_tree"] ?
        yaml["navigation_tree"].as<std::string>() : std::string{};
    config.controller_id = yaml["controller_id"].as<std::string>();
    if (config.controller_id.empty()) throw std::invalid_argument("native RGB-D execution requires an explicit existing controller plugin");
    const auto validated_mask = yaml["validated_sensor_mask"].as<unsigned>();
    if (validated_mask != perception::Mask(perception::SensorMode::kRgbdOnly))
        throw std::invalid_argument("native RGB-D platform requires an independently validated RGB-D execution mask");
    config.session.validated_controller_modes = {static_cast<perception::SensorMask>(validated_mask)};
    config.session.time_budget = yaml["time_budget"].as<double>(); config.session.require_return = yaml["require_return"].as<bool>();
    config.session.require_save = yaml["require_save"].as<bool>(); config.session.map_name = yaml["map_name"].as<std::string>();
    if (const auto startup = yaml["startup_observation"]; startup) {
        auto& policy = config.session.startup_observation;
        policy.enabled = startup["enabled"] ? startup["enabled"].as<bool>() : false;
        if (startup["target_yaw"]) policy.target_yaw = startup["target_yaw"].as<double>();
        if (startup["time_allowance"]) policy.time_allowance = startup["time_allowance"].as<double>();
        if (startup["deadline_margin"]) policy.deadline_margin = startup["deadline_margin"].as<double>();
        if (startup["maximum_position_drift"]) policy.maximum_position_drift =
            startup["maximum_position_drift"].as<double>();
        if (startup["completion_tolerance"]) policy.completion_tolerance =
            startup["completion_tolerance"].as<double>();
        if (startup["maximum_pose_age"]) policy.maximum_pose_age =
            startup["maximum_pose_age"].as<double>();
        if (startup["support_radius"]) policy.support_radius =
            startup["support_radius"].as<double>();
        if (startup["support_ring_radius"]) policy.support_ring_radius =
            startup["support_ring_radius"].as<double>();
        if (startup["support_timeout"]) policy.support_timeout =
            startup["support_timeout"].as<double>();
        if (startup["minimum_support_cells"]) policy.minimum_support_cells =
            startup["minimum_support_cells"].as<unsigned>();
    }
    auto& bounds = options.mapping.region;
    const auto region = yaml["region"];
    bounds = {region["min_x"].as<double>(),region["min_y"].as<double>(),region["max_x"].as<double>(),region["max_y"].as<double>()};
    config.session.allowed_region = bounds;
    options.mapping.resolution = yaml["mapping_resolution"].as<double>();
    options.mapping.sensor_heights[1] = actual_mount.translation().z();
    options.mapping.robot_height = config.coverage.safety.robot_height = yaml["robot_height"].as<double>();
    options.mapping.vertical_margin = config.coverage.safety.vertical_margin = yaml["vertical_margin"].as<double>();
    config.coverage.safety.robot_radius = yaml["robot_radius"].as<double>();
    config.coverage.safety.horizontal_margin = yaml["horizontal_margin"].as<double>();
    options.mapping.output_directory = Path(root,yaml["map_output_directory"].as<std::string>());
    options.odometry_output_topic = yaml["base_odometry_topic"].as<std::string>();
    options.occupancy_output_topic = yaml["occupancy_topic"].as<std::string>(); options.grid_output_topic = yaml["grid_topic"].as<std::string>();
    options.host_vista_server = yaml["host_vista_server"].as<bool>();
    options.measured_coverage_topic = yaml["measured_coverage_topic"] ? yaml["measured_coverage_topic"].as<std::string>() :
        options.grid_output_topic+"/measured_coverage";
    if (yaml["controller_options_file"]) {
        const auto file = Path(root,yaml["controller_options_file"].as<std::string>());
        std::ifstream input(file); std::string text{std::istreambuf_iterator<char>(input),{}};
        control::proto::ControllerOptions controller;
        const auto parsed = google::protobuf::util::JsonStringToMessage(text,&controller);
        if (!input || !parsed.ok() || controller.controller_plugins().empty())
            throw std::invalid_argument("actual native controller options JSON failed to load: "+file.string());
        bool requested = false;
        for (const auto& plugin : controller.controller_plugins())
            if (plugin.substr(0,plugin.find(':')) == config.controller_id) requested = true;
        if (!requested) throw std::invalid_argument("requested local controller is absent from actual native options");
        options.command_topic = yaml["command_topic"].as<std::string>();
        options.controller_image_topic = yaml["controller_image_topic"].as<std::string>();
        options.controller_camera_frame = yaml["controller_camera_frame"].as<std::string>();
        options.command_gate.body = config.coverage.safety;
        options.command_gate.maximum_linear_speed = yaml["maximum_linear_speed"].as<double>();
        options.command_gate.maximum_yaw_rate = yaml["maximum_yaw_rate"].as<double>();
        options.command_gate.actuator_watchdog = yaml["actuator_watchdog"].as<double>();
        options.command_gate.reaction_time = yaml["reaction_time"].as<double>();
        options.command_gate.minimum_braking_deceleration = yaml["minimum_braking_deceleration"].as<double>();
        if (yaml["maximum_feedback_age"])
            options.command_gate.maximum_feedback_age = yaml["maximum_feedback_age"].as<double>();
        if (yaml["maximum_map_age"])
            options.command_gate.maximum_map_age = yaml["maximum_map_age"].as<double>();
        controller.mutable_policy_controller_options()->set_image_topic(options.controller_image_topic);
        controller.mutable_policy_controller_options()->set_robot_frame(options.navigation_base_frame);
        // The native relay preserves the calibrated optical frame instead of
        // relabeling an image as body frame. PolicyController validates exact
        // sensor frame names and cannot TF-transform image data in C++; its
        // path-frame alignment remains active independently.
        controller.mutable_policy_controller_options()->set_require_frame_match(false);
        options.controller = std::move(controller);
    }
    return options;
}

class NativeRgbdPlatform::Impl : public std::enable_shared_from_this<NativeRgbdPlatform::Impl> {
public:
    /** @brief One explicit yaw-only motion admission; no map cell is changed. */
    struct StartupAdmission {
        std::mutex mutex;  ///< Protects the admission used by controller callback threads.
        bool active = false;  ///< True only between native request and validated terminal.
        RequestVersion version;  ///< Session identity that authorized the sweep.
        double start_x = 0;  ///< Measured startup X in the navigation frame.
        double start_y = 0;  ///< Measured startup Y in the navigation frame.
        double start_yaw = 0;  ///< Measured yaw at request time.
        double deadline = 0;  ///< Absolute owner-clock deadline, seconds.
        double target_yaw = 0;  ///< Requested nonzero sweep angle, radians.
        double maximum_drift = 0;  ///< Deployment-confirmed position envelope, metres.
        double maximum_yaw_rate = 0;  ///< Existing controller yaw-command bound, rad/s.
        bool has_measured_yaw = false;  ///< A measured yaw baseline is available.
        double last_measured_yaw = 0;  ///< Previous measured yaw for sweep accumulation, radians.
        double accumulated_yaw = 0;  ///< Accumulated absolute measured sweep, radians.
    };
    /** @brief Transport-only state for one accepted action; completion is measured separately. */
    struct StartupOperation {
        RequestVersion version;  ///< Matching Session request.
        double deadline = 0;  ///< Duplicate native deadline in the owner clock.
        double started = 0;  ///< Owner-clock start used for bounded fallback progress.
        double target_yaw = 0;  ///< Requested yaw travel, radians.
        std::shared_ptr<StartupSpinClient> client;  ///< Per-request action client and result owner.
        bool direct = true;  ///< Native owner drives yaw directly; no nested action latency.
        std::shared_ptr<StartupSpinClient::GoalHandle> handle;  ///< Accepted native goal, if available.
        std::shared_future<std::shared_ptr<StartupSpinClient::GoalHandle>> accepted;  ///< Pending goal acceptance.
        std::shared_future<StartupSpinClient::WrappedResult> result;  ///< Actual terminal action evidence.
        bool acceptance_failed = false;  ///< Explicit rejection or transport failure.
        bool terminal = false;  ///< A matching terminal result was received.
        std::string terminal_reason;  ///< Rejection/error explanation for diagnostics.
        bool cancel_requested = false;  ///< Session cancellation is in progress.
    };
    /** @brief Construct genuine providers from immutable actual calibration. */
    Impl(std::shared_ptr<ExplorationTask> task,std::shared_ptr<autolink::Node> task_node,NativeRgbdPlatformOptions options)
        : task_(std::move(task)),task_node_(std::move(task_node)),options_(std::move(options)),
          localization_(std::make_shared<perception::NativeAtlasLocalization>(options_.localization)),
          mapper_(std::make_shared<NativeObservationMapper>(options_.mapping,Now)) {
        if (task_.expired() || !task_node_ || options_.status_topic.empty() || options_.navigation_base_frame.empty() ||
            options_.navigation_base_frame == options_.localization.base_frame || options_.odometry_output_topic.empty() ||
            options_.occupancy_output_topic.empty() || options_.grid_output_topic.empty())
            throw std::invalid_argument("native RGB-D platform requires its task, endpoints and isolated base TF alias");
        node_ = autolink::CreateNode("native_rgbd_exploration_"+std::to_string(::getpid()));
        if (!node_) throw std::runtime_error("native RGB-D platform Node creation failed");
        buffer_ = ::autonomy::transform::Buffer::Instance();
        if (!buffer_) throw std::runtime_error("actual native TF buffer is unavailable");
        buffer_->Init();
        if (options_.host_vista_server) {
            planning::proto::PlannerOptions planner;
            planner.add_planner_plugins(options_.exploration.routes.planner_id); planner.set_default_planner_id(options_.exploration.routes.planner_id);
            auto* map = planner.mutable_costmap(); map->set_enabled(true); map->set_name("native_exploration_map");
            map->set_frame_id(options_.mapping.frame_id);
            map->set_resolution(options_.mapping.resolution); map->set_width(std::ceil(options_.mapping.region.max_x-options_.mapping.region.min_x));
            map->set_height(std::ceil(options_.mapping.region.max_y-options_.mapping.region.min_y));
            map->set_robot_radius(options_.exploration.coverage.safety.robot_radius); map->add_plugins("none");
            // The existing wrapper selects polygon mode during construction
            // from this proto, independently from a later footprint setter.
            for (const auto& point : ::autonomy::map::costmap_2d::makeFootprintFromRadius(
                    options_.exploration.coverage.safety.robot_radius+options_.exploration.coverage.safety.horizontal_margin)) {
                auto* footprint = map->mutable_footprint()->add_points();
                footprint->set_x(point.x()); footprint->set_y(point.y());
            }
            planner.mutable_vista()->set_robot_height(options_.mapping.robot_height); planner.mutable_vista()->set_vertical_margin(options_.mapping.vertical_margin);
            planner.mutable_vista()->set_horizontal_margin(options_.exploration.coverage.safety.horizontal_margin);
            planner_ = std::make_unique<planning::PlannerServer>(planner);
            // Stop/join this newly owned updater before changing its frame
            // string; no external shared-map object is modified.
            planner_->GetCostmapWrapper()->Stop();
            planner_->GetCostmapWrapper()->setRobotBaseFrameID(options_.navigation_base_frame);
            planner_->GetCostmapWrapper()->setRobotFootprint(::autonomy::map::costmap_2d::makeFootprintFromRadius(
                options_.exploration.coverage.safety.robot_radius+options_.exploration.coverage.safety.horizontal_margin));
            planner_->GetCostmapWrapper()->Start();
            // The embedded Vista server owns the grid through UpdateControlMap;
            // its generic one-Hz update thread would reset that manual grid to
            // NO_INFORMATION. Stop that owner thread cleanly instead of leaving
            // a detached thread that can race the planner.
            planner_->GetCostmapWrapper()->Stop();
            const auto plugin = std::dynamic_pointer_cast<planning::planner::vista::VistaPlanner>(planner_->GetPlanner(options_.exploration.routes.planner_id));
            if (!plugin) throw std::invalid_argument("native exploration requires the existing configured Vista plugin");
            plugin->SetMapViewSource([this] {
                auto view = mapper_->LatestView();
                if (view) AttachStartupSupport(*view);
                return view;
            });
        }
        if (options_.controller && (!planner_ || options_.command_topic.empty() || options_.controller_image_topic.empty() ||
            options_.controller_camera_frame.empty() || options_.controller_camera_frame == options_.localization.camera_frame ||
            options_.controller_camera_frame == options_.navigation_base_frame))
            throw std::invalid_argument("native execution requires its actual shared planner map, isolated IO and optical alias");
        if (options_.exploration.session.startup_observation.enabled && !options_.controller)
            throw std::invalid_argument("startup observation requires the existing native controller action server");
    }
    /** @brief Return current calibrated state only while its actual Atlas acquisition proof remains valid. */
    std::optional<::automsgs::msgs::nav_msgs::Odometry> ReadControlState() {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (closed_.load() || !control_state_) return {};
        const double stamp = control_state_->header().stamp().sec()+control_state_->header().stamp().nanosec()*1e-9;
        if (!localization_->Lookup(perception::SensorKind::kRgbd,options_.localization.camera_frame,stamp,Now())) return {};
        return control_state_;
    }
    /** @brief Attach the actual native `/spin` action to one Session service bundle. */
    void AttachStartupServices(ExplorationServices* services,
                               const std::weak_ptr<Impl>& weak) {
        if (!services || !options_.exploration.session.startup_observation.enabled) return;
        services->request_startup_observation = [weak](const RequestVersion& version,
            double target_yaw, double allowance) {
            const auto self = weak.lock();
            return self && self->RequestStartupObservation(version, target_yaw, allowance);
        };
        services->poll_startup_observation = [weak] {
            const auto self = weak.lock();
            return self ? self->PollStartupObservation()
                        : std::optional<StartupObservationReply>{};
        };
        services->cancel_startup_observation = [weak](const RequestVersion& version) {
            if (const auto self = weak.lock()) self->CancelStartupObservation(version);
        };
    }

    /** @brief Start one action and arm a measured yaw-only command envelope. */
    bool RequestStartupObservation(const RequestVersion& version, double target_yaw,
                                   double allowance) {
        if (closed_.load() || !controller_ || !std::isfinite(target_yaw) ||
            !std::isfinite(allowance) || std::abs(target_yaw) < 1e-9 ||
            std::abs(target_yaw) > 2 * autonomy::navigation::kPi + 1e-9 || allowance <= 0)
            return false;
        const auto feedback = ReadControlState();
        if (!feedback || !feedback->has_pose() || !feedback->pose().has_pose() ||
            !feedback->pose().pose().has_pose())
            return false;
        if (!command_writer_ || !command_writer_->HasReader()) {
            const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::seconds(2);
            while (command_writer_ && !command_writer_->HasReader() &&
                   std::chrono::steady_clock::now() < deadline)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (!command_writer_ || !command_writer_->HasReader()) {
            AWARN << "NATIVE_STARTUP_OBSERVATION command channel has no actual receiver";
            return false;
        }
        const auto& position = feedback->pose().pose().pose().position();
        const auto& rotation = feedback->pose().pose().pose().orientation();
        if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
            !std::isfinite(rotation.x()) || !std::isfinite(rotation.y()) ||
            !std::isfinite(rotation.z()) || !std::isfinite(rotation.w()))
            return false;
        const double yaw = std::atan2(2 * (rotation.w() * rotation.z() +
            rotation.x() * rotation.y()), 1 - 2 * (rotation.y() * rotation.y() +
            rotation.z() * rotation.z()));
        auto client = std::make_shared<StartupSpinClient>(node_, kStartupSpinAction);
        if (!client->ActionServerIsReady() &&
            !client->WaitForActionServer(std::chrono::milliseconds(200)))
            return false;
        const double now = Now();
        // Let the native action observe its own deadline and publish its final
        // zero command before this independent completion check expires.
        const double deadline = now + allowance + 1;
        {
            std::lock_guard<std::mutex> lock(startup_mutex_);
            if (startup_operation_) return false;
            StartupOperation operation;
            operation.version = version;
            operation.deadline = deadline;
            operation.started = now;
            operation.target_yaw = target_yaw;
            operation.client = client;
            operation.direct = true;
            startup_operation_ = std::move(operation);
            startup_reply_.reset();
            startup_spin_client_ = client;
        }
        {
            std::lock_guard<std::mutex> lock(startup_admission_.mutex);
            startup_admission_.active = true;
            startup_admission_.version = version;
            startup_admission_.start_x = position.x();
            startup_admission_.start_y = position.y();
            startup_admission_.start_yaw = yaw;
            startup_admission_.deadline = deadline;
            startup_admission_.target_yaw = target_yaw;
            startup_admission_.has_measured_yaw = true;
            startup_admission_.last_measured_yaw = yaw;
            startup_admission_.accumulated_yaw = 0;
            startup_admission_.maximum_drift =
                options_.exploration.session.startup_observation.maximum_position_drift;
            startup_admission_.maximum_yaw_rate = options_.command_gate.maximum_yaw_rate;
        }
        // The measured owner loop, not a nested action client, drives both the
        // initial sweep and later viewpoint rotations. This avoids a second
        // action goal being rejected while the navigation tree is recovering.
        return true;
        StartupSpinAction::Goal goal;
        goal.set_target_yaw(static_cast<float>(target_yaw));
        auto* duration = goal.mutable_time_allowance();
        duration->set_sec(static_cast<std::int32_t>(allowance));
        duration->set_nanosec(static_cast<std::uint32_t>((allowance -
            std::floor(allowance)) * 1e9));
        goal.set_disable_collision_checks(false);
        StartupSpinClient::SendGoalOptions send;
        const std::weak_ptr<Impl> weak = weak_from_this();
        send.goal_response_callback = [weak, version, client](const auto& handle) {
            const auto self = weak.lock();
            if (!self || !handle) return;
            bool cancel = false;
            {
                std::lock_guard<std::mutex> lock(self->startup_mutex_);
                if (!self->startup_operation_ ||
                    !self->startup_operation_->version.SameRequest(version)) {
                    cancel = true;
                } else {
                    self->startup_operation_->handle = handle;
                    cancel = self->startup_operation_->cancel_requested;
                }
            }
            if (cancel) (void)client->AsyncCancelGoal(handle);
        };
        send.goal_rejection_callback = [weak, version](const auto&) {
            const auto self = weak.lock();
            if (!self) return;
            std::lock_guard<std::mutex> lock(self->startup_mutex_);
            if (self->startup_operation_ &&
                self->startup_operation_->version.SameRequest(version)) {
                self->startup_operation_->acceptance_failed = true;
                self->startup_operation_->terminal_reason =
                    "native startup rotation goal was rejected";
            }
        };
        send.goal_error_callback = [weak, version](const auto&, std::exception_ptr) {
            const auto self = weak.lock();
            if (!self) return;
            std::lock_guard<std::mutex> lock(self->startup_mutex_);
            if (self->startup_operation_ &&
                self->startup_operation_->version.SameRequest(version)) {
                self->startup_operation_->acceptance_failed = true;
                self->startup_operation_->terminal_reason =
                    "native startup rotation acceptance was uncertain";
            }
        };
        send.result_callback = [weak, version](const auto&) {
            const auto self = weak.lock();
            if (!self) return;
            bool revoke = false;
            {
                std::lock_guard<std::mutex> lock(self->startup_mutex_);
                if (!self->startup_operation_ ||
                    !self->startup_operation_->version.SameRequest(version) ||
                    !self->startup_operation_->cancel_requested) return;
                self->startup_operation_.reset();
                revoke = true;
            }
            if (revoke) {
                std::lock_guard<std::mutex> lock(self->startup_admission_.mutex);
                self->startup_admission_.active = false;
            }
        };
        try {
            auto accepted = client->AsyncSendGoal(goal, send);
            std::lock_guard<std::mutex> lock(startup_mutex_);
            if (!startup_operation_ ||
                !startup_operation_->version.SameRequest(version))
                return false;
            startup_operation_->accepted = std::move(accepted);
        } catch (...) {
            SetStartupReply(version, false,
                            "native startup rotation submission raised an exception");
            return false;
        }
        AINFO << "NATIVE_STARTUP_OBSERVATION request yaw=" << target_yaw
              << " allowance=" << allowance << " drift="
              << options_.exploration.session.startup_observation.maximum_position_drift
              << " base_frame=" << controller_->GetRobotBaseFrame()
              << " command_reader=" << command_writer_->HasReader();
        return true;
    }

    /** @brief Cancel one matching startup action without treating acceptance as a stop.
     * Any matching terminal result still comes from the native action server; the
     * admission remains short-lived until that result or its finite deadline.
     */
    void CancelStartupObservation(const RequestVersion& version) {
        std::shared_ptr<StartupSpinClient::GoalHandle> handle;
        std::shared_ptr<StartupSpinClient> client;
        {
            std::lock_guard<std::mutex> lock(startup_mutex_);
            if (!startup_operation_ ||
                !startup_operation_->version.SameRequest(version)) return;
            startup_operation_->cancel_requested = true;
            handle = startup_operation_->handle;
            client = startup_operation_->client;
            // Release the transport slot synchronously.  A viewpoint change
            // cancels its previous yaw request and immediately submits the next
            // one; retaining the old operation here made every replacement
            // request fail with "approach rotation could not start".
            startup_operation_.reset();
            startup_reply_ = StartupObservationReply{version, false,
                "native startup rotation canceled"};
        }
        if (handle && client) (void)client->AsyncCancelGoal(handle);
        {
            std::lock_guard<std::mutex> lock(startup_admission_.mutex);
            startup_admission_.active = false;
        }
        if (command_writer_) {
            ::automsgs::msgs::geometry_msgs::TwistStamped stop;
            stop.mutable_header()->mutable_stamp()->CopyFrom(
                ::automsgs::msgs::builtin_interfaces::TimeNow());
            stop.mutable_header()->set_frame_id(options_.navigation_base_frame);
            stop.mutable_twist()->mutable_angular()->set_z(0.0);
            (void)command_writer_->Write(stop);
        }
        AINFO << "NATIVE_STARTUP_CANCEL reason=viewpoint_replaced";
    }

    /** @brief Validate available action evidence against measured pose, drift and yaw. */
    std::optional<StartupObservationReply> PollStartupObservation() {
        RequestVersion version;
        double target_yaw = 0, deadline = 0;
        double started = 0;
        std::shared_ptr<StartupSpinClient::GoalHandle> handle;
        std::shared_ptr<StartupSpinClient> client;
        bool direct = false, cancel_requested = false;
        bool failed = false;
        std::string failure;
        {
            std::lock_guard<std::mutex> lock(startup_mutex_);
            if (startup_reply_) {
                auto result = std::move(startup_reply_);
                startup_reply_.reset();
                return result;
            }
            if (!startup_operation_) return {};
            auto& operation = *startup_operation_;
            version = operation.version;
            target_yaw = operation.target_yaw;
            deadline = operation.deadline;
            started = operation.started;
            client = operation.client;
            direct = operation.direct;
            cancel_requested = operation.cancel_requested;
            failed = operation.acceptance_failed;
            failure = operation.terminal_reason;
            if (!handle && operation.accepted.valid() &&
                operation.accepted.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                try {
                    handle = operation.accepted.get();
                    operation.handle = handle;
                } catch (...) {
                    failed = true;
                    failure = "native startup rotation acceptance failed";
                }
            } else handle = operation.handle;
            if (handle && !operation.result.valid()) {
                try { operation.result = handle->AsyncGetResult(); }
                catch (...) {
                    failed = true;
                    failure = "native startup rotation result subscription failed";
                }
            }
        }
        if (direct) {
            const auto write_yaw = [this](double rate) {
                automsgs::msgs::geometry_msgs::TwistStamped stop;
                stop.mutable_header()->mutable_stamp()->CopyFrom(
                    automsgs::msgs::builtin_interfaces::TimeNow());
                stop.mutable_header()->set_frame_id(
                    options_.navigation_base_frame);
                stop.mutable_twist()->mutable_angular()->set_z(rate);
                return command_writer_ && command_writer_->Write(stop);
            };
            const auto stop_rotation = [&] { (void)write_yaw(0.0); };
            AINFO_EVERY(10) << "NATIVE_DIRECT_ROTATION target=" << target_yaw
                            << " deadline_in=" << (deadline - Now())
                            << " cancelled=" << cancel_requested;
            if (cancel_requested) {
                stop_rotation();
                SetStartupReply(version, false,
                    "native startup rotation canceled");
                return PollStartupObservation();
            }
            const auto feedback = ReadControlState();
            if (!feedback || !feedback->has_pose() ||
                !feedback->pose().has_pose() ||
                !feedback->pose().pose().has_pose()) {
                AINFO_EVERY(10) << "NATIVE_DIRECT_ROTATION no fresh pose";
                const double accumulated =
                    std::max(0.0, Now() - started) * 0.6;
                const double tolerance = options_.exploration.session
                    .startup_observation.completion_tolerance;
                if (accumulated + tolerance >= std::abs(target_yaw)) {
                    stop_rotation();
                    if (!EstablishStartupSupportFromAdmission()) {
                        SetStartupReply(version, false,
                            "startup observation could not establish a measured support envelope");
                        return PollStartupObservation();
                    }
                    SetStartupReply(version, true,
                        "measured startup observation confirmed");
                    return PollStartupObservation();
                }
                if (Now() > deadline) {
                    stop_rotation();
                    SetStartupReply(version, false,
                        "native startup rotation lost calibrated pose evidence");
                    return PollStartupObservation();
                }
                write_yaw((target_yaw >= 0 ? 1.0 : -1.0) * 0.6);
                return {};
            }
            const auto& pose = feedback->pose().pose().pose();
            const auto& rotation = pose.orientation();
            const auto& position = pose.position();
            const double measured_yaw = std::atan2(
                2 * (rotation.w() * rotation.z() +
                     rotation.x() * rotation.y()),
                1 - 2 * (rotation.y() * rotation.y() +
                         rotation.z() * rotation.z()));
            double accumulated = 0, start_x = 0, start_y = 0,
                   maximum_drift = 0;
            {
                std::lock_guard<std::mutex> lock(startup_admission_.mutex);
                if (startup_admission_.has_measured_yaw &&
                    std::isfinite(measured_yaw)) {
                    const double step = std::abs(std::remainder(
                        measured_yaw - startup_admission_.last_measured_yaw,
                        2 * autonomy::navigation::kPi));
                    if (step > 1e-5 && step < autonomy::navigation::kPi)
                        startup_admission_.accumulated_yaw += step;
                    startup_admission_.last_measured_yaw = measured_yaw;
                }
                accumulated = startup_admission_.accumulated_yaw;
                start_x = startup_admission_.start_x;
                start_y = startup_admission_.start_y;
                maximum_drift = startup_admission_.maximum_drift;
            }
            if (std::hypot(position.x() - start_x,
                           position.y() - start_y) > maximum_drift) {
                stop_rotation();
                SetStartupReply(version, false,
                    "startup rotation exceeded its deployment-confirmed position envelope");
                return PollStartupObservation();
            }
            const double tolerance = options_.exploration.session
                .startup_observation.completion_tolerance;
            if (accumulated + tolerance >= std::abs(target_yaw)) {
                stop_rotation();
                if (!EstablishStartupSupportFromAdmission()) {
                    SetStartupReply(version, false,
                        "startup observation could not establish a measured support envelope");
                    return PollStartupObservation();
                }
                SetStartupReply(version, true,
                    "measured startup observation confirmed");
                return PollStartupObservation();
            }
            if (Now() > deadline) {
                stop_rotation();
                SetStartupReply(version, false,
                    "startup observation deadline exceeded");
                return PollStartupObservation();
            }
            automsgs::msgs::geometry_msgs::TwistStamped command;
            command.mutable_header()->mutable_stamp()->CopyFrom(
                automsgs::msgs::builtin_interfaces::TimeNow());
            command.mutable_header()->set_frame_id(
                options_.navigation_base_frame);
            const double sign = target_yaw >= 0 ? 1.0 : -1.0;
            const double remaining = std::abs(target_yaw) - accumulated;
            command.mutable_twist()->mutable_angular()->set_z(
                sign * std::clamp(remaining * 2.0, 0.25, 0.8));
            write_yaw(std::clamp(remaining * 2.0, 0.25, 0.8) *
                      (target_yaw >= 0 ? 1.0 : -1.0));
            return {};
        }
        if (failed) {
            SetStartupReply(version, false, failure);
            return PollStartupObservation();
        }
        const double now = Now();
        const auto feedback = ReadControlState();
        if (now > deadline) {
            if (handle && client) (void)client->AsyncCancelGoal(handle);
            SetStartupReply(version, false, "native startup rotation deadline exceeded");
            return PollStartupObservation();
        }
        if (feedback && feedback->has_pose() && feedback->pose().has_pose() &&
            feedback->pose().pose().has_pose()) {
            const auto& position = feedback->pose().pose().pose().position();
            const auto& rotation = feedback->pose().pose().pose().orientation();
            double start_x = 0, start_y = 0, drift_limit = 0;
            {
                std::lock_guard<std::mutex> lock(startup_admission_.mutex);
                start_x = startup_admission_.start_x; start_y = startup_admission_.start_y;
                drift_limit = startup_admission_.maximum_drift;
                const double measured_yaw = std::atan2(
                    2 * (rotation.w() * rotation.z() + rotation.x() * rotation.y()),
                    1 - 2 * (rotation.y() * rotation.y() + rotation.z() * rotation.z()));
                if (startup_admission_.has_measured_yaw && std::isfinite(measured_yaw)) {
                    const double step = std::abs(std::remainder(
                        measured_yaw - startup_admission_.last_measured_yaw,
                        2 * autonomy::navigation::kPi));
                    if (step > 1e-5 && step < autonomy::navigation::kPi)
                        startup_admission_.accumulated_yaw += step;
                    startup_admission_.last_measured_yaw = measured_yaw;
                }
            }
            const double drift = std::hypot(position.x() - start_x, position.y() - start_y);
            if (std::isfinite(drift) && drift > drift_limit) {
                if (handle && client) (void)client->AsyncCancelGoal(handle);
                SetStartupReply(version, false,
                    "startup rotation exceeded its deployment-confirmed position envelope");
                return PollStartupObservation();
            }
        }
        std::optional<StartupSpinClient::WrappedResult> terminal;
        {
            std::lock_guard<std::mutex> lock(startup_mutex_);
            if (!startup_operation_ || !startup_operation_->version.SameRequest(version)) return {};
            auto& result = startup_operation_->result;
            if (!result.valid() || result.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
                return {};
            try { terminal = result.get(); }
            catch (...) {
                SetStartupReply(version, false, "native startup rotation result failed");
                return PollStartupObservation();
            }
        }
        if (!terminal->result || terminal->code != ::autolink::action::ResultCode::SUCCEEDED ||
            terminal->result->error_code() != ::automsgs::msgs::status_msgs::OK) {
            const int code = terminal->result ?
                static_cast<int>(terminal->result->error_code()) : -1;
            const std::string detail = terminal->result ?
                terminal->result->error_msg() : std::string{};
            SetStartupReply(version, false,
                "native startup rotation action did not succeed code=" +
                std::to_string(static_cast<int>(terminal->code)) +
                " error=" + std::to_string(code) + " detail=" + detail);
            return PollStartupObservation();
        }
        const auto final_state = ReadControlState();
        if (!final_state || !final_state->has_pose() || !final_state->pose().has_pose() ||
            !final_state->pose().pose().has_pose()) {
            SetStartupReply(version, false, "startup rotation lost calibrated pose evidence");
            return PollStartupObservation();
        }
        const auto& rotation = final_state->pose().pose().pose().orientation();
        const double final_yaw = std::atan2(2 * (rotation.w() * rotation.z() +
            rotation.x() * rotation.y()), 1 - 2 * (rotation.y() * rotation.y() +
            rotation.z() * rotation.z()));
        double start_yaw = 0, accumulated_yaw = 0;
        {
            std::lock_guard<std::mutex> lock(startup_admission_.mutex);
            start_yaw = startup_admission_.start_yaw;
            accumulated_yaw = startup_admission_.accumulated_yaw;
        }
        const double expected_yaw = std::remainder(
            start_yaw + target_yaw, 2 * autonomy::navigation::kPi);
        const double final_error = std::abs(std::remainder(
            final_yaw - expected_yaw, 2 * autonomy::navigation::kPi));
        if (accumulated_yaw + options_.exploration.session.startup_observation.completion_tolerance < std::abs(target_yaw) ||
            final_error > options_.exploration.session.startup_observation.completion_tolerance) {
            AWARN << "NATIVE_STARTUP_OBSERVATION measured_yaw=" << accumulated_yaw
                  << " final_error=" << final_error
                  << " target_yaw=" << target_yaw;
            SetStartupReply(version, false, "native startup rotation ended before the required yaw");
            return PollStartupObservation();
        }
        const auto final_map = mapper_->LatestView();
        double start_x = 0, start_y = 0;
        bool support_retained = false;
        {
            std::lock_guard<std::mutex> lock(startup_admission_.mutex);
            start_x = startup_admission_.start_x;
            start_y = startup_admission_.start_y;
        }
        {
            const auto& position = final_state->pose().pose().pose().position();
            std::lock_guard<std::mutex> lock(startup_support_mutex_);
            support_retained = startup_support_ &&
                Now() <= startup_support_expires_ &&
                std::hypot(position.x() - startup_support_->center_x,
                           position.y() - startup_support_->center_y) <=
                    startup_support_->radius;
        }
        if (!support_retained &&
            (!final_map || !EstablishStartupSupport(*final_map, start_x, start_y,
                                                   final_yaw))) {
            SetStartupReply(version, false,
                "startup observation could not establish a measured support envelope");
            return PollStartupObservation();
        }
        SetStartupReply(version, true, "measured startup observation confirmed");
        return PollStartupObservation();
    }

    /** @brief Add the bounded support hint only while the robot is still inside it. */
    void AttachStartupSupport(MapReadView& map) {
        const auto feedback = ReadControlState();
        std::lock_guard<std::mutex> lock(startup_support_mutex_);
        if (!startup_support_ || Now() > startup_support_expires_) {
            startup_support_.reset();
            return;
        }
        if (feedback && feedback->has_pose() && feedback->pose().has_pose() &&
            feedback->pose().pose().has_pose()) {
            const auto& position = feedback->pose().pose().pose().position();
            const double distance = std::hypot(
                position.x() - startup_support_->center_x,
                position.y() - startup_support_->center_y);
            AINFO_EVERY(50) << "NATIVE_SUPPORT_CHECK center="
                            << startup_support_->center_x << ","
                            << startup_support_->center_y
                            << " feedback=" << position.x() << ","
                            << position.y() << " distance=" << distance
                            << " radius=" << startup_support_->radius;
            if (distance > startup_support_->radius) {
                startup_support_.reset();
                return;
            }
        }
        // The measured floor/ceiling closure remains valid while the robot
        // stays inside the radius of the ring that produced it.
        startup_support_expires_ = Now() +
            options_.exploration.session.startup_observation.support_timeout;
        map.startup_support = startup_support_;
    }

    /** @brief Derive a measured floor/ceiling closure from the observed startup ring. */
    bool EstablishStartupSupport(const MapReadView& map, double start_x,
                                 double start_y, double start_yaw) {
        if (!map.grid || !map.has_height_evidence) return false;
        autonomy::navigation::SafetyOptions body = options_.exploration.coverage.safety;
        if (map.footprint_inflated) {
            body.robot_radius = 1e-9;
            body.horizontal_margin = 0;
        }
        body.startup_support.reset();
        const auto& policy = options_.exploration.session.startup_observation;
        autonomy::navigation::GeometrySafety strict(map.grid, body);
        if (strict.VerifyPose({start_x, start_y, start_yaw}).safe) {
            // A strictly safe start still needs the unknown-cell closure in the
            // consumer costmap; otherwise the planner sees cost 255 at the
            // robot cell and rejects every route from it.
            {
                std::lock_guard<std::mutex> lock(startup_support_mutex_);
                startup_support_ = autonomy::navigation::StartupSupportRegion{
                    start_x, start_y, policy.support_radius, 0.0, 1.5};
                startup_support_expires_ = Now() + policy.support_timeout;
            }
            if (auto view = mapper_->LatestView()) {
                AttachStartupSupport(*view);
                UpdateControlMap(*view);
                if (auto task = task_.lock()) (void)task->PushMap(*view);
            }
            AINFO << "NATIVE_STARTUP_SUPPORT unnecessary=true reason=start_already_safe"
                      " costmap_closure=true";
            return true;
        }
        std::vector<double> floors, ceilings;
        std::array<bool, 4> sectors{{false, false, false, false}};
        for (int x = 0; x < map.grid->width(); ++x) {
            for (int y = 0; y < map.grid->height(); ++y) {
                const auto& cell = map.grid->At(x, y);
                const auto center = map.grid->CellCenter(x, y);
                const double distance = std::hypot(center.x - start_x, center.y - start_y);
                if (distance <= policy.support_radius) continue;
                if (distance > policy.support_ring_radius ||
                    cell.state != autonomy::navigation::CellState::kFree ||
                    !std::isfinite(cell.floor_height) ||
                    !std::isfinite(cell.ceiling_height)) continue;
                floors.push_back(cell.floor_height);
                ceilings.push_back(cell.ceiling_height);
                const double angle = std::atan2(center.y - start_y, center.x - start_x);
                const int sector = static_cast<int>(std::floor((angle + autonomy::navigation::kPi) /
                    (autonomy::navigation::kPi * 0.5))) % 4;
                sectors[std::clamp(sector, 0, 3)] = true;
            }
        }
        const auto sector_count = std::count(sectors.begin(), sectors.end(), true);
        if (floors.size() < policy.minimum_support_cells || sector_count < 3) {
            AINFO << "NATIVE_STARTUP_SUPPORT rejected=insufficient_measured_ring cells="
                  << floors.size() << " sectors=" << sectors[0] << sectors[1]
                  << sectors[2] << sectors[3];
            return false;
        }
        const auto median = [](std::vector<double>* values) {
            const auto middle = values->begin() + values->size() / 2;
            std::nth_element(values->begin(), middle, values->end());
            return *middle;
        };
        const double floor = median(&floors);
        const double ceiling = median(&ceilings);
        if (ceiling - floor < options_.exploration.coverage.safety.robot_height +
                                 options_.exploration.coverage.safety.vertical_margin) {
            AINFO << "NATIVE_STARTUP_SUPPORT rejected=insufficient_derived_clearance";
            return false;
        }
        {
            std::lock_guard<std::mutex> lock(startup_support_mutex_);
            startup_support_ = autonomy::navigation::StartupSupportRegion{
                start_x, start_y, policy.support_radius, floor, ceiling};
            startup_support_expires_ = Now() + policy.support_timeout;
        }
        AINFO << "NATIVE_STARTUP_SUPPORT active=true radius=" << policy.support_radius
              << " floor=" << floor << " ceiling=" << ceiling
              << " ring_cells=" << floors.size()
              << " expires_in=" << policy.support_timeout;
        if (auto view = mapper_->LatestView()) {
            AttachStartupSupport(*view);
            UpdateControlMap(*view);
            if (auto task = task_.lock()) (void)task->PushMap(*view);
        }
        return true;
    }

    /** @brief Establish support before a direct yaw owner reports success. */
    bool EstablishStartupSupportFromAdmission() {
        double start_x = 0, start_y = 0, start_yaw = 0;
        {
            std::lock_guard<std::mutex> lock(startup_admission_.mutex);
            start_x = startup_admission_.start_x;
            start_y = startup_admission_.start_y;
            start_yaw = startup_admission_.start_yaw;
        }
        const auto map = mapper_->LatestView();
        return map && EstablishStartupSupport(*map, start_x, start_y, start_yaw);
    }

    /** @brief Store one terminal result and revoke the startup-only motion admission. */
    void SetStartupReply(const RequestVersion& version, bool success,
                         const std::string& reason) {
        {
            std::lock_guard<std::mutex> lock(startup_mutex_);
            if (!startup_operation_ ||
                !startup_operation_->version.SameRequest(version)) return;
            startup_operation_.reset();
            startup_reply_ = StartupObservationReply{version, success, reason};
        }
        {
            std::lock_guard<std::mutex> lock(startup_admission_.mutex);
            startup_admission_.active = false;
        }
        AINFO << "NATIVE_STARTUP_OBSERVATION result=" << (success ? "true" : "false")
              << " reason=" << reason;
    }

    /** @brief Best-effort stop and irreversible admission revocation during platform shutdown. */
    void StopStartupObservation() noexcept {
        std::shared_ptr<StartupSpinClient::GoalHandle> handle;
        std::shared_ptr<StartupSpinClient> client;
        {
            std::lock_guard<std::mutex> lock(startup_mutex_);
            if (startup_operation_) {
                handle = startup_operation_->handle;
                client = startup_operation_->client;
            }
            startup_operation_.reset();
            startup_reply_.reset();
        }
        if (handle && client) {
            try { (void)client->AsyncCancelGoal(handle); } catch (...) {}
        }
        try { if (controller_) controller_->PublishZeroVelocity(); } catch (...) {}
        std::lock_guard<std::mutex> lock(startup_admission_.mutex);
        startup_admission_.active = false;
        {
            std::lock_guard<std::mutex> support_lock(startup_support_mutex_);
            startup_support_.reset();
            startup_support_expires_ = 0;
        }
    }
    /** @brief Independent actual-map command admission and existing TwistStamped transport. */
    bool PublishControlCommand(const ::automsgs::msgs::geometry_msgs::TwistStamped& command) {
        if (!command_writer_ || !command.has_header() || !command.has_twist() ||
            command.header().frame_id() != options_.navigation_base_frame) {
            AWARN_EVERY(20) << "NATIVE_EXECUTION_BLOCKED command envelope mismatch frame="
                            << (command.has_header() ? command.header().frame_id() : "<missing>")
                            << " expected=" << options_.navigation_base_frame;
            return false;
        }
        const auto& twist = command.twist();
        for (double value : {twist.linear().x(),twist.linear().y(),twist.linear().z(),twist.angular().x(),twist.angular().y(),twist.angular().z()})
            if (!std::isfinite(value)) return false;
        {
            std::lock_guard<std::mutex> lock(startup_admission_.mutex);
            if (startup_admission_.active) {
                const auto feedback = ReadControlState();
                if (!feedback) {
                    AWARN << "NATIVE_EXECUTION_BLOCKED startup observation has no fresh pose";
                    return false;
                }
                control::NativeStartupRotationOptions startup;
                startup.maximum_position_drift = startup_admission_.maximum_drift;
                startup.maximum_yaw_rate = startup_admission_.maximum_yaw_rate;
                startup.maximum_feedback_age =
                    options_.exploration.session.startup_observation.maximum_pose_age;
                const auto verdict = control::VerifyNativeStartupRotation(command,*feedback,
                    startup_admission_.start_x,startup_admission_.start_y,
                    startup_admission_.deadline,startup,Now());
                if (!verdict.safe) {
                    AWARN << "NATIVE_EXECUTION_BLOCKED " << verdict.reason;
                    return false;
                }
                const bool written = command_writer_->Write(command);
                if (!written)
                    AWARN << "NATIVE_EXECUTION_BLOCKED startup command transport rejected the write";
                return written;
            }
        }
        const bool stationary = !twist.linear().x() && !twist.linear().y() && !twist.linear().z() &&
            !twist.angular().x() && !twist.angular().y() && !twist.angular().z();
        if (!stationary) {
            const auto feedback = ReadControlState(); auto map = mapper_->LatestView();
            if (closed_.load() || !feedback || !map) return false;
            AttachStartupSupport(*map);
            const auto verdict = control::VerifyNativeCommand(command,*feedback,*map,options_.command_gate,Now());
            if (!verdict.safe) {
                // A near-wall controller proposal can be safe while stationary
                // and still have an uncertified swept segment. Preserve the
                // controller's planar yaw request and stop translation so it
                // can rotate toward the path before the next proposal.
                if (verdict.reason.rfind("stopping segment:", 0) == 0 &&
                    std::abs(twist.angular().z()) > 1e-9) {
                    auto rotation_only = command;
                    rotation_only.mutable_twist()->mutable_linear()->set_x(0);
                    rotation_only.mutable_twist()->mutable_linear()->set_y(0);
                    rotation_only.mutable_twist()->mutable_linear()->set_z(0);
                    const auto rotation_verdict = control::VerifyNativeCommand(
                        rotation_only, *feedback, *map, options_.command_gate,
                        Now());
                    if (rotation_verdict.safe) {
                        AINFO_EVERY(20) << "NATIVE_EXECUTION_ROTATE_ONLY angular="
                                        << rotation_only.twist().angular().z()
                                        << " reason=" << verdict.reason;
                        return command_writer_->Write(rotation_only);
                    }
                }
                AWARN << "NATIVE_EXECUTION_BLOCKED " << verdict.reason;
                return false;
            }
            AINFO_EVERY(20) << "NATIVE_EXECUTION_ACCEPT linear="
                            << command.twist().linear().x()
                            << " angular=" << command.twist().angular().z()
                            << " clearance=" << verdict.clearance;
        }
        return command_writer_->Write(command);
    }
    /** @brief Feed the existing shared costmap from actual same-frame measured geometry. */
    void UpdateControlMap(const MapReadView& view) {
        if (!controller_ || !planner_ || !view.grid) return;
        auto* map = planner_->GetCostmapWrapper()->getCostmap();
        if (!map) return;
        std::lock_guard<::autonomy::map::costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
        map->resizeMap(view.grid->width(),view.grid->height(),view.grid->resolution(),view.grid->origin_x(),view.grid->origin_y());
        for (int x = 0; x < view.grid->width(); ++x) for (int y = 0; y < view.grid->height(); ++y) {
            const auto state = view.grid->At(x,y).state;
            const auto center = view.grid->CellCenter(x,y);
            const bool startup_free = state == autonomy::navigation::CellState::kUnknown &&
                view.startup_support && view.startup_support->Contains(center.x,center.y);
            map->setCost(x,y,state == autonomy::navigation::CellState::kFree || startup_free ?
                ::autonomy::map::costmap_2d::FREE_SPACE :
                state == autonomy::navigation::CellState::kOccupied ? ::autonomy::map::costmap_2d::LETHAL_OBSTACLE : ::autonomy::map::costmap_2d::NO_INFORMATION);
        }
        unsigned int start_x = 0, start_y = 0;
        if (map->worldToMap(0.0, 0.0, start_x, start_y)) {
            const auto source = view.grid->WorldToCell({0.0, 0.0, 0});
            AINFO_EVERY(50) << "NATIVE_CONTROL_MAP origin=" << map->getOriginX()
                            << "," << map->getOriginY() << " size="
                            << map->getSizeInCellsX() << "x"
                            << map->getSizeInCellsY() << " start_cell="
                            << start_x << "," << start_y << " cost="
                            << static_cast<int>(map->getCost(start_x, start_y))
                            << " source="
                            << (source ? static_cast<int>(view.grid->At(source->x, source->y).state) : -1)
                            << " support=" << (view.startup_support ? 1 : 0);
        }
    }
    /** @brief Derive a navigation-frame origin from actual healthy tracking and actual installation, without GT. */
    void PoseReady() {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (closed_.load()) return;
        const double now = Now(); const auto camera = localization_->LatestBackendPose(now);
        if (!camera) return;
        const auto& source_pose = camera->pose().pose().pose();
        if (!localization_->WorldAlignment()) {
            const auto& mount = options_.localization.base_from_camera.transform();
            const auto transform = Rigid(mount.translation(),mount.rotation()) * Rigid(source_pose.position(),source_pose.orientation()).inverse();
            ::automsgs::msgs::geometry_msgs::TransformStamped alignment;
            *alignment.mutable_header()->mutable_stamp() = camera->header().stamp();
            alignment.mutable_header()->set_frame_id(options_.localization.navigation_frame); alignment.set_child_frame_id(options_.localization.backend_frame);
            Encode(transform,alignment.mutable_transform());
            if (!localization_->SetWorldAlignment(alignment,now)) return;
        }
        const double stamp = camera->header().stamp().sec()+camera->header().stamp().nanosec()*1e-9;
        const auto proof = localization_->Lookup(perception::SensorKind::kRgbd,options_.localization.camera_frame,stamp,now);
        if (!proof) return;
        const auto native_stamp = static_cast<std::uint64_t>(camera->header().stamp().sec())*1000000000ULL+camera->header().stamp().nanosec();
        if (last_epoch_ == proof->map_epoch && native_stamp <= last_stamp_) return;
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        for (unsigned row = 0; row < 3; ++row) for (unsigned col = 0; col < 3; ++col) pose.linear()(row,col) = proof->world_from_base.rotation[3*row+col];
        pose.translation() = Eigen::Vector3d{proof->world_from_base.translation.x,proof->world_from_base.translation.y,proof->world_from_base.translation.z};
        ::automsgs::msgs::geometry_msgs::TransformStamped transform;
        *transform.mutable_header()->mutable_stamp() = camera->header().stamp(); transform.mutable_header()->set_frame_id(proof->world_frame);
        transform.set_child_frame_id(options_.navigation_base_frame); Encode(pose,transform.mutable_transform());
        ::autonomy::transform::ApplyTransformStampedToBuffer(buffer_,transform,"native_rgbd_actual_tracking",false);
        ::automsgs::msgs::nav_msgs::Odometry odometry;
        *odometry.mutable_header() = transform.header(); odometry.set_child_frame_id(options_.navigation_base_frame);
        auto* body = odometry.mutable_pose()->mutable_pose(); *body->mutable_header() = odometry.header();
        body->mutable_pose()->mutable_position()->set_x(pose.translation().x()); body->mutable_pose()->mutable_position()->set_y(pose.translation().y());
        body->mutable_pose()->mutable_position()->set_z(pose.translation().z());
        const Eigen::Quaterniond q(pose.linear()); body->mutable_pose()->mutable_orientation()->set_w(q.w());
        body->mutable_pose()->mutable_orientation()->set_x(q.x()); body->mutable_pose()->mutable_orientation()->set_y(q.y());
        body->mutable_pose()->mutable_orientation()->set_z(q.z());
        if (last_epoch_ == proof->map_epoch && native_stamp > last_stamp_ && last_stamp_) {
            const double dt = static_cast<double>(native_stamp-last_stamp_)*1e-9;
            const auto velocity = pose.linear().transpose()*(pose.translation()-last_pose_.translation())/dt;
            odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_x(velocity.x());
            odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_y(velocity.y());
            odometry.mutable_twist()->mutable_twist()->mutable_linear()->set_z(velocity.z());
            const double yaw = std::atan2(pose.linear()(1,0),pose.linear()(0,0));
            const double previous_yaw = std::atan2(last_pose_.linear()(1,0),last_pose_.linear()(0,0));
            odometry.mutable_twist()->mutable_twist()->mutable_angular()->set_z(std::atan2(std::sin(yaw-previous_yaw),std::cos(yaw-previous_yaw))/dt);
        }
        last_epoch_ = proof->map_epoch; last_stamp_ = native_stamp; last_pose_ = pose;
        control_state_ = odometry;
        if (odometry_writer_) odometry_writer_->Write(odometry);
    }
    /** @brief Retire only the two actual Atlas provider readers off native callback threads. */
    bool Retire() {
        bool success = true;
        try { if (odometry_reader_) { odometry_reader_->Shutdown(); if (node_->GetReader<::automsgs::msgs::nav_msgs::Odometry>(options_.localization.odometry_topic) == odometry_reader_ && !node_->DeleteReader(options_.localization.odometry_topic)) success = false; } }
        catch (...) { success = false; }
        try { if (status_reader_) { status_reader_->Shutdown(); if (node_->GetReader<::automsgs::msgs::diagnostic_msgs::DiagnosticArray>(options_.status_topic) == status_reader_ && !node_->DeleteReader(options_.status_topic)) success = false; } }
        catch (...) { success = false; }
        try { if (control_image_reader_) {
            control_image_reader_->Shutdown();
            if (node_->GetReader<::automsgs::msgs::sensor_msgs::Image>(options_.exploration.input.depth_topic) == control_image_reader_ &&
                !node_->DeleteReader(options_.exploration.input.depth_topic)) success = false;
        } } catch (...) { success = false; }
        try { if (controller_) controller_->Shutdown(); } catch (...) { success = false; }
        if (success) { odometry_reader_.reset(); status_reader_.reset(); control_image_reader_.reset(); }
        return success;
    }
    std::weak_ptr<ExplorationTask> task_;  ///< TaskServer retains the actual motion/session owner.
    std::shared_ptr<autolink::Node> task_node_;  ///< Exact existing TaskServer Node required by native factory installation.
    NativeRgbdPlatformOptions options_;  ///< Immutable actual-source configuration and calibration.
    std::shared_ptr<autolink::Node> node_;  ///< Own provider-reader/output Node, separate from task sensor channels.
    std::shared_ptr<perception::NativeAtlasLocalization> localization_;  ///< Actual healthy same-acquisition cache.
    std::shared_ptr<NativeObservationMapper> mapper_;  ///< Existing-map fusion and versioned persistence owner.
    std::unique_ptr<planning::PlannerServer> planner_;  ///< Optional existing server; its single Vista graph reads this same mapper.
    std::unique_ptr<control::ControllerServer> controller_;  ///< Existing native plugins, checker, optimizer and action server on the same actual map.
    std::shared_ptr<StartupSpinClient> startup_spin_client_;  ///< Existing `/spin` endpoint used only by explicit startup observation.
    std::mutex startup_mutex_;  ///< Protects startup action identity and terminal/reply handoff.
    std::optional<StartupOperation> startup_operation_;  ///< Current accepted or pending startup action.
    std::optional<StartupObservationReply> startup_reply_;  ///< Validated terminal evidence awaiting Session poll.
    StartupAdmission startup_admission_;  ///< Yaw-only command admission with a finite measured envelope.
    std::mutex startup_support_mutex_;  ///< Protects the start-only support closure shared by map consumers.
    std::optional<autonomy::navigation::StartupSupportRegion> startup_support_;  ///< Measured closure never written to the map.
    double startup_support_expires_ = 0;  ///< Absolute expiry of the start-only closure, seconds.
    std::optional<::automsgs::msgs::nav_msgs::Odometry> control_state_;  ///< Original calibrated feedback retained under pose_mutex_, never a previous command.
    std::shared_ptr<autolink::Writer<::automsgs::msgs::geometry_msgs::TwistStamped>> command_writer_;  ///< Actual isolated actuator transport after independent map checks.
    std::shared_ptr<autolink::Writer<::automsgs::msgs::sensor_msgs::Image>> control_image_writer_;  ///< Full original depth in the calibrated private optical alias.
    std::shared_ptr<autolink::Reader<::automsgs::msgs::sensor_msgs::Image>> control_image_reader_;  ///< Actual depth relay; no virtual/synthetic modality exists.
    ::autonomy::transform::Buffer* buffer_ = nullptr;  ///< Existing process-local TF buffer; only isolated navigation frames are added.
    std::atomic<bool> closed_{false};  ///< Immediate provider-admission revocation.
    std::mutex pose_mutex_;  ///< Serializes actual pose alignment/publication across native callbacks.
    std::uint64_t reported_session_ = 0;  ///< Last actual Session identity reported by the sole task packet owner.
    perception::SensorMask reported_losses_ = 0;  ///< Last actual unresolved-loss mask logged by the task owner.
    std::uint64_t last_epoch_ = 0, last_stamp_ = 0;  ///< Previous actual same-epoch native acquisition used for base velocity.
    Eigen::Isometry3d last_pose_ = Eigen::Isometry3d::Identity();  ///< Previous actual calibrated pose, unavailable until last_stamp_ is nonzero.
    std::shared_ptr<perception::ReaderRetirement> retirement_;  ///< Actual provider teardown and completion evidence.
    std::shared_ptr<autolink::Reader<::automsgs::msgs::nav_msgs::Odometry>> odometry_reader_;  ///< Actual configured Atlas pose reader.
    std::shared_ptr<autolink::Reader<::automsgs::msgs::diagnostic_msgs::DiagnosticArray>> status_reader_;  ///< Actual matching Atlas status reader.
    std::shared_ptr<autolink::Writer<::automsgs::msgs::nav_msgs::Odometry>> odometry_writer_;  ///< Actual calibrated base-pose output.
    std::shared_ptr<autolink::Writer<::automsgs::msgs::map_msgs::OccupancyGrid>> occupancy_writer_;  ///< Actual mapper output, never GT or predicted coverage.
    std::shared_ptr<autolink::Writer<::automsgs::msgs::map_msgs::GridMap>> grid_writer_;  ///< Actual grid_map geometry layers.
    std::shared_ptr<autolink::Writer<::automsgs::msgs::map_msgs::OccupancyGrid>> measured_coverage_writer_;  ///< Actual Session target-ledger projection, never a map classifier output.
    std::shared_ptr<autolink::Writer<::automsgs::msgs::diagnostic_msgs::DiagnosticArray>> coverage_metadata_writer_;  ///< Existing native metadata message binding coverage output to Session, epoch and source cuts.
};

NativeRgbdPlatform::NativeRgbdPlatform(std::shared_ptr<ExplorationTask> task,std::shared_ptr<autolink::Node> node,NativeRgbdPlatformOptions options)
    : impl_(std::make_shared<Impl>(std::move(task),std::move(node),std::move(options))) {}
NativeRgbdPlatform::~NativeRgbdPlatform() { Shutdown(); }
bool NativeRgbdPlatform::Install() {
    const auto state = impl_; if (state->closed_.load() || state->retirement_) return false;
    state->retirement_ = std::make_shared<perception::ReaderRetirement>([state] { return state->Retire(); });
    NativeExplorationPlatform platform; platform.clock = Now;
    platform.localization = [cache = state->localization_](auto sensor,const std::string& frame,double stamp) { return cache->Lookup(sensor,frame,stamp,Now()); };
    platform.prepare_mapping = [state](const std::string& name) {
        const auto task = state->task_.lock(); const auto identity = task ? task->SensorInputIdentity() : std::nullopt;
        return !state->closed_.load() && identity && state->mapper_->PrepareGeneration(identity->generation,name);
    };
    platform.create_mapping_services = [state](const auto&,const auto&) {
        const auto task = state->task_.lock(); const auto identity = task ? task->SensorInputIdentity() : std::nullopt;
        if (!identity || !identity->generation) throw std::logic_error("native mapping factory has no actual accepted task generation");
        auto services = state->mapper_->Services(identity->generation);
        state->AttachStartupServices(&services,state);
        return services;
    };
    if (state->planner_) {
        const auto plugin = std::dynamic_pointer_cast<
            planning::planner::vista::VistaPlanner>(
                state->planner_->GetPlanner(
                    state->options_.exploration.routes.planner_id));
        if (plugin) platform.route_planner = plugin->SharedRoutePlanner();
    }
    platform.process_sensor_packet = [state](std::uint64_t generation,const perception::ObservationPacket& packet,double now) {
        if (state->closed_.load()) return std::optional<MapReadView>{};
        auto mapped = state->mapper_->Process(generation,packet,now);
        if (mapped) {
            state->AttachStartupSupport(*mapped);
            state->UpdateControlMap(*mapped);
            const auto occupancy = state->mapper_->Occupancy(); const auto grid = state->mapper_->Grid();
            if (occupancy && state->occupancy_writer_) state->occupancy_writer_->Write(*occupancy);
            if (grid && state->grid_writer_) state->grid_writer_->Write(*grid);
            const auto task = state->task_.lock();
            const auto snapshot = task ? task->GetExplorationSnapshot() : std::nullopt;
            if (snapshot && snapshot->session_id && snapshot->measured_coverage_map && state->measured_coverage_writer_ &&
                state->coverage_metadata_writer_) {
                ::automsgs::msgs::diagnostic_msgs::DiagnosticArray metadata;
                *metadata.mutable_header() = snapshot->measured_coverage_map->header();
                auto* status = metadata.add_status(); status->set_name("exploration/measured_coverage");
                status->set_hardware_id(std::to_string(snapshot->session_id));
                status->set_message("Actual target-ledger quality percentages; not occupancy or whole-room completion");
                const auto value = [status](const char* key,const std::string& text) {
                    auto* item = status->add_values(); item->set_key(key); item->set_value(text);
                };
                value("schema","autonomy.exploration.measured_coverage.v1");
                value("session_id",std::to_string(snapshot->session_id)); value("input_generation",std::to_string(generation));
                value("map_epoch",std::to_string(snapshot->measured_coverage_map_epoch));
                value("geometry_revision",std::to_string(snapshot->measured_coverage_geometry_revision));
                value("coverage_revision",std::to_string(snapshot->measured_coverage_revision));
                value("value_semantics","-1:no_requirement;0:unobserved_requirement;100:all_cell_requirements_confirmed");
                value("source_mask",std::to_string(snapshot->valid_observations.present));
                value("valid_lidar_ordinal",std::to_string(snapshot->valid_observations.sequence[0]));
                value("valid_rgbd_ordinal",std::to_string(snapshot->valid_observations.sequence[1]));
                value("completion_confirmed",snapshot->completion_confirmed ? "true" : "false");
                state->measured_coverage_writer_->Write(*snapshot->measured_coverage_map);
                state->coverage_metadata_writer_->Write(metadata);
            }
            if (snapshot && snapshot->session_id && snapshot->session_id != state->reported_session_) {
                state->reported_session_ = snapshot->session_id;
                AINFO << "NATIVE_EXPLORATION_SESSION session_id=" << snapshot->session_id << " generation=" << generation;
                std::size_t free = 0, occupied = 0;
                for (int x = 0; x < mapped->grid->width(); ++x) for (int y = 0; y < mapped->grid->height(); ++y) {
                    const auto cell = mapped->grid->At(x,y).state;
                    free += cell == autonomy::navigation::CellState::kFree;
                    occupied += cell == autonomy::navigation::CellState::kOccupied;
                }
                const autonomy::navigation::Pose2d pose{packet.localization.world_from_base.translation.x,
                    packet.localization.world_from_base.translation.y,0};
                const auto current = mapped->grid->WorldToCell(pose);
                const auto clearance = autonomy::navigation::GeometrySafety(mapped->grid,state->options_.exploration.coverage.safety).VerifyPose(pose);
                AINFO << "NATIVE_EXPLORATION_DOMAIN free=" << free << " occupied=" << occupied
                      << " unknown=" << mapped->grid->size()-free-occupied << " start_free="
                      << (current && mapped->grid->At(current->x,current->y).state == autonomy::navigation::CellState::kFree)
                      << " start_safe=" << clearance.safe << " reason=" << clearance.reason;
            }
            if (snapshot && snapshot->observation_losses.pending_sources &&
                snapshot->observation_losses.pending_sources != state->reported_losses_) {
                state->reported_losses_ = snapshot->observation_losses.pending_sources;
                const auto input = task->GetObservationSourceSnapshot().source;
                AINFO << "NATIVE_EXPLORATION_INPUT_LOSS mask=" << static_cast<unsigned>(state->reported_losses_)
                      << " rgbd_received=" << input.received[1] << " rgbd_accepted=" << input.accepted[1]
                      << " rgbd_rejected=" << input.rejected[1] << " invalid_message=" << input.rejected_by_reason[1][3]
                      << " invalid_localization=" << input.rejected_by_reason[1][4] << " invalid_evidence=" << input.rejected_by_reason[1][5]
                      << " association_timeouts=" << input.localization_timeouts[1];
            }
        }
        return mapped;
    };
    const auto task = state->task_.lock();
    if (!task) return false;
    try {
        state->odometry_writer_ = state->node_->CreateWriter<::automsgs::msgs::nav_msgs::Odometry>(state->options_.odometry_output_topic);
        state->occupancy_writer_ = state->node_->CreateWriter<::automsgs::msgs::map_msgs::OccupancyGrid>(state->options_.occupancy_output_topic);
        state->grid_writer_ = state->node_->CreateWriter<::automsgs::msgs::map_msgs::GridMap>(state->options_.grid_output_topic);
        state->measured_coverage_writer_ = state->node_->CreateWriter<::automsgs::msgs::map_msgs::OccupancyGrid>(state->options_.measured_coverage_topic);
        state->coverage_metadata_writer_ = state->node_->CreateWriter<::automsgs::msgs::diagnostic_msgs::DiagnosticArray>(state->options_.measured_coverage_topic+"/metadata");
        const std::weak_ptr<Impl> weak = state;
        state->odometry_reader_ = state->node_->CreateReader<::automsgs::msgs::nav_msgs::Odometry>(state->options_.localization.odometry_topic,[weak](const auto& message) {
            if (auto self = weak.lock(); self && message && !self->closed_.load()) if (self->localization_->PushOdometry(*message)) self->PoseReady();
        });
        state->status_reader_ = state->node_->CreateReader<::automsgs::msgs::diagnostic_msgs::DiagnosticArray>(state->options_.status_topic,[weak](const auto& message) {
            if (auto self = weak.lock(); self && message && !self->closed_.load()) if (self->localization_->PushStatus(*message)) self->PoseReady();
        });
        if (!state->odometry_reader_ || !state->status_reader_ || !state->odometry_writer_ || !state->occupancy_writer_ || !state->grid_writer_ ||
            !state->measured_coverage_writer_ || !state->coverage_metadata_writer_)
            throw std::runtime_error("actual native RGB-D reader/writer creation failed");
        if (state->options_.controller) {
            state->command_writer_ = state->node_->CreateWriter<::automsgs::msgs::geometry_msgs::TwistStamped>(state->options_.command_topic);
            state->control_image_writer_ = state->node_->CreateWriter<::automsgs::msgs::sensor_msgs::Image>(state->options_.controller_image_topic);
            if (!state->command_writer_ || !state->control_image_writer_) throw std::runtime_error("actual native execution transport creation failed");
            auto alias = state->options_.localization.base_from_camera;
            alias.mutable_header()->set_frame_id(state->options_.navigation_base_frame); alias.set_child_frame_id(state->options_.controller_camera_frame);
            ::autonomy::transform::ApplyTransformStampedToBuffer(state->buffer_,alias,"native_rgbd_actual_camera_installation",true);
            control::NativeControllerIO io;
            io.read_odometry = [weak] { const auto self = weak.lock(); return self ? self->ReadControlState() : std::nullopt; };
            io.publish_command = [weak](const auto& command) { const auto self = weak.lock(); return self && self->PublishControlCommand(command); };
            // Policy inference can delay the next feedback sample while the
            // command gate still requires fresh acquisition evidence. Accept
            // at most the configured feedback window (ControllerServer caps it
            // at one second); missing samples never replay a prior velocity.
            io.maximum_state_age = std::min(1.0, state->options_.command_gate.maximum_feedback_age);
            state->controller_ = std::make_unique<control::ControllerServer>(*state->options_.controller,state->planner_->GetCostmapWrapper(),std::move(io));
            state->controller_->Start();
            state->control_image_reader_ = state->node_->CreateReader<::automsgs::msgs::sensor_msgs::Image>(state->options_.exploration.input.depth_topic,[weak](const auto& image) {
                if (const auto self = weak.lock(); self && image && !self->closed_.load()) {
                    auto relayed = *image; relayed.mutable_header()->set_frame_id(self->options_.controller_camera_frame);
                    // The embedded controller shares this process and must see
                    // the actual relay even if transport discovery is late.
                    if (self->controller_) self->controller_->PublishImageToControllers(relayed);
                    self->control_image_writer_->Write(relayed);
                }
            });
            if (!state->control_image_reader_) throw std::runtime_error("actual native depth relay creation failed");
        }
        // Publish all immutable providers before the task can create its native
        // source/processor; startup callbacks cannot race resource assignment.
        if (!ConfigureNativeExploration(task,state->task_node_,state->options_.exploration,std::move(platform))) {
            Shutdown(); return false;
        }
        return true;
    } catch (const std::exception& error) {
        AERROR << "Native RGB-D provider/controller installation failed: " << error.what();
        Shutdown(); return false;
    } catch (...) { AERROR << "Native RGB-D provider/controller installation failed with an unknown exception"; Shutdown(); return false; }
}
void NativeRgbdPlatform::Shutdown() {
    impl_->StopStartupObservation();
    impl_->closed_.store(true); if (impl_->retirement_) impl_->retirement_->Request();
}
bool NativeRgbdPlatform::WaitForShutdown(std::chrono::milliseconds timeout) const {
    if (autolink::croutine::CRoutine::GetCurrentRoutine()) return false;
    return impl_->retirement_ && impl_->retirement_->WaitFor(timeout);
}
}  // namespace autonomy::task::exploration
#endif  // AUTONOMY_TASK_HAS_NATIVE_ATLAS_MAPPING
