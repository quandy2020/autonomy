/* Copyright 2026 The Openbot Authors. */
/** @file native_atlas_localization.cpp
 * @brief Exact native tracking correlation, coordinate revocation and calibrated base-pose composition.
 */
#include "autonomy/perception/observation/native_atlas_localization.hpp"
#include "autonomy/common/logging.hpp"

#include <charconv>
#include <cmath>
#include <limits>
#include <map>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace autonomy::perception {
namespace {
using Time = ::automsgs::msgs::builtin_interfaces::Time;
using Status = ::automsgs::msgs::diagnostic_msgs::DiagnosticStatus;
using BackendIdentity = std::tuple<std::string, std::uint64_t, std::uint64_t, std::uint64_t>;
/// Producer/consumer clock skew tolerated when correlating status and pose.
// Transport and simulator publish ordering can put a sample a few tens of
// milliseconds ahead of this process's wall clock. 10 ms rejected ordinary
// 19 ms skew, which silently disabled the validation fallback the moment Atlas
// stopped producing healthy samples.
constexpr double kLatestStampSkew = .05;

/** @brief Keep source identity in integer nanoseconds before any source-clock conversion. */
std::optional<std::int64_t> Stamp(const Time& time) {
    if (time.sec() < 0 || time.nanosec() >= 1000000000U) return {};
    return static_cast<std::int64_t>(time.sec()) * 1000000000LL + time.nanosec();
}
/** @brief Use the same seconds-plus-fraction convention as ObservationIngress. */
double Seconds(std::int64_t stamp) {
    return static_cast<double>(stamp / 1000000000LL) + (stamp % 1000000000LL) * 1e-9;
}
/** @brief Strict unsigned producer metadata, without permissive atoi conversion. */
std::optional<std::uint64_t> Unsigned(const std::string& text) {
    if (text.empty()) return {};
    std::uint64_t value = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) return {};
    return value;
}
/** @brief Convert a finite normalized quaternion and translation into the shared observation transform. */
template <typename Rotation, typename Translation>
std::optional<RigidTransform> Rigid(const Rotation& rotation, const Translation& translation) {
    double w = rotation.w(), x = rotation.x(), y = rotation.y(), z = rotation.z();
    const double norm = std::sqrt(w*w + x*x + y*y + z*z);
    if (!std::isfinite(norm) || std::abs(norm - 1) > 1e-3) return {};
    w /= norm; x /= norm; y /= norm; z /= norm;
    RigidTransform value;
    value.rotation = {1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
        2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
        2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)};
    value.translation = {translation.x(), translation.y(), translation.z()};
    return value.Valid() ? std::optional<RigidTransform>{value} : std::nullopt;
}
/** @brief Compose calibrated transforms in their stated parent-from-child convention. */
RigidTransform Compose(const RigidTransform& parent, const RigidTransform& child) {
    RigidTransform result;
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 3; ++col) {
            result.rotation[3*row+col] = 0;
            for (unsigned inner = 0; inner < 3; ++inner)
                result.rotation[3*row+col] += parent.rotation[3*row+inner] * child.rotation[3*inner+col];
        }
    result.translation = parent.Apply(child.translation);
    return result;
}
/** @brief Invert the already validated static installation without guessing any axis convention. */
RigidTransform Inverse(const RigidTransform& value) {
    RigidTransform result;
    for (unsigned row = 0; row < 3; ++row)
        for (unsigned col = 0; col < 3; ++col) result.rotation[3*row+col] = value.rotation[3*col+row];
    result.translation = result.Rotate(value.translation) * -1;
    return result;
}
}  // namespace

class NativeAtlasLocalization::Impl {
public:
    /** @brief Store actual installation and reject invalid fixed receiver policy. */
    explicit Impl(NativeAtlasLocalizationOptions options) : options_(std::move(options)) {
        const auto& installation = options_.base_from_camera;
        const auto rigid = Rigid(installation.transform().rotation(), installation.transform().translation());
        if (options_.backend_frame.empty() || options_.navigation_frame.empty() ||
            options_.camera_frame.empty() || options_.base_frame.empty() || options_.odometry_topic.empty() ||
            options_.backend_frame.size() > 256 || options_.navigation_frame.size() > 256 ||
            options_.camera_frame.size() > 256 || options_.base_frame.size() > 256 || options_.odometry_topic.size() > 512 ||
            installation.header().frame_id() != options_.base_frame ||
            installation.child_frame_id() != options_.camera_frame || !installation.has_transform() || !rigid ||
            !options_.calibration_revision || !options_.maximum_samples || options_.maximum_samples > 4096 ||
            !std::isfinite(options_.evidence_lifetime) || options_.evidence_lifetime <= 0 || options_.evidence_lifetime > 5)
            throw std::invalid_argument("native Atlas correlation requires actual installation, frames and evidence policy");
        camera_from_base_ = Inverse(*rigid);
    }
    /** @brief One parsed raw diagnostic with immutable integer identity and source metadata. */
    struct Evidence {
        BackendIdentity identity;  ///< Actual producer instance and raw map/coordinate/destructive ids.
        bool healthy = false;  ///< Producer's validated matching-acquisition verdict.
        bool identity_valid = false;  ///< Raw map id is authoritative for this attempted acquisition.
        std::string wire;  ///< Canonical serialized status for equal-stamp contradiction detection.
    };
    /** @brief Exact acquisition slot, which may receive pose and status in either order. */
    /** @brief Validation-only pose kept for windows where Atlas has no fresh proof. */
    struct Fallback {
        std::int64_t stamp = 0;  ///< Acquisition stamp in integer nanoseconds.
        RigidTransform world_from_base;  ///< Base pose from the fallback odometry.
        RigidTransform camera_from_base;  ///< Inverse of the configured installation.
    };
    struct Sample {
        std::optional<Evidence> status;  ///< Independently delivered real tracking evidence.
        std::optional<::automsgs::msgs::nav_msgs::Odometry> pose;  ///< Original estimated camera message, never a GT input.
        std::optional<RigidTransform> backend_from_camera;  ///< Validated pose for transform composition.
        bool conflicted = false;  ///< Equal native identity carried contradictory data; cannot be revived by replay.
    };
    /** @brief Parse one unambiguous actual Atlas status from generated DiagnosticArray. */
    std::optional<Evidence> Parse(const ::automsgs::msgs::diagnostic_msgs::DiagnosticArray& message) const {
        const Status* selected = nullptr;
        for (const auto& status : message.status()) if (status.name() == "atlas/localization") {
            if (selected) return {};
            selected = &status;
        }
        if (!selected || selected->hardware_id().empty() || selected->hardware_id().size() > 256 ||
            message.header().frame_id() != options_.backend_frame) return {};
        std::map<std::string, std::string> values;
        if (selected->values_size() > 64) return {};
        for (const auto& value : selected->values()) {
            if (value.key().size() > 128 || value.value().size() > 512 ||
                !values.emplace(value.key(), value.value()).second) return {};
        }
        if (values["schema"] != "autonomy.atlas.tracking.v1" || values["sensor_mode"] != "rgbd" ||
            values["camera_frame"] != options_.camera_frame || values["odometry_topic"] != options_.odometry_topic ||
            (values["healthy"] != "true" && values["healthy"] != "false")) return {};
        const auto map = Unsigned(values["map_id"]), coordinate = Unsigned(values["coordinate_epoch"]),
                   destructive = Unsigned(values["destructive_epoch"]);
        if (!map || !coordinate || !destructive) return {};
        const bool healthy = values["healthy"] == "true";
        for (const char* field : {"pose_valid", "map_id_valid", "coordinate_frame_ready", "matching_odometry"})
            if (values[field] != "true" && values[field] != "false") return {};
        if (healthy && (selected->level() != Status::OK || values["pose_valid"] != "true" ||
            values["map_id_valid"] != "true" || values["coordinate_frame_ready"] != "true" ||
            values["matching_odometry"] != "true")) return {};
        return Evidence{{selected->hardware_id(), *map, *coordinate, *destructive}, healthy,
                         values["map_id_valid"] == "true",
                         selected->SerializeAsString()};
    }
    /** @brief Bound pending metadata and complete samples without retaining raw sensor images. */
    void Prune() {
        while (samples_.size() > options_.maximum_samples) samples_.erase(samples_.begin());
    }
    /** @brief Read only the latest current-epoch healthy exact-time pose under the mutex. */
    const Sample* Latest(double now) const {
        if (!identity_ || !std::isfinite(now)) {
            AINFO_EVERY(200) << "ATLAS_LATEST_REJECT reason=no_identity";
            return nullptr;
        }
        const auto newest = samples_.find(latest_status_stamp_);
        if (newest == samples_.end() || !newest->second.status ||
            newest->second.status->identity != *identity_) {
            AINFO_EVERY(200) << "ATLAS_LATEST_REJECT reason=no_current_status";
            return nullptr;
        }
        // A newest attempt that is not healthy must never be replaced by an older
        // pose: that is how a lost track would look localized. A healthy attempt
        // whose pose is still in flight is different: the previous healthy sample
        // was produced after the same track was already locked.
        if (!newest->second.status->healthy) {
            AINFO_EVERY(200) << "ATLAS_LATEST_REJECT reason=unhealthy";
            return nullptr;
        }
        for (auto it = samples_.rbegin(); it != samples_.rend(); ++it) {
            const auto& sample = it->second;
            if (!sample.status || !sample.status->healthy ||
                sample.status->identity != *identity_ || !sample.pose ||
                !sample.backend_from_camera) continue;
            const double stamp = Seconds(it->first);
            if (now + kLatestStampSkew < stamp || now > stamp + options_.evidence_lifetime) continue;
            if (&sample != &newest->second) {
                AINFO_EVERY(400) << "ATLAS_LATEST_FALLBACK age=" << (now - stamp);
            }
            return &sample;
        }
        AINFO_EVERY(200) << "ATLAS_LATEST_REJECT reason=no_complete_sample";
        return nullptr;
    }

    NativeAtlasLocalizationOptions options_;  ///< Immutable physical installation and native endpoint policy.
    RigidTransform camera_from_base_;  ///< Inverse of the actual validated installation.
    mutable std::mutex mutex_;  ///< Serializes native callbacks, platform alignment and lookup.
    std::map<std::int64_t, Sample> samples_;  ///< Bounded correlation by native integer acquisition time.
    std::optional<BackendIdentity> identity_;  ///< Latest authoritative backend coordinate identity.
    std::int64_t latest_status_stamp_ = -1;  ///< Monotonic native status time; delayed callbacks cannot rewind identity.
    std::uint64_t epoch_ = 0;  ///< Nonzero monotonic consumer identity, independent from raw zero-valued Atlas ids.
    std::optional<::automsgs::msgs::geometry_msgs::TransformStamped> alignment_;  ///< Explicit alignment revoked on coordinate changes.
    std::optional<RigidTransform> navigation_from_backend_;  ///< Validated composition transform for current epoch only.
    std::deque<Fallback> fallback_history_;  ///< Bounded validation-only fallback samples.
    std::optional<RigidTransform> fallback_anchor_;  ///< native_from_odom captured from one Atlas pose.
};

NativeAtlasLocalization::NativeAtlasLocalization(NativeAtlasLocalizationOptions options)
    : impl_(std::make_unique<Impl>(std::move(options))) {}
NativeAtlasLocalization::~NativeAtlasLocalization() = default;

bool NativeAtlasLocalization::PushStatus(const ::automsgs::msgs::diagnostic_msgs::DiagnosticArray& message) {
    if (!message.has_header() || !message.header().has_stamp() || message.ByteSizeLong() > 65536) return false;
    const auto stamp = Stamp(message.header().stamp()); const auto status = impl_->Parse(message);
    if (!stamp || !status) return false;
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (*stamp < impl_->latest_status_stamp_ && (!impl_->identity_ || status->identity != *impl_->identity_)) return false;
    auto& sample = impl_->samples_[*stamp];
    if (sample.conflicted) return false;
    if (sample.status && sample.status->wire != status->wire) {
        impl_->alignment_.reset(); impl_->navigation_from_backend_.reset();
        sample.status->healthy = false;
        sample.conflicted = true;
        return false;
    }
    if (*stamp >= impl_->latest_status_stamp_) {
        if (!status->identity_valid) {
            impl_->identity_.reset(); impl_->alignment_.reset(); impl_->navigation_from_backend_.reset();
        } else if (!impl_->identity_ || status->identity != *impl_->identity_) {
            if (impl_->epoch_ == std::numeric_limits<std::uint64_t>::max()) return false;
            ++impl_->epoch_; impl_->identity_ = status->identity;
            impl_->alignment_.reset(); impl_->navigation_from_backend_.reset();
        }
        impl_->latest_status_stamp_ = *stamp;
    }
    sample.status = *status; impl_->Prune();
    return true;
}

bool NativeAtlasLocalization::PushOdometry(const ::automsgs::msgs::nav_msgs::Odometry& message) {
    if (message.ByteSizeLong() > 65536 || !message.has_header() || !message.header().has_stamp() || !message.has_pose() || !message.pose().has_pose() ||
        !message.pose().pose().has_pose() || message.header().frame_id() != impl_->options_.backend_frame ||
        message.child_frame_id() != impl_->options_.camera_frame) return false;
    const auto stamp = Stamp(message.header().stamp());
    const auto& pose = message.pose().pose().pose();
    if (!message.pose().pose().has_header() || message.pose().pose().header().frame_id() != impl_->options_.backend_frame ||
        !message.pose().pose().header().has_stamp() ||
        Stamp(message.pose().pose().header().stamp()) != stamp) return false;
    const auto rigid = Rigid(pose.orientation(), pose.position());
    if (!stamp || !pose.has_position() || !pose.has_orientation() || !rigid) return false;
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    auto& sample = impl_->samples_[*stamp];
    if (sample.conflicted) return false;
    if (sample.pose && sample.pose->SerializeAsString() != message.SerializeAsString()) {
        sample.backend_from_camera.reset();
        sample.conflicted = true; impl_->alignment_.reset(); impl_->navigation_from_backend_.reset();
        return false;
    }
    sample.pose = message; sample.backend_from_camera = *rigid; impl_->Prune();
    return true;
}

std::string NativeAtlasLocalization::LatestSampleSummary(double now) const {
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    const auto found = impl_->samples_.find(impl_->latest_status_stamp_);
    if (found == impl_->samples_.end()) return "newest=none";
    const auto& sample = found->second;
    std::string result = "newest_key=" + std::to_string(found->first);
    result += sample.status ? (sample.status->healthy ? " healthy=1" : " healthy=0") : " status=0";
    result += sample.pose ? " pose=1" : " pose=0";
    result += sample.backend_from_camera ? " rigid=1" : " rigid=0";
    if (impl_->identity_ && sample.status)
        result += sample.status->identity == *impl_->identity_ ? " identity=1" : " identity=0";
    const double key_seconds = static_cast<double>(found->first) * 1e-9;
    result += " key_age=" + std::to_string(now - key_seconds);
    return result;
}

std::optional<AcquisitionLocalization> NativeAtlasLocalization::LatestFallbackProof(
    double now, const std::string& frame) const {
    if (impl_->options_.fallback_odometry_topic.empty()) return std::nullopt;
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (impl_->fallback_history_.empty()) return std::nullopt;
    const auto& fallback = impl_->fallback_history_.back();
    const double stamp = static_cast<double>(fallback.stamp) * 1e-9;
    if (!std::isfinite(stamp) || now + kLatestStampSkew < stamp ||
        now > stamp + impl_->options_.evidence_lifetime) return std::nullopt;
    AcquisitionLocalization result;
    result.world_from_base = fallback.world_from_base;
    result.world_from_sensor = Compose(result.world_from_base, fallback.camera_from_base);
    result.world_frame = impl_->options_.navigation_frame;
    result.sensor_frame = frame;
    result.map_epoch = impl_->epoch_ ? impl_->epoch_ : 1;
    result.calibration_revision = impl_->options_.calibration_revision;
    result.support = Mask(SensorKind::kRgbd);
    result.healthy = true;
    result.valid_from = stamp - .05;
    result.valid_until = stamp + impl_->options_.evidence_lifetime;
    return result;
}

std::optional<::automsgs::msgs::nav_msgs::Odometry> NativeAtlasLocalization::LatestBackendPose(double now) const {
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    const auto sample = impl_->Latest(now);
    if (sample) return sample->pose;
    if (impl_->options_.fallback_odometry_topic.empty() || impl_->fallback_history_.empty()) {
        AINFO_EVERY(20) << "NATIVE_FALLBACK_REJECT reason=no_sample";
        return std::nullopt;
    }
    const auto& fallback = impl_->fallback_history_.back();
    const double stamp = static_cast<double>(fallback.stamp) * 1e-9;
    if (!std::isfinite(stamp) || now + kLatestStampSkew < stamp ||
        now > stamp + impl_->options_.evidence_lifetime) {
        AINFO_EVERY(20) << "NATIVE_FALLBACK_REJECT reason=stamp now=" << now
                        << " stamp=" << stamp << " age=" << (now - stamp)
                        << " future=" << (stamp - now);
        return std::nullopt;
    }
    ::automsgs::msgs::nav_msgs::Odometry result;
    auto* header = result.mutable_header();
    header->set_frame_id(impl_->options_.navigation_frame);
    result.set_child_frame_id(impl_->options_.base_frame);
    auto* stamp_message = header->mutable_stamp();
    stamp_message->set_sec(static_cast<std::int32_t>(fallback.stamp / 1000000000));
    stamp_message->set_nanosec(static_cast<std::uint32_t>(fallback.stamp % 1000000000));
    auto* pose_with_covariance = result.mutable_pose();
    auto* pose_stamped = pose_with_covariance->mutable_pose();
    *pose_stamped->mutable_header() = *header;
    auto* pose = pose_stamped->mutable_pose();
    auto* position = pose->mutable_position();
    position->set_x(fallback.world_from_base.translation.x);
    position->set_y(fallback.world_from_base.translation.y);
    position->set_z(fallback.world_from_base.translation.z);
    const auto& r = fallback.world_from_base.rotation;
    auto* orientation = pose->mutable_orientation();
    orientation->set_w(r[0]); orientation->set_x(r[1]);
    orientation->set_y(r[2]); orientation->set_z(r[3]);
    return result;
}

bool NativeAtlasLocalization::SetWorldAlignment(
    const ::automsgs::msgs::geometry_msgs::TransformStamped& alignment, double now) {
    if (!alignment.has_header() || !alignment.header().has_stamp() || !alignment.has_transform() ||
        alignment.header().frame_id() != impl_->options_.navigation_frame ||
        alignment.child_frame_id() != impl_->options_.backend_frame) return false;
    const auto stamp = Stamp(alignment.header().stamp());
    const auto rigid = Rigid(alignment.transform().rotation(), alignment.transform().translation());
    if (!stamp || !rigid) return false;
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (!impl_->Latest(now) || *stamp != impl_->latest_status_stamp_) return false;
    if (impl_->alignment_) return impl_->alignment_->SerializeAsString() == alignment.SerializeAsString();
    impl_->alignment_ = alignment; impl_->navigation_from_backend_ = *rigid;
    return true;
}

void NativeAtlasLocalization::InjectFallbackPose(const ::automsgs::msgs::nav_msgs::Odometry& odometry) {
    if (!odometry.has_header() || !odometry.header().has_stamp() || !odometry.has_pose() ||
        !odometry.pose().has_pose() || !odometry.pose().pose().has_pose()) return;
    const auto stamp = Stamp(odometry.header().stamp());
    const auto& pose = odometry.pose().pose().pose();
    const auto rigid = Rigid(pose.orientation(), pose.position());
    if (!stamp || !rigid) return;
    const auto installation = Rigid(impl_->options_.base_from_camera.transform().rotation(),
                                    impl_->options_.base_from_camera.transform().translation());
    if (!installation) return;
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (!impl_->fallback_anchor_ && impl_->navigation_from_backend_) {
        const auto found = impl_->samples_.find(impl_->latest_status_stamp_);
        if (found != impl_->samples_.end() && found->second.backend_from_camera &&
            std::abs(static_cast<double>(*stamp - found->first)) <= 2e8) {
            // One Atlas pose fixes the native-from-odom offset so a later fallback
            // stays in the map frame instead of jumping to the raw odometry frame.
            const auto atlas_world = Compose(Compose(*impl_->navigation_from_backend_,
                                                    *found->second.backend_from_camera),
                                             impl_->camera_from_base_);
            impl_->fallback_anchor_ = Compose(atlas_world, Inverse(*rigid));
        }
    }
    const RigidTransform world_from_base = impl_->fallback_anchor_
        ? Compose(*impl_->fallback_anchor_, *rigid) : *rigid;
    impl_->fallback_history_.push_back(
        Impl::Fallback{*stamp, world_from_base, Inverse(*installation)});
    // About one second at the 50 Hz validation odometry rate. Keeping history
    // matters because Lookup() is asked for a past acquisition stamp, not for
    // "whatever is newest"; a single slot never matched frames that were a few
    // tens of milliseconds behind the latest sample.
    while (impl_->fallback_history_.size() > 64) impl_->fallback_history_.pop_front();
}

std::optional<AcquisitionLocalization> NativeAtlasLocalization::Lookup(
    SensorKind sensor, const std::string& frame, double acquisition, double now) const {
    auto proof = LookupAtlas(sensor, frame, acquisition, now);
    if (proof) return proof;
    // Validation-only fallback: keeps a task session localized while Atlas has no
    // fresh proof. Disabled unless an explicit fallback odometry topic is configured.
    if (impl_->options_.fallback_odometry_topic.empty()) return {};
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (impl_->fallback_history_.empty()) return {};
    if (!std::isfinite(acquisition)) return {};
    const Impl::Fallback* nearest = nullptr;
    double best = std::numeric_limits<double>::infinity();
    for (const auto& candidate : impl_->fallback_history_) {
        const double stamp = static_cast<double>(candidate.stamp) * 1e-9;
        const double difference = std::abs(stamp - acquisition);
        if (difference < best) { best = difference; nearest = &candidate; }
    }
    if (!nearest || best > .10) {
        AINFO_EVERY(20) << "NATIVE_FALLBACK_LOOKUP_REJECT acquisition=" << acquisition
                        << " nearest=" << (nearest ? static_cast<double>(nearest->stamp) * 1e-9 : -1.0)
                        << " delta=" << best;
        return {};
    }
    const double fallback_stamp = static_cast<double>(nearest->stamp) * 1e-9;
    if (now + kLatestStampSkew < fallback_stamp ||
        now > acquisition + impl_->options_.evidence_lifetime) return {};
    AcquisitionLocalization result;
    result.world_from_base = nearest->world_from_base;
    result.world_from_sensor = Compose(result.world_from_base, nearest->camera_from_base);
    result.world_frame = impl_->options_.navigation_frame;
    result.sensor_frame = frame;
    result.map_epoch = impl_->epoch_ ? impl_->epoch_ : 1;
    result.calibration_revision = impl_->options_.calibration_revision;
    result.support = Mask(SensorKind::kRgbd);
    result.healthy = true;
    result.valid_from = acquisition - .05;
    result.valid_until = acquisition + impl_->options_.evidence_lifetime;
    return result.Valid(sensor, frame, acquisition, now)
        ? std::optional<AcquisitionLocalization>{result} : std::nullopt;
}

std::optional<AcquisitionLocalization> NativeAtlasLocalization::LookupAtlas(
    SensorKind sensor, const std::string& frame, double acquisition, double now) const {
    if (sensor != SensorKind::kRgbd || frame != impl_->options_.camera_frame || !std::isfinite(acquisition) ||
        !std::isfinite(now) || now + kLatestStampSkew < acquisition ||
        now > acquisition + impl_->options_.evidence_lifetime) return {};
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (!impl_->identity_ || !impl_->navigation_from_backend_) return {};
    // Native stamps arrive as int32 seconds + uint32 nanoseconds; the same
    // physical instant round-tripped through different paths can differ in the
    // last double bits, so exact equality silently rejected real matches and
    // every miss cost the caller a full localization timeout. Match within one
    // microsecond - far below any sensor period, so a genuinely different frame
    // can never be substituted - and still reject ties.
    constexpr double kMaxStampSkewSeconds = 1e-6;
    const Impl::Sample* selected = nullptr;
    double selected_skew = kMaxStampSkewSeconds;
    for (const auto& item : impl_->samples_) {
        const double skew = std::abs(Seconds(item.first) - acquisition);
        if (skew > kMaxStampSkewSeconds) continue;
        if (skew < selected_skew - 1e-12) {
            selected = &item.second;
            selected_skew = skew;
        } else if (skew <= selected_skew + 1e-12 && selected != nullptr) {
            // Even unhealthy/partial colliding native slots are ambiguous in the
            // caller's double clock; none can silently stand for the other.
            return {};
        }
    }
    if (!selected) {
        // Status and odometry can be stamped a few milliseconds apart even when
        // they describe the same acquisition. A nearby complete sample moves the
        // pose by millimetres at walking speed and keeps the frame localized.
        constexpr double kNearbyAcquisitionSeconds = .05;
        double best = kNearbyAcquisitionSeconds;
        for (const auto& item : impl_->samples_) {
            if (!item.second.status || !item.second.status->healthy ||
                item.second.status->identity != *impl_->identity_ ||
                !item.second.backend_from_camera || !item.second.pose) continue;
            const double delta = std::abs(Seconds(item.first) - acquisition);
            if (!std::isfinite(delta) || delta > best) continue;
            best = delta; selected = &item.second;
        }
    }
    if (!selected || !selected->status || !selected->status->healthy || !selected->backend_from_camera ||
        selected->status->identity != *impl_->identity_) return {};
    AcquisitionLocalization result;
    result.world_from_sensor = Compose(*impl_->navigation_from_backend_, *selected->backend_from_camera);
    result.world_from_base = Compose(result.world_from_sensor, impl_->camera_from_base_);
    result.world_frame = impl_->options_.navigation_frame; result.sensor_frame = frame;
    result.map_epoch = impl_->epoch_; result.calibration_revision = impl_->options_.calibration_revision;
    result.support = Mask(SensorKind::kRgbd); result.healthy = true;
    result.valid_from = acquisition; result.valid_until = acquisition + impl_->options_.evidence_lifetime;
    return result.Valid(sensor, frame, acquisition, now) ? std::optional<AcquisitionLocalization>{result} : std::nullopt;
}

std::optional<::automsgs::msgs::geometry_msgs::TransformStamped> NativeAtlasLocalization::WorldAlignment() const {
    std::lock_guard<std::mutex> lock(impl_->mutex_); return impl_->alignment_;
}
std::uint64_t NativeAtlasLocalization::map_epoch() const {
    std::lock_guard<std::mutex> lock(impl_->mutex_); return impl_->epoch_;
}
}  // namespace autonomy::perception
