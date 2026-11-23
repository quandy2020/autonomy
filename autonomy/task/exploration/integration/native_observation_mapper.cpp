/* Copyright 2026 The Openbot Authors. */
/** @file native_observation_mapper.cpp
 * @brief Actual existing-map fusion, measured-height read views and finite-prefix persistence.
 */
#include "autonomy/task/exploration/integration/native_observation_mapper.hpp"

#ifdef AUTONOMY_TASK_HAS_NATIVE_ATLAS_MAPPING
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fcntl.h>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <stdexcept>
#include <unistd.h>

#include "atlas/map/ground_classifier.hpp"
#include "atlas/map/occupancy_grid_map.hpp"
#include "autonomy/map/esdf/esdf_map.hpp"
#include "autonomy/map/esdf/raycast.hpp"
#include "autonomy/map/grid_map/grid_map_core/grid_map.hpp"
#include "autonomy/map/grid_map/grid_map_msgs/grid_map_converter.hpp"

namespace autonomy::task::exploration {
namespace {
namespace a = ::autonomy::localization::atlas::map;
namespace n = ::autonomy::navigation;
/** @brief Native metadata clock encoding; source ordinal, rather than this double, identifies fusion input. */
void Stamp(double time, ::automsgs::msgs::builtin_interfaces::Time* stamp) {
    const auto seconds = static_cast<std::int64_t>(std::floor(time));
    auto nanos = static_cast<std::int64_t>(std::llround((time - seconds) * 1e9));
    const auto whole = seconds + nanos / 1000000000LL;
    nanos %= 1000000000LL;
    if (whole < 0 || whole > std::numeric_limits<std::int32_t>::max())
        throw std::out_of_range("native mapping stamp exceeds automsgs time range");
    stamp->set_sec(static_cast<std::int32_t>(whole)); stamp->set_nanosec(static_cast<std::uint32_t>(nanos));
}
/** @brief Persist actual generated messages to a fresh temporary file and atomically replace the named map. */
bool Write(const std::filesystem::path& path, const std::string& payload, std::uint64_t request) {
    const auto temporary = path.string() + ".tmp-" + std::to_string(::getpid()) + "-" + std::to_string(request);
    const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
    if (fd < 0) return false;
    bool ok = true; std::size_t written = 0;
    while (written < payload.size()) {
        const auto count = ::write(fd, payload.data() + written, payload.size() - written);
        if (count <= 0) { ok = false; break; }
        written += static_cast<std::size_t>(count);
    }
    if (ok) ok = ::fsync(fd) == 0;
    if (::close(fd) != 0) ok = false;
    if (ok) ok = ::rename(temporary.c_str(), path.c_str()) == 0;
    if (!ok) ::unlink(temporary.c_str());
    return ok;
}
}  // namespace

class NativeObservationMapper::Impl {
public:
    /** @brief Validate scope before allocating actual ESDF and grid_map storage. */
    Impl(NativeObservationMapperOptions options, perception::ObservationIngress::Clock clock)
        : options_(std::move(options)), clock_(std::move(clock)), reader_(options_.maximum_voxels) {
        const auto& b = options_.region;
        if (!clock_ || options_.frame_id.empty() ||
            (options_.mode != perception::SensorMode::kRgbdOnly && options_.mode != perception::SensorMode::kLidarOnly &&
             options_.mode != perception::SensorMode::kLidarRgbd) ||
            !std::isfinite(b.min_x) || !std::isfinite(b.min_y) || !std::isfinite(b.max_x) || !std::isfinite(b.max_y) ||
            b.min_x >= b.max_x || b.min_y >= b.max_y || !std::isfinite(options_.resolution) || options_.resolution <= 0 ||
            !std::isfinite(options_.minimum_height) || !std::isfinite(options_.maximum_height) ||
            options_.minimum_height >= options_.maximum_height || !std::isfinite(options_.robot_height) || options_.robot_height <= 0 ||
            !std::isfinite(options_.vertical_margin) || options_.vertical_margin < 0 ||
            !std::isfinite(options_.ground_tolerance) || options_.ground_tolerance <= 0 ||
            !std::isfinite(options_.floor_span_tolerance) ||
            options_.floor_span_tolerance < options_.ground_tolerance ||
            options_.floor_span_tolerance > .25 ||
            !options_.maximum_voxels || !options_.maximum_rays || options_.maximum_rays > 1000000 ||
            !options_.maximum_pending_ordinals || options_.output_directory.empty())
            throw std::invalid_argument("native mapper requires explicit finite scope, heights, budgets and persistence directory");
        for (unsigned i = 0; i < 2; ++i) if (perception::Mask(options_.mode) & (1U << i)) {
            if (!std::isfinite(options_.sensor_heights[i]) || options_.sensor_heights[i] <= 0)
                throw std::invalid_argument("native mapper requires actual configured sensor heights");
            a::OccupancyGridOptions grid;
            grid.resolution = options_.resolution; grid.maximum_rays_per_frame = options_.maximum_rays;
            grid.maximum_cells = options_.maximum_voxels; grid.maximum_snapshot_cells = options_.maximum_voxels;
            grid.ray_angle_bins = std::min<std::size_t>(360, options_.maximum_rays);
            grid.sensor_height = options_.sensor_heights[i]; grid.use_sensor_height = true; grid.estimate_ground_plane = true;
            grid.ground_distance_threshold = options_.ground_tolerance;
            grid.minimum_obstacle_height = options_.ground_tolerance;
            grid.maximum_obstacle_height = options_.robot_height + options_.vertical_margin;
            classifiers_[i] = std::make_unique<a::GroundClassifier>(grid);
            if (!occupancy_) occupancy_ = std::make_unique<a::OccupancyGridMap>(grid);
        }
        const Eigen::Vector3d dimensions{std::ceil((b.max_x-b.min_x)/options_.resolution),
            std::ceil((b.max_y-b.min_y)/options_.resolution),
            std::ceil((options_.maximum_height-options_.minimum_height)/options_.resolution)};
        if (!dimensions.allFinite() || dimensions.minCoeff() <= 0 || dimensions.maxCoeff() > std::numeric_limits<int>::max())
            throw std::invalid_argument("native mapping dimensions cannot be represented safely");
        sizes_ = dimensions.cast<int>();
        const auto count = static_cast<std::uint64_t>(sizes_.x()) * sizes_.y() * sizes_.z();
        if (sizes_.minCoeff() <= 0 || count > options_.maximum_voxels || count > std::numeric_limits<int>::max())
            throw std::invalid_argument("native mapping voxel allocation exceeds its explicit work bound");
        ::autonomy::map::esdf::Options esdf;
        esdf.origin = {b.min_x, b.min_y, options_.minimum_height}; esdf.resolution = options_.resolution;
        esdf.map_size = sizes_.cast<double>() * options_.resolution;
        esdf.min_ray_length = 0; esdf.occupancy_probability = .5;
        esdf_ = std::make_unique<::autonomy::map::esdf::EsdfMap>(esdf);
        seen_.resize(count, false);
        layers_ = std::make_unique<::grid_map::GridMap>(std::vector<std::string>{"occupancy", "elevation", "floor_min", "ceiling", "navigation_state"});
        layers_->setGeometry(::grid_map::Length(sizes_.x()*options_.resolution, sizes_.y()*options_.resolution),
            options_.resolution, ::grid_map::Position(b.min_x+sizes_.x()*options_.resolution/2,
                                                     b.min_y+sizes_.y()*options_.resolution/2));
        layers_->setFrameId(options_.frame_id);
        std::filesystem::create_directories(options_.output_directory);
    }
    /** @brief Actual completed source ordinals; only a hole-free prefix becomes a receipt. */
    struct Ledger {
        perception::ObservationCut cut;  ///< Completed contiguous prefixes for this task generation.
        std::array<std::set<std::uint64_t>, 2> completed;  ///< Bounded completed suffixes awaiting an earlier actual input.
    };
    /** @brief Each factory owns its own reply queues; cancellation cannot consume another generation's result. */
    struct Operations {
        std::mutex mutex;  ///< Serializes independent versioned operation queues.
        std::uint64_t generation = 0;  ///< Captured native task generation, never generation-zero bootstrap.
        std::uint64_t session = 0;  ///< Bound on the first actual versioned request; another Session cannot reuse this factory.
        std::set<std::uint64_t> retired;  ///< Bounded canceled operation ids, preventing delayed resubmission.
        std::deque<MappingReply> refresh, save;  ///< Actual completed operations awaiting owner polling.
    };
    /** @brief Bound and index existing ESDF voxels without reading its unobserved-free default. */
    bool InMap(const Eigen::Vector3i& voxel) const { return (voxel.array() >= 0).all() && (voxel.array() < sizes_.array()).all(); }
    /** @brief A real coordinate change clears old geometry and every old-epoch fusion prefix. */
    void Reset(std::uint64_t epoch) {
        occupancy_->Reset();
        esdf_ = std::make_unique<::autonomy::map::esdf::EsdfMap>(esdf_->options());
        std::fill(seen_.begin(),seen_.end(),false);
        for (const char* layer : {"occupancy","elevation","floor_min","ceiling","navigation_state"}) layers_->get(layer).setConstant(NAN);
        ledgers_.clear(); latest_.reset(); epoch_ = epoch; stamp_ = 0;
    }
    /** @brief Convert a bounded voxel into this receiver's measured-voxel mask index. */
    std::size_t Index(const Eigen::Vector3i& v) const {
        return (static_cast<std::size_t>(v.x()) * sizes_.y() + v.y()) * sizes_.z() + v.z();
    }
    /** @brief Record a real ray using the existing map raycaster and occupancy updates. */
    void Ray(const perception::ObservationRay& ray, bool ground) {
        const auto& origin = esdf_->options().origin;
        const double res = options_.resolution;
        const Eigen::Vector3d start(ray.origin.x, ray.origin.y, ray.origin.z);
        const auto endpoint = ray.Endpoint();
        const Eigen::Vector3d end(endpoint.x, endpoint.y, endpoint.z);
        std::vector<Eigen::Vector3d> traversed;
        ::autonomy::map::esdf::Raycast((start-origin)/res, (end-origin)/res,
            Eigen::Vector3d::Zero(), sizes_.cast<double>(), &traversed);
        const auto last = ((end-origin)/res).array().floor().cast<int>().matrix().eval();
        for (const auto& point : traversed) {
            const auto voxel = point.cast<int>().eval();
            if (!InMap(voxel) || (ray.hit && !ground && voxel == last)) continue;
            esdf_->SetOccupancy(voxel, false); seen_[Index(voxel)] = true;
        }
        if (InMap(last)) { esdf_->SetOccupancy(last, ray.hit && !ground); seen_[Index(last)] = true; }
    }
    /** @brief Raise a ceiling bound only through actually observed free voxels above measured ground. */
    double Ceiling(const ::grid_map::Position& position, double floor) const {
        const auto& origin = esdf_->options().origin;
        int z = static_cast<int>(std::floor((floor-origin.z())/options_.resolution));
        const int x = static_cast<int>(std::floor((position.x()-origin.x())/options_.resolution));
        const int y = static_cast<int>(std::floor((position.y()-origin.y())/options_.resolution));
        double bound = floor;
        for (; z < sizes_.z(); ++z) {
            const Eigen::Vector3i voxel{x,y,z};
            if (!InMap(voxel) || !seen_[Index(voxel)] || esdf_->GetOccupancy(voxel) != 0) break;
            bound = origin.z() + (z+1)*options_.resolution;
        }
        return bound;
    }
    /** @brief Build an immutable conservative route model from actual grid_map layers. */
    MapReadView Snapshot(std::uint64_t generation) {
        auto grid = std::make_shared<n::GridMap>(sizes_.x(), sizes_.y(), options_.resolution,
            options_.region.min_x, options_.region.min_y, options_.frame_id);
        for (int x = 0; x < sizes_.x(); ++x) for (int y = 0; y < sizes_.y(); ++y) {
            const auto center = grid->CellCenter(x,y);
            ::grid_map::Index index;
            const ::grid_map::Position position{center.x,center.y};
            if (!layers_->getIndex(position,index)) continue;
            double probability = 0;
            const bool known = occupancy_->ProbabilityAt(center.x,center.y,&probability);
            // Ground classification and navigation-floor admission have different
            // contracts. The classifier stays conservative; a cell may aggregate
            // several floor returns whose min/max differ by sensor quantisation or
            // a small ramp. Use the measured lower surface as the floor and let the
            // footprint floor-step check reject discontinuities between cells.
            const double low = layers_->at("floor_min",index);
            const double measured_high = layers_->at("elevation",index);
            n::GridCell cell;
            cell.floor_height = low; cell.ceiling_height = std::numeric_limits<double>::quiet_NaN();
            if (known && probability >= .5) cell.state = n::CellState::kOccupied;
            else if (known && std::isfinite(low) && std::isfinite(measured_high) &&
                     measured_high >= low &&
                     measured_high-low <= options_.floor_span_tolerance) {
                cell.ceiling_height = Ceiling(position,low);
                if (cell.ceiling_height-low >= options_.robot_height+options_.vertical_margin)
                    cell.state = n::CellState::kFree;
            }
            layers_->at("ceiling",index) = static_cast<float>(cell.ceiling_height);
            layers_->at("occupancy",index) = !known ? NAN : static_cast<float>(probability);
            layers_->at("navigation_state",index) = cell.state == n::CellState::kFree ? 0.f :
                cell.state == n::CellState::kOccupied ? 100.f : -1.f;
            grid->Set(x,y,cell);
        }
        auto view = reader_.Read(*grid,stamp_,epoch_,0,false,perception::UnknownCellPolicy::kReplace);
        // MapReader owns change detection; the actual acquisition localization
        // owner supplies the externally shared coordinate identity.
        view.map_epoch = epoch_; view.has_height_evidence = true;
        if (generation && ledgers_.count(generation)) {
            view.observation_cut = ledgers_.at(generation).cut; view.mapping_synchronized = true;
        }
        latest_ = view;
        occupancy_message_.Clear();
        occupancy_message_.mutable_header()->set_frame_id(options_.frame_id);
        Stamp(stamp_,occupancy_message_.mutable_header()->mutable_stamp());
        auto* info = occupancy_message_.mutable_info(); info->set_resolution(options_.resolution);
        info->set_width(sizes_.x()); info->set_height(sizes_.y());
        info->mutable_origin()->mutable_position()->set_x(options_.region.min_x);
        info->mutable_origin()->mutable_position()->set_y(options_.region.min_y);
        info->mutable_origin()->mutable_orientation()->set_w(1);
        for (int y = 0; y < sizes_.y(); ++y) for (int x = 0; x < sizes_.x(); ++x) {
            double probability = 0;
            occupancy_message_.add_data(occupancy_->ProbabilityAt(grid->CellCenter(x,y).x,grid->CellCenter(x,y).y,&probability) ?
                probability >= .5 ? 100 : 0 : -1);
        }
        layers_->setTimestamp(static_cast<::grid_map::Time>(std::llround(stamp_*1e9)));
        ::grid_map::GridMapConverter::toMessage(*layers_,grid_message_);
        return view;
    }
    /** @brief Refresh receipt for the exact operation and the actually completed finite source prefix. */
    MappingReply Receipt(const RequestVersion& version, std::uint64_t generation) const {
        MappingReply reply; reply.version = version;
        reply.success = latest_ && version.map_epoch == epoch_ && version.geometry_revision <= latest_->geometry_revision &&
            std::isfinite(version.valid_until) && clock_() <= version.valid_until;
        if (ledgers_.count(generation)) reply.fused_cut = ledgers_.at(generation).cut;
        reply.fusion_confirmed = reply.success && reply.fused_cut.Covers(version.observation_cut) &&
            !version.observation_baseline.present && !version.observation_recovery_generation && !version.recovery_loss;
        return reply;
    }
    NativeObservationMapperOptions options_;  ///< Immutable scoped geometry, source installation and work budgets.
    perception::ObservationIngress::Clock clock_;  ///< Actual shared sensor/owner clock for operation deadlines.
    mutable std::mutex mutex_;  ///< Serializes actual mapping and immutable publication.
    std::array<std::unique_ptr<a::GroundClassifier>,2> classifiers_;  ///< Existing actual-source ground classification.
    std::unique_ptr<a::OccupancyGridMap> occupancy_;  ///< Existing sparse log-odds navigation fusion, unchanged source.
    std::unique_ptr<::autonomy::map::esdf::EsdfMap> esdf_;  ///< Existing native occupancy/distance implementation.
    std::unique_ptr<::grid_map::GridMap> layers_;  ///< Existing grid_map manages measured geometry layers.
    Eigen::Vector3i sizes_{0,0,0};  ///< Bounded voxel dimensions.
    std::vector<bool> seen_;  ///< Actual ray support; ESDF's unknown occupancy is never treated as observed free space.
    std::map<std::uint64_t,Ledger> ledgers_;  ///< Actual completed source ordinals for a bounded set of task generations.
    std::uint64_t epoch_ = 0;  ///< Actual acquisition-localization coordinate identity.
    double stamp_ = 0;  ///< Latest actual processed acquisition time.
    perception::MapReader reader_;  ///< Existing immutable geometry change detector.
    std::optional<MapReadView> latest_;  ///< Immutable geometry after actual map operations.
    ::automsgs::msgs::map_msgs::OccupancyGrid occupancy_message_;  ///< Actual uninflated 2D mapper output, not a coverage ledger.
    ::automsgs::msgs::map_msgs::GridMap grid_message_;  ///< Actual native grid_map layers for geometry persistence.
};

NativeObservationMapper::NativeObservationMapper(NativeObservationMapperOptions options,
    perception::ObservationIngress::Clock clock) : impl_(std::make_shared<Impl>(std::move(options),std::move(clock))) {}
NativeObservationMapper::~NativeObservationMapper() = default;

std::optional<MapReadView> NativeObservationMapper::Process(std::uint64_t generation,
    const perception::ObservationPacket& packet, double now) {
    if (!packet.Valid(now) || packet.localization.world_frame != impl_->options_.frame_id ||
        !(perception::Mask(impl_->options_.mode) & perception::Mask(packet.observation.sensor)) ||
        packet.observation.rays.size() > impl_->options_.maximum_rays) return {};
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (impl_->epoch_ && packet.localization.map_epoch < impl_->epoch_) return {};
    if (impl_->epoch_ && packet.localization.map_epoch != impl_->epoch_) impl_->Reset(packet.localization.map_epoch);
    impl_->epoch_ = packet.localization.map_epoch;
    auto& ledger = impl_->ledgers_[generation];
    const unsigned source = packet.observation.sensor == perception::SensorKind::kLidar ? 0 : 1;
    const auto ordinal = packet.observation.sequence;
    if ((generation && ordinal <= ledger.cut.sequence[source]) || ledger.completed[source].count(ordinal))
        return impl_->Snapshot(generation);
    if (ledger.completed[source].size() >= impl_->options_.maximum_pending_ordinals) return {};
    using Key = std::tuple<double,double,double>;
    std::map<Key,std::vector<const perception::ObservationRay*>> groups;
    for (const auto& ray : packet.observation.rays) groups[{ray.origin.x,ray.origin.y,ray.origin.z}].push_back(&ray);
    for (const auto& group : groups) {
        const Eigen::Vector3f origin{static_cast<float>(std::get<0>(group.first)),static_cast<float>(std::get<1>(group.first)),
                                    static_cast<float>(std::get<2>(group.first))};
        std::vector<a::ColorPoint,Eigen::aligned_allocator<a::ColorPoint>> points;
        std::vector<const perception::ObservationRay*> hits;
        for (const auto* ray : group.second) if (ray->hit) {
            const auto end = ray->Endpoint(); a::ColorPoint point;
            point.position = {static_cast<float>(end.x),static_cast<float>(end.y),static_cast<float>(end.z)};
            points.push_back(point); hits.push_back(ray);
        }
        a::SemanticPointCloud classified;
        if (!points.empty() && !impl_->classifiers_[source]->Classify(origin,points,&classified)) return {};
        if (classified.size() != hits.size()) return {};
        impl_->occupancy_->Integrate(origin,classified);
        for (std::size_t i = 0; i < hits.size(); ++i) {
            const bool ground = classified[i].semantic == a::PointSemantic::kGround;
            impl_->Ray(*hits[i],ground);
            if (ground) {
                const auto end = hits[i]->Endpoint(); ::grid_map::Index index;
                if (impl_->layers_->getIndex(::grid_map::Position(end.x,end.y),index)) {
                    auto& high = impl_->layers_->at("elevation",index); auto& low = impl_->layers_->at("floor_min",index);
                    high = std::isfinite(high) ? std::max(high,static_cast<float>(end.z)) : static_cast<float>(end.z);
                    low = std::isfinite(low) ? std::min(low,static_cast<float>(end.z)) : static_cast<float>(end.z);
                }
            }
        }
        for (const auto* ray : group.second) if (!ray->hit) impl_->Ray(*ray,false);
    }
    if (impl_->esdf_->UpdateOccupancy(true)) impl_->esdf_->UpdateESDF();
    impl_->stamp_ = std::max(impl_->stamp_,packet.observation.stamp);
    ledger.completed[source].insert(ordinal);
    if (generation) {
        while (ledger.completed[source].erase(ledger.cut.sequence[source]+1)) ++ledger.cut.sequence[source];
        if (ledger.cut.sequence[source]) ledger.cut.present |= perception::Mask(packet.observation.sensor);
    } else while (ledger.completed[source].size() > 128) ledger.completed[source].erase(ledger.completed[source].begin());
    while (impl_->ledgers_.size() > 8) impl_->ledgers_.erase(impl_->ledgers_.begin());
    return impl_->Snapshot(generation);
}

ExplorationServices NativeObservationMapper::Services(std::uint64_t generation) {
    if (!generation) throw std::invalid_argument("bootstrap generation cannot own mapping receipts");
    const auto state = impl_; auto operations = std::make_shared<Impl::Operations>(); operations->generation = generation;
    ExplorationServices services;
    services.request_map_refresh = [state,operations](const RequestVersion& version) {
        if (!version.session_id || !version.request_id) return false;
        std::lock_guard<std::mutex> lock(state->mutex_); std::lock_guard<std::mutex> queue(operations->mutex);
        if (operations->refresh.size() >= 16) return false;
        if (operations->session && operations->session != version.session_id) return false;
        operations->session = version.session_id;
        if (operations->retired.count(version.request_id)) return false;
        operations->refresh.push_back(state->Receipt(version,operations->generation)); return true;
    };
    const auto poll = [operations](bool save) -> std::optional<MappingReply> {
        std::lock_guard<std::mutex> lock(operations->mutex); auto& queue = save ? operations->save : operations->refresh;
        if (queue.empty()) return {}; auto reply = std::move(queue.front()); queue.pop_front(); return reply;
    };
    services.poll_map_refresh = [poll] { return poll(false); }; services.poll_map_save = [poll] { return poll(true); };
    services.save_map = [state,operations](const std::string& name,const RequestVersion& version) {
        if (name.empty() || name.size() > 128 || name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos ||
            !version.session_id || !version.request_id) return false;
        std::lock_guard<std::mutex> lock(state->mutex_); std::lock_guard<std::mutex> queue(operations->mutex);
        if (operations->save.size() >= 16) return false;
        if (operations->session && operations->session != version.session_id) return false;
        operations->session = version.session_id;
        if (operations->retired.count(version.request_id)) return false;
        auto reply = state->Receipt(version,operations->generation);
        if (reply.success && reply.fusion_confirmed) {
            std::string occupancy,grid;
            reply.success = state->occupancy_message_.SerializeToString(&occupancy) && state->grid_message_.SerializeToString(&grid) &&
                Write(state->options_.output_directory/(name+".occupancy.pb"),occupancy,version.request_id) &&
                Write(state->options_.output_directory/(name+".grid.pb"),grid,version.request_id);
            const int fd = ::open(state->options_.output_directory.c_str(),O_RDONLY | O_DIRECTORY);
            if (fd < 0 || ::fsync(fd) != 0) reply.success = false;
            if (fd >= 0) ::close(fd);
            if (state->clock_() > version.valid_until) reply.success = false;
        } else reply.success = false;
        operations->save.push_back(std::move(reply)); return true;
    };
    services.cancel_mapping = [operations](const RequestVersion& version) {
        std::lock_guard<std::mutex> lock(operations->mutex);
        if (!version.session_id || !version.request_id ||
            (operations->session && operations->session != version.session_id)) return;
        operations->session = version.session_id;
        if (operations->retired.count(version.request_id)) return;
        if (operations->retired.size() >= 128) throw std::runtime_error("native mapping cancellation history exceeded its bound");
        operations->retired.insert(version.request_id);
        for (auto* queue : {&operations->refresh,&operations->save})
            queue->erase(std::remove_if(queue->begin(),queue->end(),[&](const MappingReply& reply) {
                return reply.version.SameRequest(version); }),queue->end());
    };
    return services;
}
bool NativeObservationMapper::PrepareGeneration(std::uint64_t generation,const std::string& map_name) {
    if (!generation || map_name.empty() || map_name.size() > 128 ||
        map_name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") != std::string::npos) return false;
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (!impl_->esdf_ || !impl_->occupancy_ || !impl_->layers_ || !std::filesystem::is_directory(impl_->options_.output_directory)) return false;
    impl_->ledgers_.try_emplace(generation);
    while (impl_->ledgers_.size() > 8) impl_->ledgers_.erase(impl_->ledgers_.begin());
    return true;
}
std::optional<MapReadView> NativeObservationMapper::LatestView() const {
    std::lock_guard<std::mutex> lock(impl_->mutex_); return impl_->latest_;
}
std::optional<::automsgs::msgs::map_msgs::OccupancyGrid> NativeObservationMapper::Occupancy() const {
    std::lock_guard<std::mutex> lock(impl_->mutex_); if (!impl_->latest_) return {}; return impl_->occupancy_message_;
}
std::optional<::automsgs::msgs::map_msgs::GridMap> NativeObservationMapper::Grid() const {
    std::lock_guard<std::mutex> lock(impl_->mutex_); if (!impl_->latest_) return {}; return impl_->grid_message_;
}
}  // namespace autonomy::task::exploration
#endif  // AUTONOMY_TASK_HAS_NATIVE_ATLAS_MAPPING
