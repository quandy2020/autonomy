/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/teleop/path_viz.hpp"

#include <map>
#include <optional>
#include <vector>

#include <algorithm>
#include <cmath>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>

#include "autonomy/common/logging.hpp"
#include "autonomy/task/teleop/constants.hpp"
#include "autonomy/task/teleop/path_selector.hpp"
#include "autonomy/task/teleop/obstacle_grid.hpp"

namespace autonomy::task::teleop {
namespace {

using automsgs::msgs::visualization_msgs::Marker;
using automsgs::msgs::visualization_msgs::MarkerArray;

constexpr float kFreePathLineWidth = 0.018f;
constexpr float kGroupSpineLineWidth = 0.035f;
constexpr float kBestPathLineWidth = 0.07f;
constexpr double kVizCollisionSampleStep = 0.02;

/**
 * @brief Convert HSV color to RGB components
 */
void HsvToRgb(float h, float s, float v, float* r, float* g, float* b) {
    h = std::fmod(h, 1.f);
    if (h < 0.f) {
        h += 1.f;
    }
    const int sector = static_cast<int>(h * 6.f);
    const float f = h * 6.f - static_cast<float>(sector);
    const float p = v * (1.f - s);
    const float q = v * (1.f - f * s);
    const float t = v * (1.f - (1.f - f) * s);
    switch (sector % 6) {
        case 0:
            *r = v;
            *g = t;
            *b = p;
            break;
        case 1:
            *r = q;
            *g = v;
            *b = p;
            break;
        case 2:
            *r = p;
            *g = v;
            *b = t;
            break;
        case 3:
            *r = p;
            *g = q;
            *b = v;
            break;
        case 4:
            *r = t;
            *g = p;
            *b = v;
            break;
        default:
            *r = v;
            *g = p;
            *b = q;
            break;
    }
}

/**
 * @brief Set solid color on visualization marker
 */
void SetMarkerRgb(Marker* marker, float r, float g, float b, float a) {
    if (marker == nullptr) {
        return;
    }
    marker->clear_colors();
    marker->mutable_color()->set_r(r);
    marker->mutable_color()->set_g(g);
    marker->mutable_color()->set_b(b);
    marker->mutable_color()->set_a(a);
}

/**
 * @brief Count polar groups spanned by candidates and spines
 */
int CountPathGroups(const std::vector<PathCandidate>& candidates,
                    std::size_t group_spine_count) {
    int num_groups = static_cast<int>(group_spine_count);
    for (const auto& candidate : candidates) {
        num_groups = std::max(num_groups, candidate.group_id + 1);
    }
    return std::max(num_groups, 1);
}

/**
 * @brief Stamp frame_id on path header and poses
 */
automsgs::msgs::nav_msgs::Path WithFrame(
    const automsgs::msgs::nav_msgs::Path& path, const std::string& frame_id) {
    automsgs::msgs::nav_msgs::Path out = path;
    out.mutable_header()->set_frame_id(frame_id);
    *out.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
    for (auto& pose : *out.mutable_poses()) {
        pose.mutable_header()->set_frame_id(frame_id);
    }
    return out;
}

/**
 * @brief Short forward stub path for channel discovery
 */
automsgs::msgs::nav_msgs::Path MakeStubPath(const std::string& frame_id) {
    automsgs::msgs::nav_msgs::Path path;
    path.mutable_header()->set_frame_id(frame_id);
    *path.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
    for (double x : {0.0, 0.5}) {
        auto* pose = path.add_poses();
        pose->mutable_header()->set_frame_id(frame_id);
        pose->mutable_pose()->mutable_position()->set_x(x);
        pose->mutable_pose()->mutable_orientation()->set_w(1.0);
    }
    return path;
}

/**
 * @brief Order group ids from center outward for symmetric sampling
 */
std::vector<int> CenterOutOrder(
    const std::map<int, std::vector<int>>& by_group) {
    std::vector<int> group_ids;
    group_ids.reserve(by_group.size());
    for (const auto& [group_id, _] : by_group) {
        group_ids.push_back(group_id);
    }
    std::sort(group_ids.begin(), group_ids.end());

    std::vector<int> order;
    order.reserve(group_ids.size());
    const int mid = static_cast<int>(group_ids.size()) / 2;
    for (int offset = 0; offset <= mid; ++offset) {
        if (mid + offset < static_cast<int>(group_ids.size())) {
            order.push_back(group_ids[static_cast<std::size_t>(mid + offset)]);
        }
        if (offset > 0 && mid - offset >= 0) {
            order.push_back(group_ids[static_cast<std::size_t>(mid - offset)]);
        }
    }
    return order;
}

/**
 * @brief Subsample indices evenly across polar groups
 */
std::vector<int> SampleByGroup(
    const std::vector<int>& indices,
    const std::vector<PathCandidate>& candidates, int max_count) {
    if (static_cast<int>(indices.size()) <= max_count) {
        return indices;
    }

    std::map<int, std::vector<int>> by_group;
    for (int index : indices) {
        if (index < 0 || index >= static_cast<int>(candidates.size())) {
            continue;
        }
        by_group[candidates[static_cast<std::size_t>(index)].group_id].push_back(
            index);
    }

    const std::vector<int> order = CenterOutOrder(by_group);
    std::vector<int> sampled;
    sampled.reserve(static_cast<std::size_t>(max_count));
    bool added = true;
    while (added && static_cast<int>(sampled.size()) < max_count) {
        added = false;
        for (int group_id : order) {
            auto it = by_group.find(group_id);
            if (it == by_group.end() || it->second.empty()) {
                continue;
            }
            sampled.push_back(it->second.front());
            it->second.erase(it->second.begin());
            added = true;
            if (static_cast<int>(sampled.size()) >= max_count) {
                break;
            }
        }
    }
    return sampled;
}

/**
 * @brief Sample library indices center-out by group
 */
std::vector<int> SampleIndices(const IntentPathSelector& selector,
                                      int max_count) {
    const auto& candidates = selector.candidates();
    const int candidate_count = static_cast<int>(candidates.size());
    if (candidate_count == 0 || max_count <= 0) {
        return {};
    }

    std::map<int, std::vector<int>> by_group;
    for (int i = 0; i < candidate_count; ++i) {
        by_group[candidates[static_cast<std::size_t>(i)].group_id].push_back(i);
    }

    std::vector<int> sampled;
    sampled.reserve(static_cast<std::size_t>(max_count));
    bool added = true;
    const std::vector<int> order = CenterOutOrder(by_group);
    while (added && static_cast<int>(sampled.size()) < max_count) {
        added = false;
        for (int group_id : order) {
            auto it = by_group.find(group_id);
            if (it == by_group.end() || it->second.empty()) {
                continue;
            }
            sampled.push_back(it->second.front());
            it->second.erase(it->second.begin());
            added = true;
            if (static_cast<int>(sampled.size()) >= max_count) {
                break;
            }
        }
    }
    return sampled;
}

/**
 * @brief First N library indices for full fan display
 */
std::vector<int> AllFanIndices(
    const std::vector<PathCandidate>& candidates, int max_count) {
    std::vector<int> indices;
    const int n = std::min(static_cast<int>(candidates.size()), max_count);
    indices.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        indices.push_back(i);
    }
    return indices;
}

/**
 * @brief Color marker by polar group id (HSV fan)
 */
void SetGroupMarkerColor(Marker* marker, int group_id, int num_groups,
                         float alpha = 0.95f) {
    if (marker == nullptr) {
        return;
    }
    const int denom = std::max(1, num_groups - 1);
    const float t =
        static_cast<float>(std::clamp(group_id, 0, num_groups - 1)) /
        static_cast<float>(denom);
    // Cyan (right / -Y) -> green -> yellow -> orange (left / +Y).
    const float hue = (0.52f - 0.42f * t);
    float r = 0.f;
    float g = 0.f;
    float b = 0.f;
    HsvToRgb(hue, 0.92f, 0.98f, &r, &g, &b);
    SetMarkerRgb(marker, r, g, b, alpha);
}

/**
 * @brief Color marker by normalized hue parameter
 */
void SetHueMarkerColor(Marker* marker, float hue, float alpha = 0.92f) {
    if (marker == nullptr) {
        return;
    }
    const float h = 0.52f - 0.42f * std::clamp(hue, 0.f, 1.f);
    float r = 0.f;
    float g = 0.f;
    float b = 0.f;
    HsvToRgb(h, 0.9f, 0.96f, &r, &g, &b);
    SetMarkerRgb(marker, r, g, b, alpha);
}

/**
 * @brief Build LINE_STRIP marker from nav path
 */
Marker MakeLineStripMarker(const automsgs::msgs::nav_msgs::Path& path,
                           const std::string& frame_id, int marker_id,
                           float hue, bool is_best = false,
                           int group_id = -1, int num_groups = 1,
                           float line_width = kFreePathLineWidth) {
    Marker marker;
    *marker.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
    marker.mutable_header()->set_frame_id(frame_id);
    marker.set_ns("teleop_free_paths");
    marker.set_id(marker_id);
    marker.set_type(Marker::LINE_STRIP);
    marker.set_action(Marker::ADD);
    marker.mutable_scale()->set_x(is_best ? kBestPathLineWidth : line_width);
    marker.clear_colors();
    if (is_best) {
        SetMarkerRgb(&marker, 1.0f, 0.92f, 0.1f, 1.0f);
    } else if (group_id >= 0 && num_groups > 1) {
        SetGroupMarkerColor(&marker, group_id, num_groups);
    } else {
        SetHueMarkerColor(&marker, hue);
    }

    for (const auto& pose_stamped : path.poses()) {
        auto* point = marker.add_points();
        point->set_x(pose_stamped.pose().position().x());
        point->set_y(pose_stamped.pose().position().y());
        point->set_z(0.0);
    }
    return marker;
}

/**
 * @brief True when path has no lethal costmap hits
 */
bool PathClear(const PathObstacleGrid& grid,
                                  const automsgs::msgs::nav_msgs::Path& path,
                                  int /*lethal_threshold*/) {
    return !HasLethalHit(grid, path, kVizCollisionSampleStep);
}

/**
 * @brief Build DELETE marker for a marker id
 */
Marker MakeDeleteMarker(const std::string& frame_id, int marker_id) {
    Marker marker;
    *marker.mutable_header()->mutable_stamp() =
        automsgs::msgs::builtin_interfaces::TimeNow();
    marker.mutable_header()->set_frame_id(frame_id);
    marker.set_ns("teleop_free_paths");
    marker.set_id(marker_id);
    marker.set_action(Marker::DELETE);
    return marker;
}

/**
 * @brief Append DELETE markers for ids no longer published
 */
void AppendTrailingDeletes(MarkerArray* array, const std::string& frame_id,
                           int published_count, int* last_count) {
    if (array == nullptr || last_count == nullptr) {
        return;
    }
    for (int id = published_count; id < *last_count; ++id) {
        *array->add_markers() = MakeDeleteMarker(frame_id, id);
    }
    *last_count = published_count;
}

/**
 * @brief Build full free_path_markers array for library fan
 */
MarkerArray BuildMarkerArray(const IntentPathSelector& selector,
                                     const PathObstacleGrid* grid,
                                     const std::string& frame_id,
                                     int* last_count,
                                     std::optional<int> best_index = std::nullopt,
                                     const PathSelectionResult* selection = nullptr) {
    MarkerArray array;
    const auto& candidates = selector.candidates();
    const auto& lib_opts = selector.library_options();
    const bool plot_library_fan = lib_opts.free_paths_plot_library_fan;
    const bool plot_path_set =
        !plot_library_fan && selection != nullptr && lib_opts.plot_path_set &&
        !selection->free_path_indices.empty();
    const bool apply_rot =
        selection != nullptr && lib_opts.use_rot_dir_search &&
        !lib_opts.free_paths_in_base_link;
    const double rot_deg = apply_rot ? selection->best_rot_deg : 0.0;
    const bool has_speed_selection =
        selection != nullptr && selection->best_path_scale > 0.0 &&
        selection->scaled_range > 0.0;
    const double default_arc_length =
        lib_opts.path_range > 0.0 ? lib_opts.path_range
                                  : 3.0 * lib_opts.segment_length;
    const double path_scale =
        has_speed_selection ? selection->best_path_scale : 1.0;
    const double viz_arc_length =
        has_speed_selection ? selection->scaled_range : default_arc_length;
    const int num_groups = CountPathGroups(candidates,
                                           selector.group_start_paths().size());
    const int max_markers =
        lib_opts.free_paths_max_markers > 0 ? lib_opts.free_paths_max_markers
                                            : 343;
    const int lethal_threshold = lib_opts.point_per_path_thr > 0
                                     ? lib_opts.point_per_path_thr
                                     : 2;
    const bool filter_collisions =
        lib_opts.free_paths_filter_collisions && grid != nullptr;
    const bool clip_paths = grid != nullptr;

    std::vector<int> display_indices;
    if (plot_library_fan) {
        display_indices = AllFanIndices(candidates, max_markers);
    } else if (plot_path_set) {
        display_indices = selection->free_path_indices;
    } else {
        display_indices =
            SampleIndices(selector, std::min(max_markers, 140));
    }

    std::vector<int> free_indices;
    free_indices.reserve(display_indices.size());
    for (int index : display_indices) {
        if (index < 0 || index >= static_cast<int>(candidates.size())) {
            continue;
        }
        free_indices.push_back(index);
    }

    if (free_indices.empty()) {
        AppendTrailingDeletes(&array, frame_id, 0, last_count);
        return array;
    }

    if (!plot_library_fan &&
        static_cast<int>(free_indices.size()) > max_markers) {
        free_indices =
            SampleByGroup(free_indices, candidates, max_markers);
    }

    const int total = static_cast<int>(free_indices.size());
    int published_count = 0;

    // FALCO Fig.3: 7 group spines (startPaths), thicker + group hue.
    if (plot_library_fan) {
        const auto& group_spines = selector.group_start_paths();
        for (int g = 0; g < static_cast<int>(group_spines.size()); ++g) {
            automsgs::msgs::nav_msgs::Path path =
                group_spines[static_cast<std::size_t>(g)];
            if (viz_arc_length > 0.0) {
                path = IntentPathSelector::TrimPathToArcLength(path,
                                                               viz_arc_length);
            }
            if (apply_rot || path_scale != 1.0) {
                path = IntentPathSelector::RotateAndScalePath(path, rot_deg,
                                                              path_scale);
            }
            if (filter_collisions &&
                !PathClear(*grid, path, lethal_threshold)) {
                continue;
            }
            if (clip_paths) {
                path = ClipForViz(*grid, path, kVizCollisionSampleStep);
            }
            if (path.poses_size() < 2) {
                continue;
            }
            *array.add_markers() = MakeLineStripMarker(
                path, frame_id, published_count, 0.5f, false, g, num_groups,
                kGroupSpineLineWidth);
            ++published_count;
        }
    }

    for (int slot = 0; slot < total; ++slot) {
        const int index = free_indices[static_cast<std::size_t>(slot)];
        const int group_id =
            candidates[static_cast<std::size_t>(index)].group_id;
        const bool is_best =
            best_index.has_value() && *best_index == index;
        automsgs::msgs::nav_msgs::Path path;
        if (is_best && selection != nullptr &&
            selection->best_path.has_value() &&
            selection->best_path->poses_size() >= 2) {
            path = *selection->best_path;
        } else {
            path = candidates[static_cast<std::size_t>(index)].path;
            if (viz_arc_length > 0.0) {
                path = IntentPathSelector::TrimPathToArcLength(path,
                                                               viz_arc_length);
            }
            if (apply_rot || path_scale != 1.0) {
                path = IntentPathSelector::RotateAndScalePath(path, rot_deg,
                                                              path_scale);
            }
        }
        if (filter_collisions &&
            !PathClear(*grid, path, lethal_threshold)) {
            continue;
        }
        if (clip_paths) {
            path = ClipForViz(*grid, path, kVizCollisionSampleStep);
        } else if (!plot_path_set && grid != nullptr &&
                   !IsPathCollisionFree(*grid, path)) {
            continue;
        }
        const float hue =
            total <= 1 ? 0.5f
                       : static_cast<float>(slot) / static_cast<float>(total - 1);
        if (path.poses_size() < 2) {
            continue;
        }
        *array.add_markers() = MakeLineStripMarker(
            path, frame_id, published_count, hue, is_best, group_id, num_groups);
        ++published_count;
    }
    AppendTrailingDeletes(&array, frame_id, published_count, last_count);
    return array;
}

}  // namespace

/**
 * @brief Stop discovery thread on destruction
 */
TeleopPathVisualizer::~TeleopPathVisualizer() {
    StopDiscovery();
}

/**
 * @brief Create path and marker writers
 */
void TeleopPathVisualizer::Configure(const std::shared_ptr<autolink::Node>& node,
                                     const std::string& frame_id,
                                     bool enabled) {
    StopDiscovery();
    enabled_ = enabled;
    frame_id_ = frame_id;
    node_ = node;
    selected_path_writer_.reset();
    free_path_markers_writer_.reset();

    if (!enabled_ || !node_) {
        return;
    }

    selected_path_writer_ =
        node_->CreateWriter<automsgs::msgs::nav_msgs::Path>(kTeleopPathTopic);
    free_path_markers_writer_ =
        node_->CreateWriter<MarkerArray>(kTeleopFreePathMarkersTopic);
    if (!selected_path_writer_ || !free_path_markers_writer_) {
        AWARN << "TeleopPathVisualizer: failed to create writers on "
              << kTeleopPathTopic << " / " << kTeleopFreePathMarkersTopic;
        return;
    }
    AINFO << "TeleopPathVisualizer: writers on " << kTeleopPathTopic << " and "
          << kTeleopFreePathMarkersTopic;
}

/**
 * @brief Set fallback library for discovery thread
 */
void TeleopPathVisualizer::SetLibrarySource(
    const IntentPathSelector* selector) {
    preview_selector_ = selector;
}

/**
 * @brief Register live preview callback from assist
 */
void TeleopPathVisualizer::SetPreviewCallback(PreviewCallback callback) {
    preview_callback_ = std::move(callback);
}

/**
 * @brief Start background discovery republish thread
 */
void TeleopPathVisualizer::StartDiscovery(
    std::chrono::milliseconds period) {
    StopDiscovery();
    if (!enabled_ || period.count() <= 0) {
        return;
    }
    discovery_period_ = period;
    discovery_running_.store(true, std::memory_order_release);
    discovery_thread_ = std::thread([this]() {
        std::size_t publish_count = 0;
        while (discovery_running_.load(std::memory_order_acquire)) {
            if (preview_callback_) {
                preview_callback_();
            } else {
                const std::string frame =
                    frame_id_.empty() ? kDefaultBaseFrame : frame_id_;
                PublishStub(frame);
                if (preview_selector_ != nullptr) {
                    PublishLibrary(*preview_selector_, frame);
                }
            }
            ++publish_count;
            if (publish_count == 1 || publish_count % 10 == 0) {
                AINFO << "TeleopPathVisualizer: discovery publish #"
                      << publish_count;
            }
            const auto step = std::chrono::milliseconds(50);
            auto remaining = discovery_period_;
            while (remaining.count() > 0 &&
                   discovery_running_.load(std::memory_order_acquire)) {
                const auto slice = remaining > step ? step : remaining;
                std::this_thread::sleep_for(slice);
                remaining -= slice;
            }
        }
    });
}

/**
 * @brief Join discovery publisher thread
 */
void TeleopPathVisualizer::StopDiscovery() {
    discovery_running_.store(false, std::memory_order_release);
    if (discovery_thread_.joinable()) {
        discovery_thread_.join();
    }
}

/**
 * @brief Thread-safe publish of marker array
 */
void TeleopPathVisualizer::WriteMarkers(
    automsgs::msgs::visualization_msgs::MarkerArray array) {
    if (!enabled_ || !node_ || !free_path_markers_writer_) {
        return;
    }
    std::lock_guard<std::mutex> lock(publish_mutex_);
    free_path_markers_writer_->Write(std::move(array));
}

/**
 * @brief Publish stub path for late subscribers
 */
void TeleopPathVisualizer::PublishStub(const std::string& frame_id) {
    if (!enabled_ || !node_ || !selected_path_writer_) {
        return;
    }
    std::lock_guard<std::mutex> lock(publish_mutex_);
    selected_path_writer_->Write(MakeStubPath(frame_id));
}

/**
 * @brief Alias for discovery stub path publish
 */
void TeleopPathVisualizer::PublishPathStub(const std::string& frame_id) {
    PublishStub(frame_id);
}

/**
 * @brief Build and write free_path_markers
 */
void TeleopPathVisualizer::PublishMarkers(
    const IntentPathSelector& selector, const PathObstacleGrid* grid,
    const std::string& frame_id, std::optional<int> best_index,
    const PathSelectionResult* selection) {
    if (!enabled_ || !node_ || !free_path_markers_writer_) {
        return;
    }
    auto array = BuildMarkerArray(selector, grid, frame_id,
                                          &last_free_path_marker_count_,
                                          best_index, selection);
    WriteMarkers(std::move(array));
    AINFO_EVERY(10) << "TeleopPathVisualizer: free_path_markers count="
                    << last_free_path_marker_count_ << " frame=" << frame_id;
}

/**
 * @brief Publish unclipped library fan
 */
void TeleopPathVisualizer::PublishLibrary(
    const IntentPathSelector& selector, const std::string& frame_id) {
    PublishMarkers(selector, nullptr, frame_id);
}

/**
 * @brief Publish obstacle-clipped library fan
 */
void TeleopPathVisualizer::PublishClipped(
    const IntentPathSelector& selector, const PathObstacleGrid& grid,
    const std::string& frame_id, std::optional<int> best_index,
    const PathSelectionResult* selection) {
    PublishMarkers(selector, &grid, frame_id, best_index, selection);
}

/**
 * @brief Publish best follow path topic
 */
void TeleopPathVisualizer::PublishSelected(
    const PathSelectionResult& path_selection, const std::string& frame_id,
    const PathObstacleGrid* /*grid*/) {
    if (!enabled_ || !node_ || !selected_path_writer_) {
        return;
    }
    if (!path_selection.best_path.has_value() ||
        path_selection.best_path->poses_size() < 2) {
        return;
    }
    const automsgs::msgs::nav_msgs::Path path =
        WithFrame(*path_selection.best_path, frame_id);
    std::lock_guard<std::mutex> lock(publish_mutex_);
    selected_path_writer_->Write(path);
}

/**
 * @brief Publish markers and selected path together
 */
void TeleopPathVisualizer::Publish(const IntentPathSelector& selector,
                                   const PathSelectionResult& path_selection,
                                   const std::string& frame_id) {
    if (!enabled_ || !node_) {
        return;
    }
    const std::optional<int> best_index =
        path_selection.best_index >= 0
            ? std::optional<int>(path_selection.best_index)
            : std::nullopt;
    auto array = BuildMarkerArray(
        selector, nullptr, frame_id, &last_free_path_marker_count_, best_index,
        &path_selection);
    WriteMarkers(std::move(array));
    PublishSelected(path_selection, frame_id);
}

/**
 * @brief Clear path and DELETEALL markers
 */
void TeleopPathVisualizer::PublishEmpty(const std::string& frame_id) {
    if (!enabled_ || !node_) {
        return;
    }
    MarkerArray array;
    Marker delete_all;
    delete_all.mutable_header()->set_frame_id(frame_id);
    delete_all.set_ns("teleop_free_paths");
    delete_all.set_action(Marker::DELETEALL);
    *array.add_markers() = delete_all;
    last_free_path_marker_count_ = 0;
    WriteMarkers(std::move(array));
    if (selected_path_writer_) {
        automsgs::msgs::nav_msgs::Path empty;
        empty.mutable_header()->set_frame_id(frame_id);
        *empty.mutable_header()->mutable_stamp() =
            automsgs::msgs::builtin_interfaces::TimeNow();
        std::lock_guard<std::mutex> lock(publish_mutex_);
        selected_path_writer_->Write(empty);
    }
}

}  // namespace autonomy::task::teleop
