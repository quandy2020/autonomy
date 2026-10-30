#include "examples/dataset_reader/trajectory_segments.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace autonomy::localization::atlas::examples {
namespace {

struct TumFileSummary {
    bool valid = false;
    std::size_t poses = 0;
    double first_time = 0.0;
    double last_time = 0.0;
};

bool ValidPose(const TrackingResult& result) {
    return result.valid && std::isfinite(result.time) &&
           result.pose_world_camera.translation.allFinite() &&
           result.pose_world_camera.rotation.coeffs().allFinite() &&
           result.pose_world_camera.rotation.norm() > 1e-12;
}

std::string SegmentPath(const std::string& path, std::size_t index) {
    std::ostringstream result;
    result << path << ".segment-" << std::setw(3) << std::setfill('0')
           << index << ".tum";
    return result.str();
}

std::string Csv(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped.push_back('"');
    for (const char character : value) {
        if (character == '"')
            escaped.push_back('"');
        escaped.push_back(character);
    }
    escaped.push_back('"');
    return escaped;
}

TumFileSummary SummarizeTumFile(const std::string& path) {
    std::ifstream input(path);
    TumFileSummary summary;
    if (!input)
        return summary;

    std::string line;
    double previous_time = -std::numeric_limits<double>::infinity();
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#')
            continue;
        std::istringstream row(line);
        std::array<double, 8> values{};
        for (double& value : values) {
            if (!(row >> value) || !std::isfinite(value))
                return TumFileSummary{};
        }
        std::string extra;
        if (row >> extra || values[0] <= previous_time)
            return TumFileSummary{};
        if (summary.poses == 0)
            summary.first_time = values[0];
        previous_time = values[0];
        summary.last_time = values[0];
        ++summary.poses;
    }
    summary.valid = summary.poses > 0;
    return summary;
}

bool CopyFile(const std::string& source, const std::string& destination) {
    std::ifstream input(source, std::ios::binary);
    std::ofstream output(destination, std::ios::binary | std::ios::trunc);
    if (!input || !output)
        return false;
    output << input.rdbuf();
    return input.good() || input.eof() ? static_cast<bool>(output) : false;
}

}  // namespace

void TrajectorySegmentRecorder::Record(const TrackingResult& result) {
    if (result.map_id_valid) {
        if (observed_map_id_valid_ &&
            (result.map_id != observed_map_id_ ||
             result.coordinate_epoch != observed_coordinate_epoch_ ||
             result.destructive_epoch != observed_destructive_epoch_)) {
            break_before_next_valid_ = true;
        }
        observed_map_id_ = result.map_id;
        observed_coordinate_epoch_ = result.coordinate_epoch;
        observed_destructive_epoch_ = result.destructive_epoch;
        observed_map_id_valid_ = true;
    }
    if (!ValidPose(result))
        return;

    if (segments_.empty() || break_before_next_valid_ ||
        result.time <= segments_.back().poses.back().timestamp) {
        Segment segment;
        segment.map_id = observed_map_id_;
        segment.map_id_valid = observed_map_id_valid_;
        segment.coordinate_epoch = observed_coordinate_epoch_;
        segment.destructive_epoch = observed_destructive_epoch_;
        segments_.push_back(std::move(segment));
    } else if (result.map_id_valid &&
               segments_.back().map_id_valid &&
               (result.map_id != segments_.back().map_id ||
                result.coordinate_epoch !=
                    segments_.back().coordinate_epoch ||
                result.destructive_epoch !=
                    segments_.back().destructive_epoch)) {
        Segment segment;
        segment.map_id = result.map_id;
        segment.map_id_valid = true;
        segment.coordinate_epoch = result.coordinate_epoch;
        segment.destructive_epoch = result.destructive_epoch;
        segments_.push_back(std::move(segment));
    }
    if (segments_.back().poses.empty()) {
        segments_.back().map_id = observed_map_id_;
        segments_.back().map_id_valid = observed_map_id_valid_;
        segments_.back().coordinate_epoch = observed_coordinate_epoch_;
        segments_.back().destructive_epoch = observed_destructive_epoch_;
    }
    if (result.map_id_valid && !segments_.back().map_id_valid) {
        segments_.back().map_id = result.map_id;
        segments_.back().map_id_valid = true;
        segments_.back().coordinate_epoch = result.coordinate_epoch;
        segments_.back().destructive_epoch = result.destructive_epoch;
    }
    Eigen::Quaterniond rotation = result.pose_world_camera.rotation;
    rotation.normalize();
    TrajectorySample sample;
    sample.timestamp = result.time;
    sample.world_from_camera = autonomy::common::math::SE3f(
        rotation.cast<float>(),
        result.pose_world_camera.translation.cast<float>());
    segments_.back().poses.push_back(std::move(sample));
    ++valid_pose_count_;
    break_before_next_valid_ = false;
}

bool TrajectorySegmentRecorder::SaveTum(
    const std::string& path,
    const std::string& active_optimized_path) const {
    if (path.empty() || segments_.empty())
        return false;

    std::size_t longest_index = 0;
    for (std::size_t index = 1; index < segments_.size(); ++index) {
        if (segments_[index].poses.size() >
            segments_[longest_index].poses.size()) {
            longest_index = index;
        }
    }

    std::vector<std::string> segment_paths;
    segment_paths.reserve(segments_.size());
    for (std::size_t index = 0; index < segments_.size(); ++index) {
        segment_paths.push_back(SegmentPath(path, index));
        if (!TrajectoryIO::SaveTum(segment_paths.back(),
                                   segments_[index].poses)) {
            return false;
        }
    }

    const TumFileSummary optimized = SummarizeTumFile(active_optimized_path);
    const Segment& active_segment = segments_.back();
    std::size_t active_raw_poses = 0;
    for (const Segment& segment : segments_) {
        if (segment.map_id_valid == active_segment.map_id_valid &&
            (!segment.map_id_valid ||
             (segment.map_id == active_segment.map_id &&
              segment.destructive_epoch ==
                  active_segment.destructive_epoch))) {
            active_raw_poses += segment.poses.size();
        }
    }
    const bool optimized_covers_active =
        optimized.valid && optimized.poses >= active_raw_poses;
    const bool use_optimized =
        optimized_covers_active &&
        optimized.poses >= segments_[longest_index].poses.size();
    if (use_optimized) {
        if (!CopyFile(active_optimized_path, path))
            return false;
    } else if (!TrajectoryIO::SaveTum(path,
                                      segments_[longest_index].poses)) {
        return false;
    }

    std::ofstream manifest(path + ".segments.csv");
    if (!manifest)
        return false;
    manifest << "kind,index,path,poses,first_time_s,last_time_s,selected,"
                "replace_active,"
                "map_id,map_id_valid,coordinate_epoch,destructive_epoch,"
                "coordinate_frame_scope\n";
    manifest << std::setprecision(17);
    for (std::size_t index = 0; index < segments_.size(); ++index) {
        const Segment& segment = segments_[index];
        manifest << "raw," << index << ',' << Csv(segment_paths[index]) << ','
                 << segment.poses.size() << ','
                 << segment.poses.front().timestamp << ','
                 << segment.poses.back().timestamp << ','
                 << (!use_optimized && index == longest_index ? "true"
                                                               : "false")
                 << ",false"
                 << ',' << segment.map_id << ','
                 << (segment.map_id_valid ? "true" : "false")
                 << ',' << segment.coordinate_epoch << ','
                 << segment.destructive_epoch
                 << ",independent_atlas_map\n";
    }
    if (optimized.valid) {
        manifest << "optimized_active,-1," << Csv(active_optimized_path) << ','
                 << optimized.poses << ',' << optimized.first_time << ','
                 << optimized.last_time << ','
                 << (use_optimized ? "true" : "false")
                 << ',' << (optimized_covers_active ? "true" : "false")
                 << ',' << segments_.back().map_id << ','
                 << (segments_.back().map_id_valid ? "true" : "false")
                 << ',' << segments_.back().coordinate_epoch << ','
                 << segments_.back().destructive_epoch
                 << ",active_map_only\n";
    }
    return static_cast<bool>(manifest);
}

}  // namespace autonomy::localization::atlas::examples
