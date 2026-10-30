#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "atlas/common/logging.hpp"
#include "atlas/map_builder.hpp"
#include "examples/dataset_reader/kitti_reader.hpp"

namespace atlas = autonomy::localization::atlas;

namespace {

bool WriteDenseMapPly(const std::string& path,
                      const atlas::map::DenseMapSnapshot& dense_map) {
    static_assert(sizeof(float) == 4,
                  "binary PLY export requires 32-bit float");
    static_assert(std::numeric_limits<float>::is_iec559,
                  "binary PLY export requires IEEE-754 float");
    const std::uint16_t endian_marker = 1;
    if (path.empty() ||
        *reinterpret_cast<const std::uint8_t*>(&endian_marker) != 1) {
        return false;
    }
    for (const atlas::map::ColorPoint& point : dense_map.points) {
        if (!point.position.allFinite())
            return false;
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
        return false;
    output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "element vertex " << dense_map.points.size() << "\n"
           << "property float x\n"
           << "property float y\n"
           << "property float z\n"
           << "property uchar red\n"
           << "property uchar green\n"
           << "property uchar blue\n"
           << "end_header\n";
    for (const atlas::map::ColorPoint& point : dense_map.points) {
        const std::array<float, 3> position = {
            point.position.x(), point.position.y(), point.position.z()};
        const std::array<std::uint8_t, 3> color = {
            point.red, point.green, point.blue};
        output.write(reinterpret_cast<const char*>(position.data()),
                     static_cast<std::streamsize>(sizeof(position)));
        output.write(reinterpret_cast<const char*>(color.data()),
                     static_cast<std::streamsize>(sizeof(color)));
    }
    output.flush();
    return output.good();
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 4 || argc > 6) {
        AERROR << "usage: stereo_kitti VOCAB CONFIG SEQUENCE_ROOT "
                  "[TRAJECTORY [DENSE_PLY]]"
               << std::endl;
        return 2;
    }

    std::vector<atlas::examples::KittiImageSample> samples;
    if (!atlas::examples::KittiReader::LoadStereo(argv[3], &samples)) {
        return 1;
    }

    atlas::MapBuilder builder(atlas::SystemConfig{
        argv[1], argv[2], atlas::SensorMode::kStereo, false});
    if (!builder.Start()) {
        return 1;
    }

    std::size_t valid_frames = 0;
    double total_tracking_seconds = 0.0;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const auto& sample = samples[index];
        atlas::ImageFrame left{
            sample.timestamp, "image_0",
            cv::imread(sample.left_path, cv::IMREAD_GRAYSCALE)};
        atlas::ImageFrame right{
            sample.timestamp, "image_1",
            cv::imread(sample.right_path, cv::IMREAD_GRAYSCALE)};
        if (left.pixels.empty() || right.pixels.empty()) {
            AERROR << "failed to load KITTI frame " << sample.frame_index
                   << std::endl;
            return 1;
        }

        const auto begin = std::chrono::steady_clock::now();
        atlas::TrackingResult result;
        if (!builder.TrackStereo(left, right, &result)) {
            AWARN << "KITTI stereo frame rejected: " << sample.frame_index
                  << std::endl;
        } else if (result.valid) {
            ++valid_frames;
        }
        const double tracking_seconds =
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          begin)
                .count();
        total_tracking_seconds += tracking_seconds;

        double frame_interval = 0.0;
        if (index + 1 < samples.size()) {
            frame_interval = samples[index + 1].timestamp - sample.timestamp;
        } else if (index > 0) {
            frame_interval = sample.timestamp - samples[index - 1].timestamp;
        }
        const double sleep_seconds = frame_interval - tracking_seconds;
        if (sleep_seconds > 0.0) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(sleep_seconds));
        }
    }

    const double mean_tracking_ms =
        samples.empty() ? 0.0
                        : 1000.0 * total_tracking_seconds / samples.size();
    AINFO << "Atlas KITTI stereo valid frames: " << valid_frames << "/"
          << samples.size() << ", mean tracking: " << mean_tracking_ms << " ms"
          << std::endl;

    std::shared_ptr<const atlas::map::MapSnapshot> map_snapshot;
    if (argc >= 5) {
        if (!builder.ShutdownAndSaveKittiTrajectory(argv[4])) {
            AERROR << "failed to save KITTI trajectory: " << argv[4]
                   << std::endl;
            return 1;
        }
        // ShutdownAndSaveKittiTrajectory drains the asynchronous mapping
        // worker, so this is the final map rather than an in-flight snapshot.
        map_snapshot = builder.LatestMappingSnapshot();
    } else {
        for (int attempt = 0; attempt < 20; ++attempt) {
            map_snapshot = builder.LatestMappingSnapshot();
            if (map_snapshot && !map_snapshot->dense_map.points.empty())
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        builder.Stop();
    }

    if (map_snapshot && !map_snapshot->dense_map.points.empty()) {
        const auto& occupancy = map_snapshot->occupancy_grid;
        std::size_t unknown_cells = 0;
        std::size_t free_cells = 0;
        std::size_t inflated_cells = 0;
        std::size_t occupied_cells = 0;
        for (const std::int8_t value : occupancy.data) {
            if (value < 0) {
                ++unknown_cells;
            } else if (value == 0) {
                ++free_cells;
            } else if (value == 100) {
                ++occupied_cells;
            } else {
                ++inflated_cells;
            }
        }
        AINFO << "Atlas KITTI dense map points: "
              << map_snapshot->dense_map.points.size()
              << ", generation: " << map_snapshot->dense_map.generation
              << ", navigation points [-1m, 1m]: "
              << map_snapshot->navigation_map.points.size()
              << ", occupancy grid: " << occupancy.width << "x"
              << occupancy.height << ", cells: " << occupancy.data.size()
              << " (unknown=" << unknown_cells << ", free=" << free_cells
              << ", inflated=" << inflated_cells
              << ", occupied=" << occupied_cells << ")" << std::endl;
    } else {
        AWARN << "Atlas KITTI dense map snapshot is not available"
              << std::endl;
    }

    if (argc == 6) {
        if (!map_snapshot || map_snapshot->dense_map.points.empty()) {
            AERROR << "cannot save an empty KITTI dense map" << std::endl;
            return 1;
        }
        if (!WriteDenseMapPly(argv[5], map_snapshot->dense_map)) {
            AERROR << "failed to save KITTI dense map PLY: " << argv[5]
                   << std::endl;
            return 1;
        }
        AINFO << "Atlas KITTI dense map saved: " << argv[5] << std::endl;
    }
    return 0;
}
