#include <chrono>
#include <cstddef>
#include <thread>

#include <opencv2/imgcodecs.hpp>

#include "atlas/common/logging.hpp"
#include "atlas/map_builder.hpp"
#include "examples/dataset_reader/tartanair_reader.hpp"

int main(int argc, char** argv) {
    namespace atlas = autonomy::localization::atlas;
    if (argc != 4 && argc != 5) {
        AERROR << "usage: rgbd_inertial_tartanair VOCAB CONFIG SEQUENCE_ROOT "
                  "[TUM_TRAJECTORY]"
               << std::endl;
        return 2;
    }

    atlas::examples::TartanAirSequence sequence;
    if (!atlas::examples::TartanAirReader::LoadSequence(argv[3], &sequence) ||
        sequence.frames.empty() || sequence.imu.empty()) {
        AERROR << "failed to load a complete TartanAir V2 RGB-D+IMU sequence"
               << std::endl;
        return 1;
    }

    AWARN << "TartanAir RGB-D+IMU uses the documented AirSim NED-body to "
             "OpenCV-optical fixed rotation with co-located sensors. This "
             "entry validates the synthetic-data interface only; it does not "
             "establish real-sensor accuracy."
          << std::endl;

    atlas::MapBuilder builder(atlas::SystemConfig{
        argv[1], argv[2], atlas::SensorMode::kRgbdInertial, false});
    if (!builder.Start())
        return 1;

    std::size_t imu_index = 0;
    while (imu_index < sequence.imu.size() &&
           sequence.imu[imu_index].time <= sequence.frames.front().timestamp) {
        ++imu_index;
    }
    if (imu_index > 0)
        --imu_index;

    std::size_t accepted_frames = 0;
    std::size_t valid_poses = 0;
    for (std::size_t index = 0; index < sequence.frames.size(); ++index) {
        const auto& sample = sequence.frames[index];
        if (index > 0) {
            while (imu_index < sequence.imu.size() &&
                   sequence.imu[imu_index].time <= sample.timestamp) {
                if (!builder.AddImu(sequence.imu[imu_index])) {
                    AERROR << "rejected TartanAir IMU sample at "
                           << sequence.imu[imu_index].time << std::endl;
                    return 1;
                }
                ++imu_index;
            }
        }

        atlas::ImageFrame color{
            sample.timestamp, "lcam_front",
            cv::imread(sample.color_path, cv::IMREAD_COLOR)};
        cv::Mat depth_meters;
        if (color.pixels.empty() ||
            !atlas::examples::TartanAirReader::LoadDepth(sample.depth_path,
                                                         &depth_meters) ||
            depth_meters.size() != color.pixels.size()) {
            AERROR << "failed to load aligned TartanAir RGB-D frame " << index
                   << std::endl;
            return 1;
        }
        atlas::ImageFrame depth{sample.timestamp, "depth_lcam_front",
                                depth_meters};

        const auto tracking_start = std::chrono::steady_clock::now();
        atlas::TrackingResult result;
        if (!builder.TrackRgbd(color, depth, &result)) {
            AWARN << "TartanAir RGB-D+IMU frame rejected: " << index
                  << std::endl;
        } else {
            ++accepted_frames;
            if (result.valid)
                ++valid_poses;
        }
        const std::chrono::duration<double> tracking_duration =
            std::chrono::steady_clock::now() - tracking_start;
        double frame_interval = 0.0;
        if (index + 1 < sequence.frames.size()) {
            frame_interval =
                sequence.frames[index + 1].timestamp - sample.timestamp;
        } else if (index > 0) {
            frame_interval =
                sample.timestamp - sequence.frames[index - 1].timestamp;
        }
        const double sleep_seconds = frame_interval - tracking_duration.count();
        if (sleep_seconds > 0.0 && frame_interval < 1.0) {
            std::this_thread::sleep_for(
                std::chrono::duration<double>(sleep_seconds));
        }
    }

    AINFO << "Atlas TartanAir RGB-D+IMU summary: samples="
          << sequence.frames.size() << ", accepted=" << accepted_frames
          << ", valid_poses=" << valid_poses << std::endl;
    if (argc == 5) {
        if (!builder.ShutdownAndSaveTumTrajectory(argv[4])) {
            AERROR << "failed to save TartanAir TUM trajectory: " << argv[4]
                   << std::endl;
            return 1;
        }
    } else {
        builder.Stop();
    }
    return 0;
}
