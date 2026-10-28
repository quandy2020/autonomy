#include "examples/dataset_reader/euroc_reader.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

namespace autonomy::localization::atlas::examples {
namespace {

std::string Join(const std::string& root, const std::string& suffix) {
    return root.empty() || root.back() == '/' ? root + suffix
                                              : root + '/' + suffix;
}

}  // namespace

bool EurocReader::LoadImages(const std::string& dataset_root,
                             const std::string& timestamps_file, bool stereo,
                             std::vector<EurocImageSample>* samples) {
    if (samples == nullptr)
        return false;
    std::ifstream input(timestamps_file);
    if (!input)
        return false;
    std::vector<EurocImageSample> parsed;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#')
            continue;
        std::istringstream row(line);
        std::string stamp;
        row >> stamp;
        if (stamp.empty())
            return false;
        try {
            EurocImageSample sample;
            sample.timestamp = std::stod(stamp) * 1.0e-9;
            if (!std::isfinite(sample.timestamp))
                return false;
            sample.left_path =
                Join(dataset_root, "mav0/cam0/data/" + stamp + ".png");
            if (stereo) {
                sample.right_path =
                    Join(dataset_root, "mav0/cam1/data/" + stamp + ".png");
            }
            parsed.push_back(std::move(sample));
        } catch (const std::exception&) {
            return false;
        }
    }
    *samples = std::move(parsed);
    return !samples->empty();
}

bool EurocReader::LoadImu(const std::string& csv_path,
                          std::vector<ImuSample>* samples) {
    if (samples == nullptr)
        return false;
    std::ifstream input(csv_path);
    if (!input)
        return false;
    std::vector<ImuSample> parsed;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#')
            continue;
        for (char& value : line) {
            if (value == ',')
                value = ' ';
        }
        std::istringstream row(line);
        double timestamp_ns = 0.0;
        ImuSample sample;
        row >> timestamp_ns >> sample.angular_velocity.x() >>
            sample.angular_velocity.y() >> sample.angular_velocity.z() >>
            sample.linear_acceleration.x() >> sample.linear_acceleration.y() >>
            sample.linear_acceleration.z();
        if (!row || !std::isfinite(timestamp_ns) ||
            !sample.angular_velocity.allFinite() ||
            !sample.linear_acceleration.allFinite()) {
            return false;
        }
        sample.time = timestamp_ns * 1.0e-9;
        parsed.push_back(sample);
    }
    *samples = std::move(parsed);
    return !samples->empty();
}

}  // namespace autonomy::localization::atlas::examples
