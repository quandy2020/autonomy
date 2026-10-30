#include "examples/dataset_reader/euroc_reader.hpp"

#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <system_error>

namespace autonomy::localization::atlas::examples {
namespace {

std::string Join(const std::string& root, const std::string& suffix) {
    return root.empty() || root.back() == '/' ? root + suffix
                                              : root + '/' + suffix;
}

std::string Trim(const std::string& value) {
    std::size_t begin = 0;
    while (begin < value.size() &&
           std::isspace(static_cast<unsigned char>(value[begin]))) {
        ++begin;
    }
    std::size_t end = value.size();
    while (end > begin &&
           std::isspace(static_cast<unsigned char>(value[end - 1]))) {
        --end;
    }
    return value.substr(begin, end - begin);
}

bool IsRegularFile(const std::string& path) {
    std::error_code error;
    return std::filesystem::is_regular_file(path, error) && !error;
}

}  // namespace

bool EurocReader::LoadImages(const std::string& dataset_root,
                             const std::string& timestamps_file, bool stereo,
                             std::vector<EurocImageSample>* samples,
                             std::size_t* skipped_missing_samples) {
    if (samples == nullptr)
        return false;
    if (skipped_missing_samples != nullptr)
        *skipped_missing_samples = 0;
    std::ifstream input(timestamps_file);
    if (!input)
        return false;
    std::vector<EurocImageSample> parsed;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed_line = Trim(line);
        if (trimmed_line.empty() || trimmed_line.front() == '#')
            continue;

        const std::size_t comma = trimmed_line.find(',');
        const std::string stamp = Trim(trimmed_line.substr(0, comma));
        if (stamp.empty())
            return false;
        std::string filename;
        if (comma != std::string::npos) {
            filename = Trim(trimmed_line.substr(comma + 1));
            if (filename.find(',') != std::string::npos)
                return false;
            if (filename.size() >= 2 && filename.front() == '"' &&
                filename.back() == '"') {
                filename = filename.substr(1, filename.size() - 2);
            }
        }

        try {
            EurocImageSample sample;
            std::size_t consumed = 0;
            sample.timestamp = std::stod(stamp, &consumed) * 1.0e-9;
            if (consumed != stamp.size() || !std::isfinite(sample.timestamp))
                return false;
            const std::string image_filename =
                filename.empty() ? stamp + ".png" : filename;
            sample.left_path =
                Join(dataset_root, "mav0/cam0/data/" + image_filename);
            if (stereo) {
                sample.right_path =
                    Join(dataset_root, "mav0/cam1/data/" + image_filename);
            }
            if (skipped_missing_samples != nullptr &&
                (!IsRegularFile(sample.left_path) ||
                 (stereo && !IsRegularFile(sample.right_path)))) {
                ++*skipped_missing_samples;
                continue;
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
