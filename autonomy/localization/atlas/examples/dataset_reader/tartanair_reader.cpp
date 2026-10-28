#include "examples/dataset_reader/tartanair_reader.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace autonomy::localization::atlas::examples {
namespace {

bool HasSuffix(const std::string& value, const std::string& suffix) {
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) ==
               0;
}

bool IsStrictlyIncreasing(const std::vector<double>& values) {
    if (values.empty())
        return false;
    for (std::size_t index = 0; index < values.size(); ++index) {
        if (!std::isfinite(values[index]) ||
            (index > 0 && values[index] <= values[index - 1])) {
            return false;
        }
    }
    return true;
}

bool ReadScalarFile(const std::filesystem::path& path,
                    std::vector<double>* values) {
    std::ifstream input(path);
    if (!input)
        return false;

    std::vector<double> parsed;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty())
            return false;
        std::istringstream row(line);
        double value = 0.0;
        std::string extra;
        if (!(row >> value) || (row >> extra) || !std::isfinite(value))
            return false;
        parsed.push_back(value);
    }
    if (!IsStrictlyIncreasing(parsed))
        return false;
    *values = std::move(parsed);
    return true;
}

bool ReadVectorFile(const std::filesystem::path& path,
                    std::vector<Eigen::Vector3d>* values) {
    std::ifstream input(path);
    if (!input)
        return false;

    std::vector<Eigen::Vector3d> parsed;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty())
            return false;
        std::istringstream row(line);
        Eigen::Vector3d value;
        std::string extra;
        if (!(row >> value.x() >> value.y() >> value.z()) || (row >> extra) ||
            !value.allFinite()) {
            return false;
        }
        parsed.push_back(value);
    }
    if (parsed.empty())
        return false;
    *values = std::move(parsed);
    return true;
}

bool CountFilesWithSuffix(const std::filesystem::path& directory,
                          const std::string& suffix, std::size_t* count) {
    if (count == nullptr)
        return false;
    std::error_code error;
    if (!std::filesystem::is_directory(directory, error) || error)
        return false;

    std::size_t result = 0;
    std::filesystem::directory_iterator iterator(directory, error);
    const std::filesystem::directory_iterator end;
    while (!error && iterator != end) {
        const auto& entry = *iterator;
        if (entry.is_regular_file(error) && !error &&
            HasSuffix(entry.path().filename().string(), suffix)) {
            ++result;
        }
        iterator.increment(error);
    }
    if (error)
        return false;
    *count = result;
    return true;
}

std::string Trim(const std::string& value) {
    const std::size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
        return std::string();
    const std::size_t end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

bool ParseShape(const std::string& text, std::size_t* rows,
                std::size_t* columns) {
    std::vector<std::size_t> dimensions;
    std::istringstream input(text);
    std::string token;
    while (std::getline(input, token, ',')) {
        token = Trim(token);
        if (token.empty())
            continue;
        std::size_t parsed_characters = 0;
        try {
            const unsigned long long value =
                std::stoull(token, &parsed_characters);
            if (parsed_characters != token.size() || value == 0 ||
                value > std::numeric_limits<std::size_t>::max()) {
                return false;
            }
            dimensions.push_back(static_cast<std::size_t>(value));
        } catch (const std::exception&) {
            return false;
        }
    }
    if (dimensions.size() != 2)
        return false;
    *rows = dimensions[0];
    *columns = dimensions[1];
    return true;
}

bool ParseNpyHeader(const std::string& header, std::size_t* rows,
                    std::size_t* columns) {
    const std::regex descriptor_pattern(
        R"REGEX(['"]descr['"]\s*:\s*['"]([^'"]+)['"])REGEX");
    const std::regex fortran_pattern(
        R"REGEX(['"]fortran_order['"]\s*:\s*(True|False))REGEX");
    const std::regex shape_pattern(
        R"REGEX(['"]shape['"]\s*:\s*\(([^)]*)\))REGEX");
    std::smatch descriptor_match;
    std::smatch fortran_match;
    std::smatch shape_match;
    if (!std::regex_search(header, descriptor_match, descriptor_pattern) ||
        !std::regex_search(header, fortran_match, fortran_pattern) ||
        !std::regex_search(header, shape_match, shape_pattern)) {
        return false;
    }
    const std::string descriptor = descriptor_match[1].str();
    if (descriptor != "<f4" && descriptor != "=f4")
        return false;
    if (fortran_match[1].str() != "False")
        return false;
    return ParseShape(shape_match[1].str(), rows, columns);
}

bool ReadLittleEndianLength(std::istream* input, std::size_t byte_count,
                            std::uint32_t* value) {
    if (input == nullptr || value == nullptr ||
        (byte_count != 2 && byte_count != 4)) {
        return false;
    }
    std::array<unsigned char, 4> bytes{};
    input->read(reinterpret_cast<char*>(bytes.data()),
                static_cast<std::streamsize>(byte_count));
    if (!*input)
        return false;
    std::uint32_t parsed = 0;
    for (std::size_t index = 0; index < byte_count; ++index) {
        parsed |= static_cast<std::uint32_t>(bytes[index]) << (8 * index);
    }
    *value = parsed;
    return true;
}

constexpr float kMaximumTartanAirDepthMeters = 80.0F;

float SanitizeDepth(float depth_meters) {
    if (!std::isfinite(depth_meters) || depth_meters <= 0.0F ||
        depth_meters > kMaximumTartanAirDepthMeters) {
        return 0.0F;
    }
    return depth_meters;
}

bool LoadPackedFloatDepthPng(const std::string& path, cv::Mat* depth_meters) {
    if (depth_meters == nullptr)
        return false;
    depth_meters->release();

    // TartanAir V2 packs the four little-endian bytes of each float32 depth
    // value into an 8-bit PNG pixel. OpenCV decodes the stored RGBA channels
    // back to the original BGRA byte order used by the dataset writer.
    const cv::Mat encoded = cv::imread(path, cv::IMREAD_UNCHANGED);
    if (encoded.empty() || encoded.type() != CV_8UC4)
        return false;

    static_assert(sizeof(float) == sizeof(std::uint32_t));
    cv::Mat parsed(encoded.rows, encoded.cols, CV_32FC1);
    for (int row = 0; row < encoded.rows; ++row) {
        const std::uint8_t* input = encoded.ptr<std::uint8_t>(row);
        float* output = parsed.ptr<float>(row);
        for (int column = 0; column < encoded.cols; ++column) {
            const std::uint8_t* bytes = input + 4 * column;
            const std::uint32_t bits =
                static_cast<std::uint32_t>(bytes[0]) |
                (static_cast<std::uint32_t>(bytes[1]) << 8U) |
                (static_cast<std::uint32_t>(bytes[2]) << 16U) |
                (static_cast<std::uint32_t>(bytes[3]) << 24U);
            std::memcpy(&output[column], &bits, sizeof(float));
            output[column] = SanitizeDepth(output[column]);
        }
    }
    *depth_meters = std::move(parsed);
    return true;
}

std::string FrameStem(std::size_t index) {
    std::ostringstream output;
    output << std::setfill('0') << std::setw(6) << index << "_lcam_front";
    return output.str();
}

}  // namespace

bool TartanAirReader::LoadDepthNpy(const std::string& path,
                                   cv::Mat* depth_meters) {
    if (depth_meters == nullptr)
        return false;
    depth_meters->release();

    std::ifstream input(path, std::ios::binary);
    if (!input)
        return false;

    constexpr std::array<unsigned char, 6> kMagic{0x93, 'N', 'U',
                                                  'M',  'P', 'Y'};
    std::array<unsigned char, 6> magic{};
    input.read(reinterpret_cast<char*>(magic.data()),
               static_cast<std::streamsize>(magic.size()));
    if (!input || magic != kMagic)
        return false;

    std::array<unsigned char, 2> version{};
    input.read(reinterpret_cast<char*>(version.data()), version.size());
    if (!input)
        return false;
    std::size_t length_bytes = 0;
    if (version[0] == 1) {
        length_bytes = 2;
    } else if (version[0] == 2 || version[0] == 3) {
        length_bytes = 4;
    } else {
        return false;
    }

    std::uint32_t header_length = 0;
    if (!ReadLittleEndianLength(&input, length_bytes, &header_length) ||
        header_length == 0) {
        return false;
    }
    constexpr std::uint32_t kMaxHeaderLength = 1024U * 1024U;
    if (header_length > kMaxHeaderLength ||
        header_length > static_cast<std::uint64_t>(
                            std::numeric_limits<std::streamsize>::max())) {
        return false;
    }
    std::string header(header_length, '\0');
    input.read(header.data(), static_cast<std::streamsize>(header.size()));
    if (!input)
        return false;

    std::size_t rows = 0;
    std::size_t columns = 0;
    if (!ParseNpyHeader(header, &rows, &columns) ||
        rows > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        columns > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
        rows > std::numeric_limits<std::size_t>::max() / columns) {
        return false;
    }
    const std::size_t element_count = rows * columns;
    if (element_count >
        std::numeric_limits<std::size_t>::max() / sizeof(float)) {
        return false;
    }
    const std::size_t data_bytes = element_count * sizeof(float);
    if (data_bytes > static_cast<std::uint64_t>(
                         std::numeric_limits<std::streamsize>::max())) {
        return false;
    }
    const std::streampos data_position = input.tellg();
    if (data_position == std::streampos(-1))
        return false;
    input.seekg(0, std::ios::end);
    const std::streampos end_position = input.tellg();
    if (end_position == std::streampos(-1) || end_position < data_position ||
        static_cast<std::uint64_t>(end_position - data_position) !=
            data_bytes) {
        return false;
    }
    input.seekg(data_position);
    if (!input)
        return false;

    cv::Mat parsed(static_cast<int>(rows), static_cast<int>(columns), CV_32FC1);
    input.read(reinterpret_cast<char*>(parsed.ptr<float>()),
               static_cast<std::streamsize>(data_bytes));
    if (!input)
        return false;
    for (std::size_t index = 0; index < element_count; ++index) {
        float& depth = parsed.ptr<float>()[index];
        depth = SanitizeDepth(depth);
    }
    *depth_meters = std::move(parsed);
    return true;
}

bool TartanAirReader::LoadDepth(const std::string& path,
                                cv::Mat* depth_meters) {
    if (depth_meters == nullptr)
        return false;
    depth_meters->release();

    const std::string extension =
        std::filesystem::path(path).extension().string();
    if (extension == ".png")
        return LoadPackedFloatDepthPng(path, depth_meters);
    if (extension == ".npy")
        return LoadDepthNpy(path, depth_meters);
    return false;
}

bool TartanAirReader::LoadSequence(const std::string& sequence_root,
                                   TartanAirSequence* sequence) {
    if (sequence == nullptr)
        return false;
    *sequence = TartanAirSequence{};

    const std::filesystem::path root(sequence_root);
    const std::filesystem::path imu_directory = root / "imu";
    std::vector<double> camera_times;
    std::vector<double> imu_times;
    std::vector<Eigen::Vector3d> accelerations;
    std::vector<Eigen::Vector3d> angular_velocities;
    if (!ReadScalarFile(imu_directory / "cam_time.txt", &camera_times) ||
        !ReadScalarFile(imu_directory / "imu_time.txt", &imu_times) ||
        !ReadVectorFile(imu_directory / "acc.txt", &accelerations) ||
        !ReadVectorFile(imu_directory / "gyro.txt", &angular_velocities) ||
        imu_times.size() != accelerations.size() ||
        imu_times.size() != angular_velocities.size()) {
        return false;
    }

    const std::filesystem::path color_directory = root / "image_lcam_front";
    const std::filesystem::path depth_directory = root / "depth_lcam_front";
    std::size_t color_count = 0;
    std::size_t packed_png_count = 0;
    std::size_t npy_count = 0;
    if (!CountFilesWithSuffix(color_directory, "_lcam_front.png",
                              &color_count) ||
        !CountFilesWithSuffix(depth_directory, "_lcam_front_depth.png",
                              &packed_png_count) ||
        !CountFilesWithSuffix(depth_directory, "_lcam_front_depth.npy",
                              &npy_count) ||
        color_count != camera_times.size()) {
        return false;
    }
    std::string depth_suffix;
    if (packed_png_count == camera_times.size()) {
        depth_suffix = "_depth.png";
    } else if (npy_count == camera_times.size()) {
        depth_suffix = "_depth.npy";
    } else {
        return false;
    }

    TartanAirSequence parsed;
    parsed.frames.reserve(camera_times.size());
    for (std::size_t index = 0; index < camera_times.size(); ++index) {
        const std::string stem = FrameStem(index);
        TartanAirRgbdSample frame;
        frame.timestamp = camera_times[index];
        frame.color_path = (color_directory / (stem + ".png")).string();
        frame.depth_path = (depth_directory / (stem + depth_suffix)).string();
        std::error_code error;
        if (!std::filesystem::is_regular_file(frame.color_path, error) ||
            error ||
            !std::filesystem::is_regular_file(frame.depth_path, error) ||
            error) {
            return false;
        }
        parsed.frames.push_back(std::move(frame));
    }

    parsed.imu.reserve(imu_times.size());
    for (std::size_t index = 0; index < imu_times.size(); ++index) {
        ImuSample sample;
        sample.time = imu_times[index];
        sample.linear_acceleration = accelerations[index];
        sample.angular_velocity = angular_velocities[index];
        parsed.imu.push_back(sample);
    }
    *sequence = std::move(parsed);
    return true;
}

}  // namespace autonomy::localization::atlas::examples
