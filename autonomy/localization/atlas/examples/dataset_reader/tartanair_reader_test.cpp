#include "examples/dataset_reader/tartanair_reader.hpp"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::examples {
namespace {

class ScopedTempDirectory
{
public:
    ScopedTempDirectory() {
        const auto unique_id =
            std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = std::filesystem::temp_directory_path() /
                ("atlas_tartanair_reader_test_" + std::to_string(unique_id));
        if (!std::filesystem::create_directories(path_)) {
            throw std::runtime_error("failed to create test directory");
        }
    }

    ~ScopedTempDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    const std::filesystem::path& path() const {
        return path_;
    }

private:
    std::filesystem::path path_;
};

void WriteNpy(const std::filesystem::path& path, std::size_t rows,
              std::size_t columns, const std::vector<float>& values) {
    ASSERT_EQ(values.size(), rows * columns);
    std::string header = "{'descr': '<f4', 'fortran_order': False, 'shape': (" +
                         std::to_string(rows) + ", " + std::to_string(columns) +
                         "), }";
    constexpr std::size_t kPreambleSize = 10;
    const std::size_t remainder = (kPreambleSize + header.size() + 1) % 16;
    if (remainder != 0)
        header.append(16 - remainder, ' ');
    header.push_back('\n');
    ASSERT_LE(header.size(), std::numeric_limits<std::uint16_t>::max());

    std::ofstream output(path, std::ios::binary);
    ASSERT_TRUE(output);
    const unsigned char magic[] = {0x93, 'N', 'U', 'M', 'P', 'Y'};
    output.write(reinterpret_cast<const char*>(magic), sizeof(magic));
    output.put(1);
    output.put(0);
    const auto header_size = static_cast<std::uint16_t>(header.size());
    output.put(static_cast<char>(header_size & 0xff));
    output.put(static_cast<char>((header_size >> 8) & 0xff));
    output.write(header.data(), static_cast<std::streamsize>(header.size()));
    output.write(reinterpret_cast<const char*>(values.data()),
                 static_cast<std::streamsize>(values.size() * sizeof(float)));
    ASSERT_TRUE(output);
}

void WritePackedDepthPng(const std::filesystem::path& path, int rows,
                         int columns, const std::vector<float>& values) {
    ASSERT_EQ(values.size(), static_cast<std::size_t>(rows * columns));
    static_assert(sizeof(float) == sizeof(std::uint32_t));
    cv::Mat encoded(rows, columns, CV_8UC4);
    for (int row = 0; row < rows; ++row) {
        std::uint8_t* output = encoded.ptr<std::uint8_t>(row);
        for (int column = 0; column < columns; ++column) {
            std::uint32_t bits = 0;
            const float value = values[row * columns + column];
            std::memcpy(&bits, &value, sizeof(float));
            output[4 * column] = static_cast<std::uint8_t>(bits);
            output[4 * column + 1] = static_cast<std::uint8_t>(bits >> 8U);
            output[4 * column + 2] = static_cast<std::uint8_t>(bits >> 16U);
            output[4 * column + 3] = static_cast<std::uint8_t>(bits >> 24U);
        }
    }
    ASSERT_TRUE(cv::imwrite(path.string(), encoded));
}

void WriteText(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path);
    ASSERT_TRUE(output);
    output << text;
    ASSERT_TRUE(output);
}

TEST(TartanAirReaderTest, ReadsLittleEndianFloatDepthNpy) {
    ScopedTempDirectory directory;
    const auto path = directory.path() / "depth.npy";
    WriteNpy(path, 2, 3, {1.0F, 2.0F, 3.5F, 4.0F, 5.25F, 6.0F});

    cv::Mat depth;
    ASSERT_TRUE(TartanAirReader::LoadDepthNpy(path.string(), &depth));
    ASSERT_EQ(depth.type(), CV_32FC1);
    ASSERT_EQ(depth.rows, 2);
    ASSERT_EQ(depth.cols, 3);
    EXPECT_FLOAT_EQ(depth.at<float>(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(depth.at<float>(0, 2), 3.5F);
    EXPECT_FLOAT_EQ(depth.at<float>(1, 1), 5.25F);
}

TEST(TartanAirReaderTest, ReadsOfficialPackedFloatDepthPng) {
    ScopedTempDirectory directory;
    const auto path = directory.path() / "depth.png";
    WritePackedDepthPng(path, 2, 3, {0.75F, 1.0F, 2.5F, 10.0F, 80.0F, 8240.0F});

    cv::Mat depth;
    ASSERT_TRUE(TartanAirReader::LoadDepth(path.string(), &depth));
    ASSERT_EQ(depth.type(), CV_32FC1);
    ASSERT_EQ(depth.rows, 2);
    ASSERT_EQ(depth.cols, 3);
    EXPECT_FLOAT_EQ(depth.at<float>(0, 0), 0.75F);
    EXPECT_FLOAT_EQ(depth.at<float>(0, 2), 2.5F);
    EXPECT_FLOAT_EQ(depth.at<float>(1, 1), 80.0F);
    EXPECT_FLOAT_EQ(depth.at<float>(1, 2), 0.0F);
}

TEST(TartanAirReaderTest, SanitizesInvalidAndOutOfRangeDepth) {
    ScopedTempDirectory directory;
    const auto path = directory.path() / "depth.npy";
    WriteNpy(path, 1, 6,
             {1.0F, 0.0F, -1.0F, 80.0F, 80.01F,
              std::numeric_limits<float>::quiet_NaN()});

    cv::Mat depth;
    ASSERT_TRUE(TartanAirReader::LoadDepth(path.string(), &depth));
    EXPECT_FLOAT_EQ(depth.at<float>(0, 0), 1.0F);
    EXPECT_FLOAT_EQ(depth.at<float>(0, 1), 0.0F);
    EXPECT_FLOAT_EQ(depth.at<float>(0, 2), 0.0F);
    EXPECT_FLOAT_EQ(depth.at<float>(0, 3), 80.0F);
    EXPECT_FLOAT_EQ(depth.at<float>(0, 4), 0.0F);
    EXPECT_FLOAT_EQ(depth.at<float>(0, 5), 0.0F);
}

TEST(TartanAirReaderTest, LoadsStrictlyAlignedRgbdImuSequence) {
    ScopedTempDirectory directory;
    const auto root = directory.path();
    const auto color_directory = root / "image_lcam_front";
    const auto depth_directory = root / "depth_lcam_front";
    const auto imu_directory = root / "imu";
    ASSERT_TRUE(std::filesystem::create_directories(color_directory));
    ASSERT_TRUE(std::filesystem::create_directories(depth_directory));
    ASSERT_TRUE(std::filesystem::create_directories(imu_directory));

    WriteText(color_directory / "000000_lcam_front.png", "image");
    WriteText(color_directory / "000001_lcam_front.png", "image");
    WritePackedDepthPng(depth_directory / "000000_lcam_front_depth.png", 1, 1,
                        {1.0F});
    WritePackedDepthPng(depth_directory / "000001_lcam_front_depth.png", 1, 1,
                        {2.0F});
    WriteText(imu_directory / "cam_time.txt", "0.0\n0.1\n");
    WriteText(imu_directory / "imu_time.txt", "0.0\n0.01\n0.1\n");
    WriteText(imu_directory / "acc.txt",
              "0.0 0.0 9.81\n0.1 0.0 9.80\n0.2 0.0 9.79\n");
    WriteText(imu_directory / "gyro.txt",
              "0.0 0.0 0.0\n0.01 0.02 0.03\n0.02 0.03 0.04\n");

    TartanAirSequence sequence;
    ASSERT_TRUE(TartanAirReader::LoadSequence(root.string(), &sequence));
    ASSERT_EQ(sequence.frames.size(), 2U);
    ASSERT_EQ(sequence.imu.size(), 3U);
    EXPECT_DOUBLE_EQ(sequence.frames[1].timestamp, 0.1);
    EXPECT_DOUBLE_EQ(sequence.imu[1].time, 0.01);
    EXPECT_DOUBLE_EQ(sequence.imu[2].linear_acceleration.z(), 9.79);
    EXPECT_EQ(std::filesystem::path(sequence.frames[0].depth_path).filename(),
              "000000_lcam_front_depth.png");
}

}  // namespace
}  // namespace autonomy::localization::atlas::examples
