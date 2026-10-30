#include "examples/dataset_reader/euroc_reader.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::examples {
namespace {

class ScopedTextFile
{
public:
    ScopedTextFile(const std::string& filename, const std::string& contents) {
        path_ = ::testing::TempDir();
        if (!path_.empty() && path_.back() != '/')
            path_.push_back('/');
        path_ += filename;
        std::ofstream output(path_);
        if (!output)
            throw std::runtime_error("failed to create temporary test file");
        output << contents;
        if (!output)
            throw std::runtime_error("failed to write temporary test file");
    }

    ~ScopedTextFile() {
        std::remove(path_.c_str());
    }

    const std::string& path() const {
        return path_;
    }

private:
    std::string path_;
};

class ScopedDataset
{
public:
    explicit ScopedDataset(const std::string& name) {
        path_ = std::filesystem::path(::testing::TempDir()) / name;
        std::error_code error;
        std::filesystem::remove_all(path_, error);
        if (!std::filesystem::create_directories(path_))
            throw std::runtime_error("failed to create temporary dataset");
    }

    ~ScopedDataset() {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }

    void Touch(const std::string& relative_path) const {
        const std::filesystem::path path = path_ / relative_path;
        if (!std::filesystem::create_directories(path.parent_path()) &&
            !std::filesystem::is_directory(path.parent_path())) {
            throw std::runtime_error("failed to create image directory");
        }
        std::ofstream output(path);
        if (!output)
            throw std::runtime_error("failed to create image file");
    }

    std::string path() const {
        return path_.string();
    }

private:
    std::filesystem::path path_;
};

TEST(EurocReaderTest, LoadsOfficialCameraCsvWithOptionalFilename) {
    const ScopedTextFile timestamps(
        "atlas_euroc_official_data.csv",
        "#timestamp [ns],filename\n"
        "1403715273262142976,1403715273262142976.png\n"
        "1403715273312143104, custom_name.png  \n");

    std::vector<EurocImageSample> samples;
    ASSERT_TRUE(
        EurocReader::LoadImages("/dataset", timestamps.path(), true, &samples));
    ASSERT_EQ(samples.size(), 2U);
    EXPECT_DOUBLE_EQ(samples[0].timestamp,
                     std::stod("1403715273262142976") * 1.0e-9);
    EXPECT_EQ(samples[0].left_path,
              "/dataset/mav0/cam0/data/1403715273262142976.png");
    EXPECT_EQ(samples[0].right_path,
              "/dataset/mav0/cam1/data/1403715273262142976.png");
    EXPECT_EQ(samples[1].left_path, "/dataset/mav0/cam0/data/custom_name.png");
    EXPECT_EQ(samples[1].right_path, "/dataset/mav0/cam1/data/custom_name.png");
}

TEST(EurocReaderTest, PreservesPlainTimestampListCompatibility) {
    const ScopedTextFile timestamps("atlas_euroc_plain_timestamps.txt",
                                    "# one timestamp per line\n"
                                    "1403715273262142976\n"
                                    "  1403715273312143104  \n");

    std::vector<EurocImageSample> samples;
    ASSERT_TRUE(EurocReader::LoadImages("/dataset/", timestamps.path(), false,
                                        &samples));
    ASSERT_EQ(samples.size(), 2U);
    EXPECT_EQ(samples[0].left_path,
              "/dataset/mav0/cam0/data/1403715273262142976.png");
    EXPECT_TRUE(samples[0].right_path.empty());
    EXPECT_EQ(samples[1].left_path,
              "/dataset/mav0/cam0/data/1403715273312143104.png");
    EXPECT_TRUE(samples[1].right_path.empty());
}

TEST(EurocReaderTest, FiltersMissingLeftAndRightStereoImages) {
    const ScopedDataset dataset("atlas_euroc_missing_stereo_images");
    dataset.Touch("mav0/cam0/data/100.png");
    dataset.Touch("mav0/cam1/data/100.png");
    dataset.Touch("mav0/cam0/data/200.png");
    dataset.Touch("mav0/cam1/data/300.png");
    const ScopedTextFile timestamps(
        "atlas_euroc_missing_stereo_data.csv",
        "#timestamp [ns],filename\n"
        "100,100.png\n"
        "200,200.png\n"
        "300,300.png\n");

    std::size_t skipped_missing_samples = 0;
    std::vector<EurocImageSample> samples;
    ASSERT_TRUE(EurocReader::LoadImages(
        dataset.path(), timestamps.path(), true, &samples,
        &skipped_missing_samples));
    ASSERT_EQ(samples.size(), 1U);
    EXPECT_EQ(samples.front().left_path,
              dataset.path() + "/mav0/cam0/data/100.png");
    EXPECT_EQ(samples.front().right_path,
              dataset.path() + "/mav0/cam1/data/100.png");
    EXPECT_EQ(skipped_missing_samples, 2U);
}

TEST(EurocReaderTest, ReturnsFalseWhenEveryRequestedImageIsMissing) {
    const ScopedDataset dataset("atlas_euroc_all_images_missing");
    const ScopedTextFile timestamps(
        "atlas_euroc_all_images_missing.csv",
        "100,100.png\n"
        "200,200.png\n");

    std::size_t skipped_missing_samples = 0;
    std::vector<EurocImageSample> samples;
    EXPECT_FALSE(EurocReader::LoadImages(
        dataset.path(), timestamps.path(), true, &samples,
        &skipped_missing_samples));
    EXPECT_TRUE(samples.empty());
    EXPECT_EQ(skipped_missing_samples, 2U);
}

}  // namespace
}  // namespace autonomy::localization::atlas::examples
