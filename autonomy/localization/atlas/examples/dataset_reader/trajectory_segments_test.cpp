#include "examples/dataset_reader/trajectory_segments.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas::examples {
namespace {

TrackingResult Result(double time, double x, bool valid = true,
                      std::uint64_t map_id = 1,
                      bool map_id_valid = true,
                      std::uint64_t coordinate_epoch = 0,
                      std::uint64_t destructive_epoch = 0) {
    TrackingResult result;
    result.time = time;
    result.valid = valid;
    result.map_id = map_id;
    result.map_id_valid = map_id_valid;
    result.coordinate_epoch = coordinate_epoch;
    result.destructive_epoch = destructive_epoch;
    result.pose_world_camera.translation.x() = x;
    return result;
}

TrajectorySample Sample(double time, float x) {
    TrajectorySample sample;
    sample.timestamp = time;
    sample.world_from_camera.translation().x() = x;
    return sample;
}

std::filesystem::path TestPath(const std::string& name) {
    const std::filesystem::path path =
        std::filesystem::path(::testing::TempDir()) / name;
    std::error_code error;
    for (const auto& entry :
         std::filesystem::directory_iterator(path.parent_path(), error)) {
        if (!error && entry.path().filename().string().rfind(
                          path.filename().string(), 0) == 0) {
            std::filesystem::remove(entry.path(), error);
        }
    }
    return path;
}

std::size_t DataRows(const std::filesystem::path& path) {
    std::ifstream input(path);
    std::size_t rows = 0;
    std::string line;
    while (std::getline(input, line)) {
        if (!line.empty() && line.front() != '#')
            ++rows;
    }
    return rows;
}

double FirstTranslationX(const std::filesystem::path& path) {
    std::ifstream input(path);
    double time = 0.0;
    double x = 0.0;
    input >> time >> x;
    return x;
}

TEST(TrajectorySegmentRecorderTest,
     SelectsLongestRawSegmentInsteadOfJoiningResetFrames) {
    TrajectorySegmentRecorder recorder;
    recorder.Record(Result(1.0, 1.0));
    recorder.Record(Result(2.0, 2.0));
    recorder.Record(Result(2.5, 0.0, false, 2));
    recorder.Record(Result(3.0, 30.0, true, 2));

    const std::filesystem::path path =
        TestPath("atlas_trajectory_segments_longest.tum");
    const std::filesystem::path optimized = path.string() + ".active.tum";
    ASSERT_TRUE(TrajectoryIO::SaveTum(
        optimized.string(), std::vector<TrajectorySample>{Sample(3.0, 99.0F)}));
    ASSERT_TRUE(recorder.SaveTum(path.string(), optimized.string()));

    EXPECT_EQ(recorder.valid_pose_count(), 3U);
    EXPECT_EQ(recorder.segment_count(), 2U);
    EXPECT_EQ(DataRows(path), 2U);
    EXPECT_FLOAT_EQ(static_cast<float>(FirstTranslationX(path)), 1.0F);
    EXPECT_EQ(DataRows(path.string() + ".segment-000.tum"), 2U);
    EXPECT_EQ(DataRows(path.string() + ".segment-001.tum"), 1U);

    std::ifstream manifest(path.string() + ".segments.csv");
    const std::string contents((std::istreambuf_iterator<char>(manifest)),
                               std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("raw,0"), std::string::npos);
    EXPECT_NE(contents.find("true,false,1,true,0,0,independent_atlas_map"),
              std::string::npos);
}

TEST(TrajectorySegmentRecorderTest,
     SplitsScaleAndDestructiveEpochsWithoutChangingMapId) {
    TrajectorySegmentRecorder recorder;
    recorder.Record(Result(1.0, 1.0, true, 4, true, 0, 0));
    recorder.Record(Result(2.0, 2.0, true, 4, true, 1, 0));
    recorder.Record(Result(3.0, 3.0, true, 4, true, 2, 1));

    const std::filesystem::path path =
        TestPath("atlas_trajectory_segments_coordinate_epochs.tum");
    const std::filesystem::path optimized = path.string() + ".active.tum";
    ASSERT_TRUE(TrajectoryIO::SaveTum(
        optimized.string(), std::vector<TrajectorySample>{Sample(3.0, 30.0F)}));
    ASSERT_TRUE(recorder.SaveTum(path.string(), optimized.string()));

    EXPECT_EQ(recorder.valid_pose_count(), 3U);
    EXPECT_EQ(recorder.segment_count(), 3U);
    EXPECT_EQ(DataRows(path.string() + ".segment-000.tum"), 1U);
    EXPECT_EQ(DataRows(path.string() + ".segment-001.tum"), 1U);
    EXPECT_EQ(DataRows(path.string() + ".segment-002.tum"), 1U);
    EXPECT_FLOAT_EQ(static_cast<float>(FirstTranslationX(path)), 30.0F);

    std::ifstream manifest(path.string() + ".segments.csv");
    const std::string contents((std::istreambuf_iterator<char>(manifest)),
                               std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("raw,1"), std::string::npos);
    EXPECT_NE(contents.find(",4,true,1,0,independent_atlas_map"),
              std::string::npos);
    EXPECT_NE(contents.find(",4,true,2,1,active_map_only"),
              std::string::npos);
}

TEST(TrajectorySegmentRecorderTest, InvalidPoseDoesNotSplitUnchangedMap) {
    TrajectorySegmentRecorder recorder;
    recorder.Record(Result(1.0, 1.0));
    recorder.Record(Result(2.0, 0.0, false));
    recorder.Record(Result(3.0, 3.0));

    const std::filesystem::path path =
        TestPath("atlas_trajectory_segments_invalid_same_map.tum");
    ASSERT_TRUE(recorder.SaveTum(path.string(), ""));
    EXPECT_EQ(recorder.valid_pose_count(), 2U);
    EXPECT_EQ(recorder.segment_count(), 1U);
    EXPECT_EQ(DataRows(path), 2U);
}

TEST(TrajectorySegmentRecorderTest, KeepsFinalOptimizedSegmentWhenComplete) {
    TrajectorySegmentRecorder recorder;
    recorder.Record(Result(1.0, 1.0));
    recorder.Record(Result(2.0, 2.0));

    const std::filesystem::path path =
        TestPath("atlas_trajectory_segments_optimized.tum");
    const std::filesystem::path optimized = path.string() + ".active.tum";
    ASSERT_TRUE(TrajectoryIO::SaveTum(
        optimized.string(),
        std::vector<TrajectorySample>{Sample(1.0, 42.0F),
                                      Sample(2.0, 43.0F)}));
    ASSERT_TRUE(recorder.SaveTum(path.string(), optimized.string()));

    EXPECT_EQ(DataRows(path), 2U);
    EXPECT_FLOAT_EQ(static_cast<float>(FirstTranslationX(path)), 42.0F);
    std::ifstream manifest(path.string() + ".segments.csv");
    const std::string contents((std::istreambuf_iterator<char>(manifest)),
                               std::istreambuf_iterator<char>());
    EXPECT_NE(contents.find("optimized_active,-1"), std::string::npos);
    EXPECT_NE(contents.find("true,true,1,true,0,0,active_map_only"),
              std::string::npos);
}

TEST(TrajectorySegmentRecorderTest, RejectsInvalidAndNonFinitePoses) {
    TrajectorySegmentRecorder recorder;
    recorder.Record(Result(1.0, 1.0, false));
    TrackingResult non_finite = Result(2.0, 2.0);
    non_finite.pose_world_camera.translation.x() =
        std::numeric_limits<double>::quiet_NaN();
    recorder.Record(non_finite);
    EXPECT_EQ(recorder.valid_pose_count(), 0U);
    EXPECT_EQ(recorder.segment_count(), 0U);
    EXPECT_FALSE(recorder.SaveTum(
        TestPath("atlas_trajectory_segments_empty.tum").string(), ""));
}

}  // namespace
}  // namespace autonomy::localization::atlas::examples
