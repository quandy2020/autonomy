//
// Created by rgrandia on 27.04.22.
//

#include <gtest/gtest.h>

#include <bolza/core/misc/load_std_vector_of_pair.hpp>

#include <filesystem>

namespace {
const std::string dataFolder = std::filesystem::path(__FILE__).parent_path().generic_string() + "/data/";
}

TEST(testLoadPair, loadStringPair) {
  std::vector<std::pair<std::string, std::string>> loadVector;
  bolza::loadData::LoadStdVectorOfPair(dataFolder + "/pairVectors.info", "stringPairs", loadVector);

  EXPECT_EQ(loadVector.size(), 2);
  EXPECT_EQ(loadVector[0].first, "s1");
  EXPECT_EQ(loadVector[0].second, "s2");
  EXPECT_EQ(loadVector[1].first, "s3");
  EXPECT_EQ(loadVector[1].second, "s4");
}

TEST(testLoadPair, loadSizePair) {
  std::vector<std::pair<size_t, size_t>> loadVector;
  bolza::loadData::LoadStdVectorOfPair(dataFolder + "/pairVectors.info", "sizePairs", loadVector);

  EXPECT_EQ(loadVector.size(), 2);
  EXPECT_EQ(loadVector[0].first, 1);
  EXPECT_EQ(loadVector[0].second, 2);
  EXPECT_EQ(loadVector[1].first, 3);
  EXPECT_EQ(loadVector[1].second, 4);
}

TEST(testLoadPair, loadStringScalarPair) {
  std::vector<std::pair<std::string, bolza::scalar_t>> loadVector;
  bolza::loadData::LoadStdVectorOfPair(dataFolder + "/pairVectors.info", "stringScalarPairs", loadVector);

  EXPECT_EQ(loadVector.size(), 2);
  EXPECT_EQ(loadVector[0].first, "s1");
  EXPECT_DOUBLE_EQ(loadVector[0].second, 2.1);
  EXPECT_EQ(loadVector[1].first, "s3");
  EXPECT_DOUBLE_EQ(loadVector[1].second, 4.3);
}

TEST(testLoadPair, loadStringSizePair) {
  std::vector<std::pair<std::string, size_t>> loadVector;
  bolza::loadData::LoadStdVectorOfPair(dataFolder + "/pairVectors.info", "stringSizePairs", loadVector);

  EXPECT_EQ(loadVector.size(), 2);
  EXPECT_EQ(loadVector[0].first, "s1");
  EXPECT_EQ(loadVector[0].second, 2);
  EXPECT_EQ(loadVector[1].first, "s3");
  EXPECT_EQ(loadVector[1].second, 4);
}