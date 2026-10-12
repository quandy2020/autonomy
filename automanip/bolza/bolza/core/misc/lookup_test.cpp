

#include <gtest/gtest.h>
#include <bolza/core/misc/lookup.hpp>

using namespace bolza;
using namespace lookup;

TEST(testLookup, findIndexInTimeArray) {
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(FindIndexInTimeArray(timeArray, -2.0), 0);
  ASSERT_EQ(FindIndexInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(FindIndexInTimeArray(timeArray, 0.0), 1);
  ASSERT_EQ(FindIndexInTimeArray(timeArray, 2.0), 1);
  ASSERT_EQ(FindIndexInTimeArray(timeArray, 2.5), 2);
  ASSERT_EQ(FindIndexInTimeArray(timeArray, 3.0), 2);
  ASSERT_EQ(FindIndexInTimeArray(timeArray, 4.0), 3);

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(FindIndexInTimeArray(timeArrayRepeated, 1.9), 1);
  ASSERT_EQ(FindIndexInTimeArray(timeArrayRepeated, 2.0), 1);
  ASSERT_EQ(FindIndexInTimeArray(timeArrayRepeated, 2.1), 4);

  // Single Time
  std::vector<double> timeArraySingle{1.0};
  ASSERT_EQ(FindIndexInTimeArray(timeArraySingle, 0.0), 0);
  ASSERT_EQ(FindIndexInTimeArray(timeArraySingle, 1.0), 0);
  ASSERT_EQ(FindIndexInTimeArray(timeArraySingle, 2.0), 1);

  // empty time
  std::vector<double> timeArrayEmpty;
  ASSERT_EQ(FindIndexInTimeArray(timeArrayEmpty, -1.0), 0);
  ASSERT_EQ(FindIndexInTimeArray(timeArrayEmpty, 0.0), 0);
  ASSERT_EQ(FindIndexInTimeArray(timeArrayEmpty, 1.0), 0);
}

TEST(testLookup, findIndexInTimeArray_precision_lowNumbers) {
  std::vector<double> timeArray{0.0};
  double tQuery = timeArray.front();

  // Test next representable number in both directions
  double tQueryMinus = std::nextafter(tQuery, std::numeric_limits<double>::lowest());
  double tQueryPlus = std::nextafter(tQuery, std::numeric_limits<double>::max());

  // Make sure the tested values have the correct ordering
  ASSERT_TRUE(tQueryMinus <= timeArray.front());
  ASSERT_TRUE(tQuery <= timeArray.front());
  ASSERT_TRUE(tQueryPlus > timeArray.front());

  // Test lookup
  ASSERT_EQ(FindIndexInTimeArray(timeArray, tQuery), 0);
  ASSERT_EQ(FindIndexInTimeArray(timeArray, tQueryMinus), 0);
  ASSERT_EQ(FindIndexInTimeArray(timeArray, tQueryPlus), 1);
}

TEST(testLookup, findIndexInTimeArray_precision_highNumbers) {
  std::vector<double> timeArray{10000.0};
  double tQuery = timeArray.front();

  // Test next representable number in both directions
  double tQueryMinus = std::nextafter(tQuery, std::numeric_limits<double>::lowest());
  double tQueryPlus = std::nextafter(tQuery, std::numeric_limits<double>::max());

  // Make sure the tested values have the correct ordering
  ASSERT_TRUE(tQueryMinus <= timeArray.front());
  ASSERT_TRUE(tQuery <= timeArray.front());
  ASSERT_TRUE(tQueryPlus > timeArray.front());

  // Test lookup
  ASSERT_EQ(FindIndexInTimeArray(timeArray, tQuery), 0);
  ASSERT_EQ(FindIndexInTimeArray(timeArray, tQueryMinus), 0);
  ASSERT_EQ(FindIndexInTimeArray(timeArray, tQueryPlus), 1);
}

TEST(testLookup, findIntervalInTimeArray) {
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(FindIntervalInTimeArray(timeArray, -2.0), -1);
  ASSERT_EQ(FindIntervalInTimeArray(timeArray, -1.0), -1);
  ASSERT_EQ(FindIntervalInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(FindIntervalInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(FindIntervalInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(FindIntervalInTimeArray(timeArray, 3.0), 1);
  ASSERT_EQ(FindIntervalInTimeArray(timeArray, 4.0), 2);

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(FindIntervalInTimeArray(timeArrayRepeated, 1.9), 0);
  ASSERT_EQ(FindIntervalInTimeArray(timeArrayRepeated, 2.0), 0);
  ASSERT_EQ(FindIntervalInTimeArray(timeArrayRepeated, 2.1), 3);

  // Single Time
  std::vector<double> timeArraySingle{1.0};
  ASSERT_EQ(FindIntervalInTimeArray(timeArraySingle, 0.0), -1);
  ASSERT_EQ(FindIntervalInTimeArray(timeArraySingle, 1.0), -1);
  ASSERT_EQ(FindIntervalInTimeArray(timeArraySingle, 2.0), 0);

  // empty time
  std::vector<double> timeArrayEmpty;
  ASSERT_EQ(FindIntervalInTimeArray(timeArrayEmpty, -1.0), 0);
  ASSERT_EQ(FindIntervalInTimeArray(timeArrayEmpty, 0.0), 0);
  ASSERT_EQ(FindIntervalInTimeArray(timeArrayEmpty, 1.0), 0);
}

TEST(testLookup, findActiveIntervalInTimeArray) {
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArray, -2.0), -1);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArray, 3.0), 1);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArray, 4.0), 2);

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArrayRepeated, 1.9), 0);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArrayRepeated, 2.0), 0);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArrayRepeated, 2.1), 3);

  // Single Time
  std::vector<double> timeArraySingle{1.0};
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArraySingle, 0.0), -1);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArraySingle, 1.0), 0);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArraySingle, 2.0), 0);

  // empty time
  std::vector<double> timeArrayEmpty;
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArrayEmpty, -1.0), 0);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArrayEmpty, 0.0), 0);
  ASSERT_EQ(FindActiveIntervalInTimeArray(timeArrayEmpty, 1.0), 0);
}

TEST(testLookup, findBoundedActiveIntervalInTimeArray) {
  // Normal case
  std::vector<double> timeArray{-1.0, 2.0, 3.0};
  ASSERT_ANY_THROW(FindBoundedActiveIntervalInTimeArray(timeArray, -2.0));  // throws on -1
  ASSERT_EQ(FindBoundedActiveIntervalInTimeArray(timeArray, -1.0), 0);
  ASSERT_EQ(FindBoundedActiveIntervalInTimeArray(timeArray, 0.0), 0);
  ASSERT_EQ(FindBoundedActiveIntervalInTimeArray(timeArray, 2.0), 0);
  ASSERT_EQ(FindBoundedActiveIntervalInTimeArray(timeArray, 2.5), 1);
  ASSERT_EQ(FindBoundedActiveIntervalInTimeArray(timeArray, 3.0), 1);
  ASSERT_ANY_THROW(FindBoundedActiveIntervalInTimeArray(timeArray, 4.0));  // throws on 2

  // With repetitions
  std::vector<double> timeArrayRepeated{-1.0, 2.0, 2.0, 2.0, 3.0};
  ASSERT_EQ(FindBoundedActiveIntervalInTimeArray(timeArrayRepeated, 1.9), 0);
  ASSERT_EQ(FindBoundedActiveIntervalInTimeArray(timeArrayRepeated, 2.0), 0);
  ASSERT_EQ(FindBoundedActiveIntervalInTimeArray(timeArrayRepeated, 2.1), 3);

  // Single Time -> always throws
  std::vector<double> timeArraySingle{1.0};
  ASSERT_ANY_THROW(FindBoundedActiveIntervalInTimeArray(timeArraySingle, 0.0));
  ASSERT_ANY_THROW(FindBoundedActiveIntervalInTimeArray(timeArraySingle, 1.0));
  ASSERT_ANY_THROW(FindBoundedActiveIntervalInTimeArray(timeArraySingle, 2.0));

  // empty time -> always throws
  std::vector<double> timeArrayEmpty;
  ASSERT_ANY_THROW(FindBoundedActiveIntervalInTimeArray(timeArrayEmpty, -1.0));
  ASSERT_ANY_THROW(FindBoundedActiveIntervalInTimeArray(timeArrayEmpty, 0.0));
  ASSERT_ANY_THROW(FindBoundedActiveIntervalInTimeArray(timeArrayEmpty, 1.0));
}
