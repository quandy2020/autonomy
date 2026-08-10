/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <gtest/gtest.h>

#include "autoviz/display/tf_display_utils.hpp"

using autoviz::display::FilterTfFrameNames;
using autoviz::display::TfAgeVisualForTimeout;

TEST(TfDisplayUtils, FilterWhitelistBlacklist) {
  const std::vector<std::string> in{"map", "base_link", "laser", "camera"};
  auto out = FilterTfFrameNames(in, "laser|camera", "", nullptr);
  ASSERT_EQ(out.size(), 2u);
  EXPECT_EQ(out[0], "laser");
  EXPECT_EQ(out[1], "camera");

  out = FilterTfFrameNames(in, "", "map", nullptr);
  EXPECT_EQ(out.size(), 3u);

  std::string err;
  out = FilterTfFrameNames(in, "(", "", &err);
  EXPECT_EQ(out.size(), 4u);  // invalid whitelist → treat as empty
  EXPECT_FALSE(err.empty());
}

TEST(TfDisplayUtils, AgeTimeoutSegments) {
  auto v = TfAgeVisualForTimeout(1.0, 15.0, QColor(220, 60, 60));
  EXPECT_TRUE(v.visible);
  EXPECT_FLOAT_EQ(v.alpha, 1.f);
  EXPECT_EQ(v.color.red(), 220);

  v = TfAgeVisualForTimeout(6.0, 15.0, QColor(220, 60, 60));
  EXPECT_TRUE(v.visible);
  EXPECT_EQ(v.color.red(), 160);

  v = TfAgeVisualForTimeout(14.0, 15.0, QColor(220, 60, 60));
  EXPECT_TRUE(v.visible);
  EXPECT_LT(v.alpha, 1.f);

  v = TfAgeVisualForTimeout(16.0, 15.0, QColor(220, 60, 60));
  EXPECT_FALSE(v.visible);
}
