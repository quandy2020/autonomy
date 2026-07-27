/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 ****************************************************************************/

#pragma once

#ifdef AUTOVIZ_USE_OGRE

#include <gtest/gtest.h>

namespace autoviz {
namespace rendering {
namespace test {

/** gtest global environment: one offscreen GL context + RenderSystem per process. */
class OgreGtestEnvironment : public ::testing::Environment {
 public:
  void SetUp() override;
  void TearDown() override;
};

/** Per-fixture accessor (RenderSystem already initialized globally). */
class OgreTestingEnvironment {
 public:
  void setUpOgreTestEnvironment(bool debug = false);
  int glslVersion() const;
  ~OgreTestingEnvironment();
};

}  // namespace test
}  // namespace rendering
}  // namespace autoviz

#endif
