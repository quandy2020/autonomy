/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <gtest/gtest.h>

#ifdef AUTOVIZ_USE_OGRE
#include "tests/ogre_testing_environment.hpp"
#endif

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
#ifdef AUTOVIZ_USE_OGRE
  ::testing::AddGlobalTestEnvironment(
      new autoviz::rendering::test::OgreGtestEnvironment());
#endif
  return RUN_ALL_TESTS();
}
