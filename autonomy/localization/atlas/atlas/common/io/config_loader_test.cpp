#include "atlas/common/io/config_loader.hpp"

#include <type_traits>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas {
namespace {

TEST(ConfigLoaderTest, PublicTypeIsAvailable) {
    EXPECT_TRUE(std::is_class_v<ConfigLoader>);
}

}  // namespace
}  // namespace autonomy::localization::atlas
