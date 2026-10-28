#include "atlas/common/config.hpp"

#include <gtest/gtest.h>

namespace autonomy::localization::atlas {
namespace {

TEST(ConfigTest, ReadsTypedValuesAndDefaults) {
    Config config;
    config.Set("features", std::int64_t{1000});
    config.Set("scale", 1.2);
    EXPECT_EQ(config.Get<std::int64_t>("features"), 1000);
    EXPECT_DOUBLE_EQ(config.Get<double>("features"), 1000.0);
    EXPECT_DOUBLE_EQ(config.GetOr<double>("missing", 2.0), 2.0);
    EXPECT_THROW(config.Get<std::string>("scale"), std::invalid_argument);
}

}  // namespace
}  // namespace autonomy::localization::atlas
