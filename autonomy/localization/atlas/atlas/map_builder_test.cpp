#include "atlas/map_builder.hpp"

#include <utility>

#include <gtest/gtest.h>

namespace autonomy::localization::atlas {
namespace {

TEST(MapBuilderTest, RejectsMissingConfiguration) {
    SystemConfig config;
    config.settings_path = "/path/that/does/not/exist.yaml";
    config.vocabulary_path = "/path/that/does/not/exist.fbow";
    MapBuilder builder(std::move(config));
    EXPECT_FALSE(builder.Start());
    EXPECT_FALSE(builder.started());
}

}  // namespace
}  // namespace autonomy::localization::atlas
