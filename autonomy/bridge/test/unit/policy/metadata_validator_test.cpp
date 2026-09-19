/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/metadata/metadata_keys.hpp"
#include "autonomy/bridge/policy/metadata/metadata_validator.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace policy {
namespace {

TEST(MetadataValidatorTest, RequireOffIgnoresMissing) {
    MetadataValidator validator(/*require_robot_id=*/false);
    EXPECT_TRUE(validator.Validate({}).ok);
}

TEST(MetadataValidatorTest, RequireOnRejectsMissing) {
    MetadataValidator validator(/*require_robot_id=*/true);
    const auto result = validator.Validate({});
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.detail, "missing x-robot-id");
}

TEST(MetadataValidatorTest, RequireOnAcceptsPresent) {
    MetadataValidator validator(/*require_robot_id=*/true);
    EXPECT_TRUE(validator
                    .Validate({{kMetadataKeyRobotId, "robot-1"}})
                    .ok);
}

TEST(MetadataValidatorTest, ExtraKeysIgnored) {
    MetadataValidator validator(/*require_robot_id=*/true);
    EXPECT_TRUE(validator
                    .Validate({{kMetadataKeyRobotId, "r"},
                               {"x-extra", "v"},
                               {kMetadataKeyRequestId, "req"}})
                    .ok);
}

TEST(MetadataValidatorTest, EmptyRobotIdRejected) {
    MetadataValidator validator(/*require_robot_id=*/true);
    EXPECT_FALSE(validator.Validate({{kMetadataKeyRobotId, ""}}).ok);
}

}  // namespace
}  // namespace policy
}  // namespace bridge
}  // namespace autonomy
