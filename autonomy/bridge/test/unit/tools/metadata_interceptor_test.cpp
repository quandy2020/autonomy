/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/policy/metadata/metadata_keys.hpp"
#include "autonomy/bridge/policy/metadata/metadata_validator.hpp"
#include "autonomy/bridge/tools/interceptors/interceptor_utils.hpp"
#include "gtest/gtest.h"

#include <map>

namespace autonomy {
namespace bridge {
namespace tools {
namespace interceptors {
namespace {

TEST(MetadataInterceptorLogicTest, FlattenLowercasesKeys) {
    std::multimap<::grpc::string_ref, ::grpc::string_ref> raw;
    raw.emplace("X-Robot-Id", "bot");
    raw.emplace("Authorization", "Bearer t");
    const auto flat = FlattenMetadata(&raw);
    EXPECT_EQ(LookupMetadata(flat, policy::kMetadataKeyRobotId), "bot");
    EXPECT_EQ(LookupMetadata(flat, policy::kMetadataKeyAuthorization),
              "Bearer t");
}

TEST(MetadataInterceptorLogicTest, ValidatorRejectPath) {
    policy::MetadataValidator validator(true);
    EXPECT_FALSE(validator.Validate({}).ok);
    EXPECT_TRUE(
        validator.Validate({{policy::kMetadataKeyRobotId, "r"}}).ok);
}

}  // namespace
}  // namespace interceptors
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
