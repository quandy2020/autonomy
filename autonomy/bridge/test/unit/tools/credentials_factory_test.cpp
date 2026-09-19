/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/bridge/tools/transport/credentials_factory.hpp"
#include "gtest/gtest.h"

namespace autonomy {
namespace bridge {
namespace tools {
namespace {

TEST(CredentialsFactoryTest, InsecureWhenSslDisabled) {
    proto::GrpcOptions options;
    options.set_enable_ssl_encryption(false);
    auto creds = CredentialsFactory::Create(options);
    ASSERT_NE(creds, nullptr);
}

TEST(CredentialsFactoryTest, FallsBackWhenTlsPathsMissing) {
    proto::GrpcOptions options;
    options.set_enable_ssl_encryption(true);
    options.set_tls_cert_path("/nonexistent/cert.pem");
    options.set_tls_key_path("/nonexistent/key.pem");
    auto creds = CredentialsFactory::Create(options);
    ASSERT_NE(creds, nullptr);
}

}  // namespace
}  // namespace tools
}  // namespace bridge
}  // namespace autonomy
