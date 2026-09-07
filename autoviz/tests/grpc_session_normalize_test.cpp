/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <gtest/gtest.h>

#include "autoviz/integration/grpc/grpc_session.hpp"

using autoviz::integration::grpc_client::NormalizeGrpcTarget;
using autoviz::integration::grpc_client::NormalizeMethodFullName;

TEST(GrpcSessionNormalize, StripsSchemesAndPath) {
  EXPECT_EQ(NormalizeGrpcTarget("grpc://localhost:50051"), "localhost:50051");
  EXPECT_EQ(NormalizeGrpcTarget("http://127.0.0.1:8080/"), "127.0.0.1:8080");
  EXPECT_EQ(NormalizeGrpcTarget("https://api.example.com:443/v1"),
            "api.example.com:443");
  EXPECT_EQ(NormalizeGrpcTarget("  host:1234  "), "host:1234");
  EXPECT_EQ(NormalizeGrpcTarget("localhost:50051"), "localhost:50051");
}

TEST(GrpcInvokerNormalize, DotAndSlashMethodNames) {
  EXPECT_EQ(NormalizeMethodFullName("pkg.Svc.Method"), "pkg.Svc.Method");
  EXPECT_EQ(NormalizeMethodFullName("pkg.Svc/Method"), "pkg.Svc.Method");
  EXPECT_EQ(NormalizeMethodFullName("/pkg.Svc/Method"), "pkg.Svc.Method");
}
