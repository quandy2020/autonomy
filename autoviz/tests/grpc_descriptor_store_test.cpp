/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <gtest/gtest.h>

#include <google/protobuf/descriptor.pb.h>

#include <string>

#include "autoviz/integration/grpc/grpc_descriptor_store.hpp"

using namespace autoviz::integration::grpc_client;

namespace {

std::string FixtureProtoPath() {
  return std::string(AUTOVIZ_TEST_FIXTURES_DIR) + "/grpc_hello.proto";
}

}  // namespace

TEST(GrpcDescriptorStore, LoadHelloProtoListsMethods) {
  GrpcDescriptorStore store;
  std::string err;
  ASSERT_TRUE(store.loadProtoFile(FixtureProtoPath(), {AUTOVIZ_TEST_FIXTURES_DIR},
                                  &err))
      << err;
  const auto methods = store.listMethods();
  ASSERT_GE(methods.size(), 4u);
  const auto* say = store.findMethod("autoviz.test.HelloService.SayHello");
  ASSERT_NE(say, nullptr);
  EXPECT_EQ(store.methodType(say), MethodType::kUnary);
  EXPECT_EQ(store.methodType(
                store.findMethod("autoviz.test.HelloService.BidiHello")),
            MethodType::kBidiStreaming);
}

TEST(GrpcDescriptorStore, StreamingMethodTypes) {
  GrpcDescriptorStore store;
  std::string err;
  ASSERT_TRUE(store.loadProtoFile(FixtureProtoPath(), {AUTOVIZ_TEST_FIXTURES_DIR},
                                  &err))
      << err;

  EXPECT_EQ(store.methodType(store.findMethod(
                "autoviz.test.HelloService.LotsOfGreetings")),
            MethodType::kClientStreaming);
  EXPECT_EQ(store.methodType(
                store.findMethod("autoviz.test.HelloService.LotsOfReplies")),
            MethodType::kServerStreaming);
}

TEST(GrpcDescriptorStore, MethodTypeNullIsSafe) {
  GrpcDescriptorStore store;
  EXPECT_EQ(store.methodType(nullptr), MethodType::kUnary);
}

TEST(GrpcDescriptorStore, LoadFromFileDescriptorSet) {
  GrpcDescriptorStore seed;
  std::string err;
  ASSERT_TRUE(seed.loadProtoFile(FixtureProtoPath(), {AUTOVIZ_TEST_FIXTURES_DIR},
                                 &err))
      << err;

  const auto* say = seed.findMethod("autoviz.test.HelloService.SayHello");
  ASSERT_NE(say, nullptr);
  google::protobuf::FileDescriptorSet fds;
  say->file()->CopyTo(fds.add_file());

  GrpcDescriptorStore store;
  ASSERT_TRUE(store.loadFromFileDescriptorSet(fds, &err)) << err;
  ASSERT_NE(store.findMethod("autoviz.test.HelloService.SayHello"), nullptr);
  EXPECT_EQ(store.listMethods().size(), 4u);
}

TEST(GrpcDescriptorStore, ExampleJsonIsObject) {
  GrpcDescriptorStore store;
  std::string err;
  ASSERT_TRUE(store.loadProtoFile(FixtureProtoPath(), {AUTOVIZ_TEST_FIXTURES_DIR},
                                  &err))
      << err;
  const auto* say = store.findMethod("autoviz.test.HelloService.SayHello");
  ASSERT_NE(say, nullptr);
  const std::string json = store.exampleJson(say->input_type());
  EXPECT_FALSE(json.empty());
  EXPECT_EQ(json.front(), '{');
  EXPECT_EQ(json.back(), '}');
}

#ifdef AUTOVIZ_AUTOMSGS_PROTO_ROOT
TEST(GrpcDescriptorStore, LoadAutomsgsRpcsSoft) {
  GrpcDescriptorStore store;
  std::string err;
  if (!store.loadAutomsgsRpcs(&err)) {
    GTEST_SKIP() << "loadAutomsgsRpcs unavailable: " << err;
  }
  EXPECT_FALSE(store.listMethods().empty());
}
#endif
