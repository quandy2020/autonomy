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

TEST(GrpcDescriptorStore, LoadFromFileDescriptorSetOutOfOrderDeps) {
  using google::protobuf::FieldDescriptorProto;
  using google::protobuf::FileDescriptorProto;

  FileDescriptorProto base;
  base.set_name("order_base.proto");
  base.set_package("autoviz.test.order");
  base.set_syntax("proto3");
  auto* base_msg = base.add_message_type();
  base_msg->set_name("BaseMsg");
  auto* name_field = base_msg->add_field();
  name_field->set_name("name");
  name_field->set_number(1);
  name_field->set_label(FieldDescriptorProto::LABEL_OPTIONAL);
  name_field->set_type(FieldDescriptorProto::TYPE_STRING);

  FileDescriptorProto dependent;
  dependent.set_name("order_dependent.proto");
  dependent.set_package("autoviz.test.order");
  dependent.set_syntax("proto3");
  dependent.add_dependency("order_base.proto");
  auto* dep_msg = dependent.add_message_type();
  dep_msg->set_name("DepMsg");
  auto* nested = dep_msg->add_field();
  nested->set_name("nested");
  nested->set_number(1);
  nested->set_label(FieldDescriptorProto::LABEL_OPTIONAL);
  nested->set_type(FieldDescriptorProto::TYPE_MESSAGE);
  nested->set_type_name(".autoviz.test.order.BaseMsg");
  auto* service = dependent.add_service();
  service->set_name("OrderService");
  auto* rpc = service->add_method();
  rpc->set_name("Do");
  rpc->set_input_type(".autoviz.test.order.DepMsg");
  rpc->set_output_type(".autoviz.test.order.BaseMsg");

  google::protobuf::FileDescriptorSet fds;
  // Dependent file first — must still resolve via multi-pass BuildFile.
  *fds.add_file() = dependent;
  *fds.add_file() = base;

  GrpcDescriptorStore store;
  std::string err;
  ASSERT_TRUE(store.loadFromFileDescriptorSet(fds, &err)) << err;
  const auto* found = store.findMethod("autoviz.test.order.OrderService.Do");
  ASSERT_NE(found, nullptr);
  EXPECT_EQ(store.methodType(found), MethodType::kUnary);
  EXPECT_NE(store.exampleJson(found->input_type()).find('{'),
            std::string::npos);
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
