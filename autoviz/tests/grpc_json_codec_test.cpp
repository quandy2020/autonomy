/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <gtest/gtest.h>

#include <google/protobuf/compiler/importer.h>
#include <google/protobuf/descriptor.h>

#include <memory>
#include <string>

#include "autoviz/integration/grpc/grpc_json_codec.hpp"

using namespace autoviz::integration::grpc_client;

namespace {

class CollectingErrorCollector
    : public google::protobuf::compiler::MultiFileErrorCollector {
 public:
  void RecordError(absl::string_view filename, int line, int column,
                   absl::string_view message) override {
    if (!text.empty()) {
      text += '\n';
    }
    text += std::string(filename);
    text += ':';
    text += std::to_string(line);
    text += ':';
    text += std::to_string(column);
    text += ": ";
    text += std::string(message);
  }

  std::string text;
};

class GrpcJsonCodecTest : public ::testing::Test {
 protected:
  void SetUp() override {
    source_tree_.MapPath("", AUTOVIZ_TEST_FIXTURES_DIR);
    importer_ = std::make_unique<google::protobuf::compiler::Importer>(
        &source_tree_, &errors_);
    const google::protobuf::FileDescriptor* file =
        importer_->Import("grpc_hello.proto");
    ASSERT_NE(file, nullptr) << errors_.text;
    desc_ = importer_->pool()->FindMessageTypeByName(
        "autoviz.test.HelloRequest");
    ASSERT_NE(desc_, nullptr);
  }

  google::protobuf::compiler::DiskSourceTree source_tree_;
  CollectingErrorCollector errors_;
  std::unique_ptr<google::protobuf::compiler::Importer> importer_;
  const google::protobuf::Descriptor* desc_ = nullptr;
};

}  // namespace

TEST_F(GrpcJsonCodecTest, RoundTripHelloRequest) {
  std::string err;
  auto msg = JsonToDynamicMessage(R"({"greeting":"Cooper"})", desc_, &err);
  ASSERT_NE(msg, nullptr) << err;
  std::string json;
  ASSERT_TRUE(DynamicMessageToJson(*msg, /*include_defaults=*/false, &json, &err))
      << err;
  EXPECT_NE(json.find("Cooper"), std::string::npos);
}

TEST_F(GrpcJsonCodecTest, InvalidJsonFails) {
  std::string err;
  auto msg = JsonToDynamicMessage("{", desc_, &err);
  EXPECT_EQ(msg, nullptr);
  EXPECT_FALSE(err.empty());
}
