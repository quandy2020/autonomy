#include "autonomy/autoviz/core/convert/message/convert_automsgs_message.hpp"

#if defined(AUTOVIZ_HAS_FOXGLOVE) && AUTOVIZ_HAS_FOXGLOVE && \
    defined(AUTOVIZ_FOXGLOVE_HAS_ENCODE) && AUTOVIZ_FOXGLOVE_HAS_ENCODE

#include "autonomy/autoviz/core/convert/message/convert_automsgs_body_3d.hpp"
#include "autonomy/autoviz/core/convert/encode/foxglove_protobuf_encode.hpp"
#include "autonomy/autoviz/core/convert/registry/automsgs_foxglove_registry.hpp"

#include "autolink/message/protobuf_factory.hpp"
#include "automsgs/msgs/std_msgs/string.pb.h"

#include <google/protobuf/text_format.h>

#include <foxglove/schemas.hpp>

#include <string>

namespace autoviz {
namespace converter {
namespace {

template <typename FoxgloveSchemaT>
void CopySchemaToString(const FoxgloveSchemaT& fs, FoxgloveConvertedMessage* out) {
  out->schema_name = fs.name;
  out->schema_encoding = fs.encoding;
  if (fs.data != nullptr && fs.data_len > 0) {
    out->schema_data.assign(reinterpret_cast<const char*>(fs.data), fs.data_len);
  } else {
    out->schema_data.clear();
  }
}

bool BuildFallbackLog(const std::string& proto_full_name, const void* serialized_data,
                      std::size_t serialized_size, FoxgloveConvertedMessage* out) {
  foxglove::schemas::Log log_msg;
  log_msg.level = foxglove::schemas::Log::LogLevel::INFO;
  log_msg.name = proto_full_name;

  if (proto_full_name == "automsgs.msgs.std_msgs.String") {
    automsgs::msgs::std_msgs::String s;
    if (!s.ParseFromArray(serialized_data, static_cast<int>(serialized_size))) {
      log_msg.message = "(parse automsgs.msgs.std_msgs.String failed)";
    } else {
      log_msg.message = s.data();
    }
  } else {
    google::protobuf::Message* m =
        autolink::message::ProtobufFactory::Instance()->GenerateMessageByType(proto_full_name);
    if (m == nullptr) {
      log_msg.message = "(ProtobufFactory cannot create message for type " + proto_full_name + ")";
    } else {
      if (!m->ParseFromArray(serialized_data, static_cast<int>(serialized_size))) {
        log_msg.message = "(parse failed for type " + proto_full_name + ")";
      } else {
        std::string text;
        google::protobuf::TextFormat::PrintToString(*m, &text);
        constexpr std::size_t kMax = 256 * 1024;
        if (text.size() > kMax) {
          text.resize(kMax);
          text += "\n... (truncated)";
        }
        log_msg.message = std::move(text);
      }
      delete m;
    }
  }

  const std::string payload = SerializeLog(log_msg);
  if (payload.empty()) {
    return false;
  }

  const auto fs = foxglove::schemas::Log::schema();
  CopySchemaToString(fs, out);
  out->payload = std::move(payload);
  out->ok = true;
  return true;
}

}  // namespace

bool ConvertMessageToFoxglove(const std::string& proto_full_name, const void* serialized_data,
                              const std::size_t serialized_size, FoxgloveConvertedMessage* out) {
  if (out == nullptr || serialized_data == nullptr || serialized_size == 0) {
    return false;
  }
  out->ok = false;
  out->payload.clear();
  out->schema_name.clear();
  out->schema_data.clear();

  const AutomsgsFoxgloveEntry* ent = FindFoxgloveRule(proto_full_name);
  if (ent == nullptr) {
    return false;
  }

  if (ent->strategy == AutomsgsFoxgloveStrategy::kPassthroughProtobuf) {
    return false;
  }

  if (TryConvertBodyToFoxglove(ent->strategy, proto_full_name, serialized_data, serialized_size, out)) {
    return out->ok;
  }

  switch (ent->strategy) {
    case AutomsgsFoxgloveStrategy::kConvertToImage:
    case AutomsgsFoxgloveStrategy::kConvertToLog:
      return BuildFallbackLog(proto_full_name, serialized_data, serialized_size, out);
    default:
      return BuildFallbackLog(proto_full_name, serialized_data, serialized_size, out);
  }
}

}  // namespace converter
}  // namespace autoviz

#else

namespace autoviz {
namespace converter {

bool ConvertMessageToFoxglove(const std::string&, const void*, std::size_t, FoxgloveConvertedMessage*) {
  return false;
}

}  // namespace converter
}  // namespace autoviz

#endif
