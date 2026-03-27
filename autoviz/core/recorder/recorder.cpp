#include "autonomy/autoviz/core/recorder/recorder.hpp"

#if defined(AUTOVIZ_HAS_MCAP)
#include <mcap/writer.hpp>
#endif

namespace autoviz {
namespace recorder {

#if defined(AUTOVIZ_HAS_MCAP)
class Recorder::Impl {
 public:
  ::mcap::McapWriter writer;
};
#endif

Recorder::Recorder(const config::McapConfig& cfg) : cfg_(cfg) {
#if defined(AUTOVIZ_HAS_MCAP)
  impl_ = std::make_unique<Impl>();
#endif
}

Recorder::~Recorder() {
  Stop();
}

bool Recorder::Start() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (started_) {
    return true;
  }
  if (!cfg_.enabled) {
    return false;
  }
#if defined(AUTOVIZ_HAS_MCAP)
  ::mcap::McapWriterOptions options("");
  const auto status = impl_->writer.open(cfg_.output_path, options);
  if (!status.ok()) {
    return false;
  }
#endif
  started_ = true;
  return true;
}

void Recorder::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_) {
    return;
  }
#if defined(AUTOVIZ_HAS_MCAP)
  impl_->writer.close();
#endif
  started_ = false;
}

bool Recorder::IsEnabled() const {
  return cfg_.enabled;
}

bool Recorder::AddTopic(const std::string& topic,
                             const std::string& msg_type,
                             const std::string& proto_desc) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_) {
    return false;
  }

  auto& state = topic_states_[topic];
  if (state.registered) {
    return true;
  }

  state.schema_id = next_schema_id_++;
  state.channel_id = next_channel_id_++;

#if defined(AUTOVIZ_HAS_MCAP)
  // MCAP Schema for protobuf: name=full type, encoding="protobuf",
  // data=serialized descriptor string (provided by autolink).
  ::mcap::Schema schema(msg_type, "protobuf", proto_desc);
  schema.id = state.schema_id;  // MCAP uses schemaId references from channels.
  impl_->writer.addSchema(schema);

  ::mcap::Channel channel;
  channel.id = state.channel_id;
  channel.topic = topic;
  channel.messageEncoding = "protobuf";
  channel.schemaId = state.schema_id;
  impl_->writer.addChannel(channel);
#endif

  state.registered = true;
  return true;
}

bool Recorder::WriteSample(const std::string& topic,
                            const void* data,
                            std::size_t size,
                            std::uint64_t log_time_ns,
                            std::uint64_t publish_time_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!started_ || data == nullptr || size == 0) {
    return false;
  }

  const auto it = topic_states_.find(topic);
  if (it == topic_states_.end() || !it->second.registered) {
    return false;
  }

#if defined(AUTOVIZ_HAS_MCAP)
  ::mcap::Message message;
  message.channelId = it->second.channel_id;
  message.sequence = 0;
  message.logTime = log_time_ns;
  message.publishTime = publish_time_ns;
  message.data = reinterpret_cast<const std::byte*>(data);
  message.dataSize = static_cast<uint64_t>(size);
  if (!impl_->writer.write(message).ok()) {
    return false;
  }
  if (cfg_.flush_on_write) {
    impl_->writer.closeLastChunk();
  }
#else
  (void)log_time_ns;
  (void)publish_time_ns;
#endif

  return true;
}

}  // namespace recorder
}  // namespace autoviz

