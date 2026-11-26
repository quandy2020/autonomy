// Copyright 2026 The Openbot Authors
#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include <unistd.h>

#include "autolink/autolink.hpp"
#include "autoviz/common/interaction/channel_reader_registry.hpp"

int main(int argc, char** argv) {
  (void)argc;
  if (!autolink::Init(argv[0])) return 1;
  int failures = 0;
  auto check = [&](bool ok, const char* what) {
    if (!ok) { std::fprintf(stderr, "FAIL: %s\n", what); ++failures; }
  };
  {
    auto node = autolink::CreateNode("autoviz_registry_test_" + std::to_string(getpid()));
    auto& registry = autoviz::integration::ChannelReaderRegistry::instance();
    registry.setNode(node);
    const std::string channel = "/autoviz_registry_test_" + std::to_string(getpid());
    std::array<std::atomic<int>, 8> received{};
    std::array<decltype(registry.subscribe({}, {})), 8> ids{};
    std::array<std::thread, 8> threads;
    for (std::size_t i = 0; i < threads.size(); ++i) {
      threads[i] = std::thread([&, i] {
        ids[i] = registry.subscribe(i % 2 ? channel : channel.substr(1),
            [&, i](const std::string&) { ++received[i]; });
      });
    }
    for (auto& t : threads) t.join();
    for (auto id : ids) check(id != 0, "concurrent alias subscription succeeds");
    check(node->GetReader<autolink::message::RawMessage>(channel) != nullptr,
          "one canonical reader exists");
    check(node->GetReader<autolink::message::RawMessage>(channel.substr(1)) == nullptr,
          "no duplicate reader for slash alias");
    auto writer = node->CreateWriter<autolink::message::RawMessage>(channel);
    check(writer != nullptr, "writer created");
    for (int n = 0; writer && n < 40 && received[0] == 0; ++n) {
      writer->Write(std::make_shared<autolink::message::RawMessage>("sample"));
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
    for (const auto& count : received) check(count > 0, "all subscribers receive messages");
    for (auto id : ids) registry.unsubscribe(id);
    check(node->GetReader<autolink::message::RawMessage>(channel) == nullptr,
          "last unsubscribe releases node reader");
    std::atomic<int> subscribe_failures{0};
    for (auto& t : threads) {
      t = std::thread([&] {
        for (int n = 0; n < 20; ++n) {
          auto id = registry.subscribe(channel.substr(1), [](const auto&) {});
          if (id == 0) ++subscribe_failures;
          registry.unsubscribe(id);
        }
      });
    }
    for (auto& t : threads) t.join();
    check(subscribe_failures == 0, "repeated concurrent enable/disable has no failures");
    const auto id = registry.subscribe(channel, [](const auto&) {});
    check(id != 0, "can subscribe after teardown");
    registry.setNode(nullptr);
    check(node->GetReader<autolink::message::RawMessage>(channel) == nullptr,
          "changing node releases old readers");
  }
  autolink::Clear();
  if (!failures) std::puts("PASS alias fanout, concurrent subscriptions and reader teardown");
  return failures ? 1 : 0;
}
