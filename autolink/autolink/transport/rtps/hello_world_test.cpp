#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "gtest/gtest.h"

#include "autolink/proto/qos_profile.pb.h"

#include "autolink/common/log.hpp"
#include "autolink/transport/message/message_info.hpp"
#include "autolink/transport/qos/qos_profile_conf.hpp"
#include "autolink/transport/rtps/participant.hpp"
#include "autolink/transport/rtps/publisher.hpp"
#include "autolink/transport/rtps/subscriber.hpp"
#include "autolink/transport/rtps/underlay_message.hpp"

namespace autolink {
namespace transport {
namespace {

TEST(HelloWorldTest, PublishAndReceiveOnce) {
    const std::string channel_name = "hello_world_topic_test";
    auto participant =
        std::make_shared<Participant>("hello_world_participant_test", 0);
    ASSERT_TRUE(participant->Init());

    autolink::proto::QosProfile qos = QosProfileConf::QOS_PROFILE_DEFAULT;

    std::mutex mutex;
    std::condition_variable cv;
    bool received = false;

    auto subscriber_callback = [&](const std::shared_ptr<std::string>& msg,
                                   uint64_t, const MessageInfo& info) {
        std::lock_guard<std::mutex> lock(mutex);
        received = true;
        AINFO << "Received message: \"" << *msg << "\" (seq: " << info.seq_num()
              << ")";
        cv.notify_all();
    };

    auto subscriber =
        participant->CreateSubscriber(channel_name, qos, subscriber_callback);
    ASSERT_NE(subscriber, nullptr);

    auto publisher = participant->CreatePublisher(channel_name, qos);
    ASSERT_NE(publisher, nullptr);

    UnderlayMessage message;
    message.data("Hello Fast DDS Test!");
    message.seq(1);
    message.timestamp(42u);
    ASSERT_TRUE(publisher->Write(message));

    std::unique_lock<std::mutex> lock(mutex);
    EXPECT_TRUE(
        cv.wait_for(lock, std::chrono::seconds(1), [&] { return received; }));

    publisher->Shutdown();
    participant->Shutdown();
}

}  // namespace
}  // namespace transport
}  // namespace autolink
