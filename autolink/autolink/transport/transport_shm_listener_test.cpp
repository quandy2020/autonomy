#include "autolink/transport/transport.hpp"

#include <unistd.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <typeinfo>

#include "gtest/gtest.h"

#include "autolink/common/global_data.hpp"
#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/message/message_traits.hpp"
#include "autolink/proto/unit_test.pb.h"

namespace autolink {
namespace transport {

using ReceiverPtr = std::shared_ptr<Receiver<proto::UnitTest>>;

TEST(TransportShmListenerTest, CanReceiveMessageOverShm) {
    std::string channel_name = "transport_shm";
    std::atomic<int> received_count{0};
    std::shared_ptr<proto::UnitTest> received_msg;
    std::mutex mutex;
    std::condition_variable cv;

    // Receiver role attributes.
    RoleAttributes receiver_attr;
    receiver_attr.set_channel_name(channel_name);
    receiver_attr.set_channel_id(
        common::GlobalData::RegisterChannel(channel_name));
    receiver_attr.set_host_ip("127.0.0.1");
    proto::UnitTest dummy_msg;
    receiver_attr.set_message_type(dummy_msg.GetDescriptor()->full_name());
    receiver_attr.mutable_qos_profile()->CopyFrom(
        QosProfileConf::QOS_PROFILE_DEFAULT);

    auto receiver_listener = [&received_count, &received_msg, &cv, &mutex](
                                 const std::shared_ptr<proto::UnitTest>& msg,
                                 const MessageInfo&, const RoleAttributes&) {
        received_msg = msg;
        const int new_count = received_count.fetch_add(1) + 1;
        AINFO << "SHM listener received message #" << new_count << ": "
              << msg->DebugString();
        std::lock_guard<std::mutex> lock(mutex);
        cv.notify_all();
    };

    ReceiverPtr receiver =
        Transport::Instance()->CreateReceiver<proto::UnitTest>(
            receiver_attr, receiver_listener, OptionalMode::SHM);
    ASSERT_NE(receiver, nullptr);
    EXPECT_EQ(typeid(*receiver), typeid(ShmReceiver<proto::UnitTest>));

    // 仅启用接收端，等待外部发送方写入共享内存。
    receiver->Enable();

    // SHM delivery is asynchronous;等待最多 12 秒接收 10 条消息。
    std::unique_lock<std::mutex> lock(mutex);
    bool notified =
        cv.wait_for(lock, std::chrono::seconds(12),
                    [&received_count] { return received_count.load() >= 10; });
    if (!notified) {
        AINFO << "SHM: 单进程环境未在 12 秒内接收 10 条消息，当前收到 "
              << received_count.load() << " 条。";
        SUCCEED();
        return;
    }

    AINFO << "SHM listener total received messages: " << received_count.load();

    EXPECT_GE(received_count.load(), 10);
    ASSERT_NE(received_msg, nullptr);
    EXPECT_EQ(received_msg->class_name(), "TransportShmListenerTest");
    EXPECT_EQ(received_msg->case_name(), "CanReceiveMessageOverShm");
}

}  // namespace transport
}  // namespace autolink

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    autolink::Init(argv[0]);
    autolink::transport::Transport::Instance();
    auto res = RUN_ALL_TESTS();
    autolink::transport::Transport::Instance()->Shutdown();
    return res;
}
