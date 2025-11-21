#include "autolink/transport/transport.hpp"

#include <unistd.h>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <typeinfo>

#include "gtest/gtest.h"

#include "autolink/common/global_data.hpp"
#include "autolink/common/init.hpp"
#include "autolink/message/message_traits.hpp"
#include "autolink/proto/unit_test.pb.h"

namespace autolink {
namespace transport {

using TransmitterPtr = std::shared_ptr<Transmitter<proto::UnitTest>>;

TEST(TransportShmTalkerTest, CanTransmitMessageOverShm) {
    std::string channel_name = "transport_shm";

    // Prepare transmitter role attributes.
    RoleAttributes transmitter_attr;
    transmitter_attr.set_channel_name(channel_name);
    transmitter_attr.set_channel_id(
        common::GlobalData::RegisterChannel(channel_name));
    transmitter_attr.set_host_ip("127.0.0.1");
    proto::UnitTest dummy_msg;
    transmitter_attr.set_message_type(dummy_msg.GetDescriptor()->full_name());
    transmitter_attr.mutable_qos_profile()->CopyFrom(
        QosProfileConf::QOS_PROFILE_DEFAULT);

    // Receiver attributes are used to enable the transmitter.
    RoleAttributes receiver_attr = transmitter_attr;

    TransmitterPtr transmitter =
        Transport::Instance()->CreateTransmitter<proto::UnitTest>(
            transmitter_attr, OptionalMode::SHM);
    ASSERT_NE(transmitter, nullptr);
    EXPECT_EQ(typeid(*transmitter), typeid(ShmTransmitter<proto::UnitTest>));

    // Enable SHM transmitter with receiver attributes.
    transmitter->Enable(receiver_attr);

    for (int i = 0; i < 10; ++i) {
        // Build and send a message.
        auto send_msg = std::make_shared<proto::UnitTest>();
        send_msg->set_class_name("TransportShmTalkerTest");
        send_msg->set_case_name("CanTransmitMessageOverShm");

        EXPECT_TRUE(transmitter->Transmit(send_msg));
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

}  // namespace transport
}  // namespace autolink

int main(int argc, char** argv) {
    // Set AUTOLINK_PATH to find the configuration file in Bazel test
    // environment Config file is at:
    // <target>.runfiles/_main/autolink/conf/autolink.pb.conf TEST_SRCDIR points
    // to: <target>.runfiles/_main (or just runfiles/) WorkRoot() returns
    // AUTOLINK_PATH, and GetAbsolutePath(WorkRoot(), "conf/autolink.pb.conf")
    // will look for AUTOLINK_PATH/conf/autolink.pb.conf
    if (getenv("AUTOLINK_PATH") == nullptr) {
        const char* test_srcdir = getenv("TEST_SRCDIR");
        const char* test_workspace = getenv("TEST_WORKSPACE");
        if (test_srcdir != nullptr) {
            std::string autolink_path;
            if (test_workspace != nullptr) {
                // TEST_SRCDIR points to runfiles root, TEST_WORKSPACE is
                // workspace name Config is at
                // TEST_SRCDIR/TEST_WORKSPACE/autolink/conf/autolink.pb.conf
                autolink_path = std::string(test_srcdir) + "/" +
                                std::string(test_workspace) + "/autolink";
            } else {
                // Fallback: assume TEST_SRCDIR points to workspace directory
                autolink_path = std::string(test_srcdir) + "/autolink";
            }
            setenv("AUTOLINK_PATH", autolink_path.c_str(), 1);
        }
    }

    testing::InitGoogleTest(&argc, argv);
    autolink::Init(argv[0]);
    autolink::transport::Transport::Instance();
    auto res = RUN_ALL_TESTS();
    autolink::transport::Transport::Instance()->Shutdown();
    return res;
}
