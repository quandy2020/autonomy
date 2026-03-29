/******************************************************************************
 * Copyright 2018 The Autolink Authors. All Rights Reserved.
 *****************************************************************************/

#include "autolink/autolink.hpp"
#include "examples.pb.h"

using autolink::examples::Driver;

int main(int argc, char* argv[]) {
    if (!autolink::Init(argv[0]))
        return 1;
    auto node = autolink::CreateNode("start_node", "/examples");
    auto server = node->CreateService<Driver, Driver>(
        "test_server", [](const std::shared_ptr<Driver>& request,
                          std::shared_ptr<Driver>& response) {
            AINFO << "server: i am driver server";
            static uint64_t id = 0;
            ++id;
            response->set_msg_id(id);
            response->set_timestamp(0);
        });
    auto client = node->CreateClient<Driver, Driver>("test_server");
    auto driver_msg = std::make_shared<Driver>();
    driver_msg->set_msg_id(0);
    driver_msg->set_timestamp(0);
    while (autolink::OK()) {
        auto res = client->SendRequest(driver_msg);
        if (res != nullptr) {
            AINFO << "client: response: " << res->ShortDebugString();
        } else {
            AINFO << "client: service may not ready.";
        }
        sleep(1);
    }
    autolink::WaitForShutdown();
    return 0;
}
