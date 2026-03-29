/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <getopt.h>
#include <signal.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "autolink/autolink.hpp"
#include "autolink/init.hpp"
#include "autolink/message/protobuf_factory.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/proto/role_attributes.pb.h"
#include "autolink/service_discovery/specific_manager/channel_manager.hpp"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autolink/state.hpp"
#include "autolink/time/time.hpp"

namespace {

constexpr int kDefaultBwWindowSize = 100;
constexpr int kDefaultHzWindowSize = 50000;
constexpr int kMaxWindowSize = 50000;

void PrintUsage() {
    std::cout
        << "autolink_channel is a command-line tool for printing information "
           "about CyberRT Channels.\n\n"
        << "Commands:\n"
        << "\tautolink_channel list\tlist active channels\n"
        << "\tautolink_channel info\tprint information about active channel\n"
        << "\tautolink_channel echo\tprint messages to screen\n"
        << "\tautolink_channel hz\tdisplay publishing rate of channel\n"
        << "\tautolink_channel bw\tdisplay bandwidth used by channel\n"
        << "\tautolink_channel type\tprint channel type\n\n"
        << "Type autolink_channel <command> -h for more detailed usage.\n";
}

// -----------------------------------------------------------------------------
// list: list active channels (sorted)
// -----------------------------------------------------------------------------
void CmdList() {
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(2);
    std::vector<std::string> channels;
    topology->channel_manager()->GetChannelNames(&channels);
    std::sort(channels.begin(), channels.end());
    std::cout << "The number of channels is: " << channels.size() << std::endl;
    for (const auto& ch : channels) {
        std::cout << ch << std::endl;
    }
}

// -----------------------------------------------------------------------------
// type: print channel message type
// -----------------------------------------------------------------------------
void PrintChannelType(const std::string& channel_name) {
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(2);
    std::string msg_type;
    topology->channel_manager()->GetMsgType(channel_name, &msg_type);
    std::cout << channel_name << " type is [ " << msg_type << " ]" << std::endl;
}

// -----------------------------------------------------------------------------
// info: print role attributes (roleid, hostname, processid, nodename, msgtype)
// -----------------------------------------------------------------------------
void PrintRole(const autolink::proto::RoleAttributes& attr) {
    std::cout << "\troleid\t\t" << attr.id() << std::endl;
    std::cout << "\thostname\t" << attr.host_name() << std::endl;
    std::cout << "\tprocessid\t" << attr.process_id() << std::endl;
    std::cout << "\tnodename\t" << attr.node_name() << std::endl;
    std::cout << "\tmsgtype\t\t" << attr.message_type() << std::endl;
}

void CmdInfo(const std::string& channel_name, bool all_channels) {
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(1);
    std::vector<autolink::proto::RoleAttributes> writers, readers;
    topology->channel_manager()->GetWriters(&writers);
    topology->channel_manager()->GetReaders(&readers);

    std::map<std::string, std::vector<autolink::proto::RoleAttributes>> info;
    for (auto& attr : writers) {
        info[attr.channel_name()].push_back(attr);
    }
    for (auto& attr : readers) {
        info[attr.channel_name()].push_back(attr);
    }

    if (info.empty()) {
        std::cout << "channelsinfo dict is null" << std::endl;
        return;
    }

    if (!channel_name.empty()) {
        auto it = info.find(channel_name);
        if (it != info.end()) {
            std::cout << channel_name << std::endl;
            for (const auto& attr : it->second) {
                PrintRole(attr);
            }
        }
        return;
    }

    std::vector<std::string> channels;
    for (const auto& p : info) {
        channels.push_back(p.first);
    }
    std::sort(channels.begin(), channels.end());
    std::cout << "The number of channels is: " << channels.size() << std::endl;
    for (const auto& ch : channels) {
        std::cout << ch << std::endl;
        for (const auto& attr : info[ch]) {
            PrintRole(attr);
        }
    }
}

// -----------------------------------------------------------------------------
// echo: subscribe and print messages as debug string by msg type
// -----------------------------------------------------------------------------
std::string GetDebugStringRawMsg(const std::string& msg_type,
                                 const std::string& rawmsgdata) {
    if (msg_type.empty() || rawmsgdata.empty()) {
        return "";
    }
    auto* factory = autolink::message::ProtobufFactory::Instance();
    google::protobuf::Message* msg = factory->GenerateMessageByType(msg_type);
    if (!msg) {
        return "";
    }
    std::string result;
    if (msg->ParseFromString(rawmsgdata)) {
        result = msg->DebugString();
    }
    delete msg;
    return result;
}

void CmdEcho(const std::string& channel_name) {
    auto node = autolink::CreateNode("listener_node_echo");
    if (!node) {
        std::cerr << "Failed to create node" << std::endl;
        return;
    }
    std::string msg_type;
    auto* topology = autolink::service_discovery::TopologyManager::Instance();
    sleep(2);
    topology->channel_manager()->GetMsgType(channel_name, &msg_type);

    auto callback =
        [channel_name,
         msg_type](const std::shared_ptr<const autolink::message::RawMessage>&
                       raw_msg) {
            std::string data = raw_msg ? raw_msg->message : "";
            std::string debug = GetDebugStringRawMsg(msg_type, data);
            if (!debug.empty()) {
                std::cout << debug << std::endl;
            }
        };

    auto reader = node->CreateReader<autolink::message::RawMessage>(
        channel_name, callback);
    if (!reader) {
        std::cerr << "Failed to create reader for channel: " << channel_name
                  << std::endl;
        return;
    }
    std::cout << "reader to [" << channel_name << "]" << std::endl;
    while (!autolink::IsShutdown()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// -----------------------------------------------------------------------------
// bw: bandwidth stats (window of message sizes, print rate every second)
// -----------------------------------------------------------------------------
class ChannelBw
{
public:
    explicit ChannelBw(int window_size)
        : window_size_(window_size <= 0 || window_size > kMaxWindowSize
                           ? kDefaultBwWindowSize
                           : window_size) {
        std::cout << "bw window_size: " << window_size_ << std::endl;
    }

    void OnData(const std::string& rawdata) {
        std::lock_guard<std::mutex> lock(mutex_);
        double t = autolink::Time::Now().ToSecond();
        times_.push_back(t);
        sizes_.push_back(static_cast<int>(rawdata.size()));
        while (static_cast<int>(times_.size()) > window_size_) {
            times_.erase(times_.begin());
            sizes_.erase(sizes_.begin());
        }
    }

    void Print() {
        std::vector<double> times;
        std::vector<int> sizes;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (times_.size() < 2)
                return;
            times = times_;
            sizes = sizes_;
        }
        double tn = autolink::Time::Now().ToSecond();
        double t0 = times.front();
        int total = 0;
        for (int s : sizes)
            total += s;
        int n = static_cast<int>(sizes.size());
        double bytes_per_s = (tn > t0) ? (total / (tn - t0)) : 0;
        double mean = (n > 0) ? static_cast<double>(total) / n : 0;
        int max_s = *std::max_element(sizes.begin(), sizes.end());
        int min_s = *std::min_element(sizes.begin(), sizes.end());

        auto fmt = [](double v, double scale, const char* unit) -> std::string {
            char buf[64];
            snprintf(buf, sizeof(buf), "%.2f%s", v / scale, unit);
            return buf;
        };
        std::string bw_str, mean_str, min_str, max_str;
        if (bytes_per_s < 1000) {
            bw_str = fmt(bytes_per_s, 1, "B");
            mean_str = fmt(mean, 1, "B");
            min_str = fmt(static_cast<double>(min_s), 1, "B");
            max_str = fmt(static_cast<double>(max_s), 1, "B");
        } else if (bytes_per_s < 1000000) {
            bw_str = fmt(bytes_per_s, 1000, "KB");
            mean_str = fmt(mean, 1000, "KB");
            min_str = fmt(static_cast<double>(min_s), 1000, "KB");
            max_str = fmt(static_cast<double>(max_s), 1000, "KB");
        } else {
            bw_str = fmt(bytes_per_s, 1000000, "MB");
            mean_str = fmt(mean, 1000000, "MB");
            min_str = fmt(static_cast<double>(min_s), 1000000, "MB");
            max_str = fmt(static_cast<double>(max_s), 1000000, "MB");
        }
        std::cout << "average: " << bw_str << "/s\n\tmean: " << mean_str
                  << " min: " << min_str << " max: " << max_str
                  << " window: " << n << std::endl;
    }

private:
    int window_size_;
    std::mutex mutex_;
    std::vector<double> times_;
    std::vector<int> sizes_;
};

void CmdBw(const std::string& channel_name, int window_size) {
    auto bw = std::make_shared<ChannelBw>(window_size);
    auto node = autolink::CreateNode("listener_node_bw");
    if (!node) {
        std::cerr << "Failed to create node" << std::endl;
        return;
    }
    auto cb =
        [bw](const std::shared_ptr<const autolink::message::RawMessage>& m) {
            if (m)
                bw->OnData(m->message);
        };
    auto reader =
        node->CreateReader<autolink::message::RawMessage>(channel_name, cb);
    if (!reader) {
        std::cerr << "Failed to create reader for channel: " << channel_name
                  << std::endl;
        return;
    }
    std::cout << "reader to [" << channel_name << "]" << std::endl;
    while (!autolink::IsShutdown()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        bw->Print();
    }
}

// -----------------------------------------------------------------------------
// hz: frequency stats (inter-message intervals, print rate every second)
// -----------------------------------------------------------------------------
class ChannelHz
{
public:
    explicit ChannelHz(int window_size)
        : window_size_(window_size <= 0 || window_size > kMaxWindowSize
                           ? kDefaultHzWindowSize
                           : window_size),
          last_printed_tn_(0),
          msg_t0_(-1),
          msg_tn_(0) {
        std::cout << "hz window_size: " << window_size_ << std::endl;
    }

    void OnMessage() {
        std::lock_guard<std::mutex> lock(mutex_);
        double curr = autolink::Time::Now().ToSecond();
        if (curr == 0) {
            if (!times_.empty()) {
                std::cout << "reset times." << std::endl;
                times_.clear();
            }
            return;
        }
        if (msg_t0_ < 0 || msg_t0_ > curr) {
            msg_t0_ = curr;
            msg_tn_ = curr;
            times_.clear();
        } else {
            times_.push_back(curr - msg_tn_);
            msg_tn_ = curr;
        }
        while (static_cast<int>(times_.size()) > window_size_ - 1) {
            times_.erase(times_.begin());
        }
    }

    void Print() {
        std::vector<double> times;
        double msg_tn_snapshot = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (times_.empty())
                return;
            if (msg_tn_ == last_printed_tn_) {
                std::cout << "no new messages" << std::endl;
                return;
            }
            times = times_;
            msg_tn_snapshot = msg_tn_;
            last_printed_tn_ = msg_tn_;
        }
        int n = static_cast<int>(times.size());
        double mean = 0;
        for (double t : times)
            mean += t;
        mean /= n;
        double rate = (mean > 0) ? (1.0 / mean) : 0;
        double var = 0;
        for (double t : times)
            var += (t - mean) * (t - mean);
        double std_dev = std::sqrt(var / n);
        double max_delta = *std::max_element(times.begin(), times.end());
        double min_delta = *std::min_element(times.begin(), times.end());
        (void)msg_tn_snapshot;
        std::cout << "average rate: " << std::fixed << std::setprecision(3)
                  << rate << "\n\tmin: " << std::setprecision(3) << min_delta
                  << "s max: " << max_delta
                  << "s std dev: " << std::setprecision(5) << std_dev
                  << "s window: " << (n + 1) << std::endl;
    }

private:
    int window_size_;
    std::mutex mutex_;
    double last_printed_tn_;
    double msg_t0_;
    double msg_tn_;
    std::vector<double> times_;
};

void CmdHz(const std::string& channel_name, int window_size) {
    auto hz = std::make_shared<ChannelHz>(window_size);
    auto node = autolink::CreateNode("listener_node_hz");
    if (!node) {
        std::cerr << "Failed to create node" << std::endl;
        return;
    }
    auto cb =
        [hz](const std::shared_ptr<const autolink::message::RawMessage>&) {
            hz->OnMessage();
        };
    auto reader =
        node->CreateReader<autolink::message::RawMessage>(channel_name, cb);
    if (!reader) {
        std::cerr << "Failed to create reader for channel: " << channel_name
                  << std::endl;
        return;
    }
    std::cout << "reader to [" << channel_name << "]" << std::endl;
    while (!autolink::IsShutdown()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        hz->Print();
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        PrintUsage();
        return 64;
    }

    const std::string command(argv[1]);

    std::signal(SIGINT, [](int sig) { autolink::OnShutdown(sig); });
    std::signal(SIGTERM, [](int sig) { autolink::OnShutdown(sig); });

    autolink::Init(argv[0]);

    int ret = 0;

    if (command == "list") {
        if (argc > 2) {
            std::cerr << "usage: autolink_channel list" << std::endl;
            ret = 64;
        } else {
            CmdList();
        }
    } else if (command == "type") {
        if (argc < 3) {
            std::cerr << "usage: autolink_channel type channelname"
                      << std::endl;
            ret = 64;
        } else if (argc > 3) {
            std::cerr << "you may only specify one input channel" << std::endl;
            ret = 64;
        } else {
            PrintChannelType(argv[2]);
        }
    } else if (command == "info") {
        bool all_channels = false;
        std::string channel_name;
        int info_argc = argc - 2;
        char** info_argv = argv + 2;
        static const struct option info_opts[] = {
            {"all", no_argument, nullptr, 'a'},
            {"help", no_argument, nullptr, 'h'},
            {nullptr, 0, nullptr, 0}};
        optind = 0;
        int c;
        while (info_argc > 0 && (c = getopt_long(info_argc, info_argv, "ah",
                                                 info_opts, nullptr)) != -1) {
            if (c == 'a')
                all_channels = true;
            if (c == 'h') {
                std::cout << "usage: autolink_channel info channelname\n"
                          << "       autolink_channel info -a|--all\n";
                return 0;
            }
        }
        if (optind < info_argc) {
            channel_name = info_argv[optind];
            if (optind + 1 < info_argc) {
                std::cerr << "you may only specify one topic name" << std::endl;
                ret = 64;
                channel_name.clear();
            }
        }
        if (ret == 0 && channel_name.empty() && !all_channels) {
            std::cerr << "channelname must be specified (or use -a/--all)"
                      << std::endl;
            ret = 64;
        } else if (ret == 0) {
            CmdInfo(channel_name, all_channels);
        }
    } else if (command == "echo") {
        if (argc < 3) {
            std::cerr << "usage: autolink_channel echo channelname"
                      << std::endl;
            ret = 64;
        } else if (argc > 3) {
            std::cerr << "you may only specify one input channel" << std::endl;
            ret = 64;
        } else {
            CmdEcho(argv[2]);
        }
    } else if (command == "bw") {
        int window_size = kDefaultBwWindowSize;
        std::string channel_name;
        int bw_argc = argc - 2;
        char** bw_argv = argv + 2;
        static const struct option bw_opts[] = {
            {"window", required_argument, nullptr, 'w'},
            {"help", no_argument, nullptr, 'h'},
            {nullptr, 0, nullptr, 0}};
        optind = 0;
        int c;
        while (bw_argc > 0 && (c = getopt_long(bw_argc, bw_argv, "w:h", bw_opts,
                                               nullptr)) != -1) {
            if (c == 'w') {
                try {
                    window_size = std::stoi(optarg);
                } catch (...) {
                    std::cerr << "window size must be an integer" << std::endl;
                    return 64;
                }
            }
            if (c == 'h') {
                std::cout << "usage: autolink_channel bw channelname "
                             "[-w|--window N]\n";
                return 0;
            }
        }
        if (optind < bw_argc) {
            channel_name = bw_argv[optind];
        }
        if (channel_name.empty() && bw_argc > 0 && bw_argv[0][0] != '-') {
            channel_name = bw_argv[0];
        }
        if (channel_name.empty()) {
            std::cerr << "usage: autolink_channel bw channelname" << std::endl;
            ret = 64;
        } else {
            CmdBw(channel_name, window_size);
        }
    } else if (command == "hz") {
        int window_size = kDefaultHzWindowSize;
        std::string channel_name;
        int hz_argc = argc - 2;
        char** hz_argv = argv + 2;
        static const struct option hz_opts[] = {
            {"window", required_argument, nullptr, 'w'},
            {"help", no_argument, nullptr, 'h'},
            {nullptr, 0, nullptr, 0}};
        optind = 0;
        int c;
        while (hz_argc > 0 && (c = getopt_long(hz_argc, hz_argv, "w:h", hz_opts,
                                               nullptr)) != -1) {
            if (c == 'w') {
                try {
                    window_size = std::stoi(optarg);
                } catch (...) {
                    std::cerr << "window size must be an integer" << std::endl;
                    return 64;
                }
            }
            if (c == 'h') {
                std::cout << "usage: autolink_channel hz channelname "
                             "[-w|--window N]\n";
                return 0;
            }
        }
        if (optind < hz_argc) {
            channel_name = hz_argv[optind];
        }
        if (channel_name.empty() && hz_argc > 0 && hz_argv[0][0] != '-') {
            channel_name = hz_argv[0];
        }
        if (channel_name.empty()) {
            std::cerr << "usage: autolink_channel hz channelname" << std::endl;
            ret = 64;
        } else {
            CmdHz(channel_name, window_size);
        }
    } else {
        PrintUsage();
        ret = 64;
    }

    autolink::Clear();
    return ret;
}
