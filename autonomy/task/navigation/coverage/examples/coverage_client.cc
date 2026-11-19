// Copyright 2026 The Openbot Authors (duyongquan).
// SPDX-License-Identifier: Apache-2.0
/** @file coverage_client.cc
 * @brief Native coverage action CLI and AutoSim observation recorder. */
#include "autolink/autolink.hpp"
#include "autolink/action/client.hpp"
#include "autonomy/task/navigation/coverage/proto/coverage.pb.h"
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <google/protobuf/text_format.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>

namespace coverage = autonomy::task::coverage::proto;
int main(int argc,char** argv) {
  if(argc<4){std::cerr<<"Usage: task.coverage_client plan|navigate|observe request.pb.txt output-prefix [seconds]\n";return 2;}
  if(!autolink::Init(argv[0]))return 2;
  auto node=autolink::CreateNode("coverage_cli");
  coverage::CoverageRequest request;
  std::ifstream input(argv[2]);std::string text((std::istreambuf_iterator<char>(input)),{});
  if(!google::protobuf::TextFormat::ParseFromString(text,&request)){std::cerr<<"Invalid request\n";return 2;}
  const std::string mode=argv[1],output=argv[3];const double duration=argc>4?std::stod(argv[4]):180;
  std::ofstream poses(output+"-poses.csv");poses<<"t,x,y,yaw\n";std::mutex mutex;
  const auto began=std::chrono::steady_clock::now();
  auto pose_reader=node->CreateReader<automsgs::msgs::geometry_msgs::PoseStamped>("/gt/pose",[&](const auto& pose){if(!pose)return;const auto& p=pose->pose().position();const auto& q=pose->pose().orientation();const double yaw=std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));std::lock_guard<std::mutex> lock(mutex);poses<<std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<<','<<p.x()<<','<<p.y()<<','<<yaw<<'\n';poses.flush();});
  std::ofstream tools(output+"-tools.pb.txt");std::ofstream tool_csv(output+"-tools.csv");tool_csv<<"t,sequence,enabled,speed\n";auto ack_writer=node->CreateWriter<coverage::ToolAcknowledgment>("/coverage/tool_ack");
  auto tool_reader=node->CreateReader<coverage::ToolCommand>("/coverage/tool_command",[&](const auto& command){if(!command)return;std::lock_guard<std::mutex> lock(mutex);tools<<command->DebugString()<<'\n';tools.flush();tool_csv<<std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<<','<<command->sequence()<<','<<command->enabled()<<','<<command->speed()<<'\n';tool_csv.flush();coverage::ToolAcknowledgment ack;ack.set_task_id(command->task_id());ack.set_sequence(command->sequence());ack.set_enabled(command->enabled());ack.set_accepted(true);ack_writer->Write(ack);});
  auto plan_reader=node->CreateReader<automsgs::msgs::nav_msgs::Path>("/coverage/plan",[&](const auto& path){if(!path)return;std::lock_guard<std::mutex> lock(mutex);std::ofstream(output+"-plan.pb.txt")<<path->DebugString();});
  if(mode=="observe"){while(std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()<duration)std::this_thread::sleep_for(std::chrono::milliseconds(100));return 0;}
  if(mode=="plan") {
    using Action=coverage::ComputeCoveragePathAction;auto client=std::make_shared<autolink::action::Client<Action>>(node,"/compute_coverage_path");
    if(!client->WaitForActionServer(std::chrono::seconds(10))){std::cerr<<"Planner unavailable\n";return 1;}
    Action::Goal goal;*goal.mutable_request()=request;auto acceptance=client->AsyncSendGoal(goal);
    if(acceptance.wait_for(std::chrono::seconds(10))!=std::future_status::ready){std::cerr<<"Coverage goal acceptance deadline exceeded\n";return 1;}
    auto handle=acceptance.get();if(!handle){std::cerr<<"Coverage goal rejected\n";return 1;}std::cerr<<"Coverage goal accepted\n";auto result=client->AsyncGetResult(handle);
    if(result.wait_for(std::chrono::seconds(60))!=std::future_status::ready){client->AsyncCancelGoal(handle);std::cerr<<"Coverage planning result deadline exceeded\n";return 1;}
    auto wrapped=result.get();if(!wrapped.result)return 1;std::ofstream(output+"-result.pb.txt")<<wrapped.result->DebugString();std::cout<<wrapped.result->error_code()<<' '<<wrapped.result->error_msg()<<" coverage="<<wrapped.result->core_coverage()<<'\n';return wrapped.result->error_code()==0?0:1;
  }
  using Action=coverage::NavigateCompleteCoverageAction;auto client=std::make_shared<autolink::action::Client<Action>>(node,"/navigate_complete_coverage");
  if(!client->WaitForActionServer(std::chrono::seconds(10))){std::cerr<<"Navigator unavailable\n";return 1;}
  Action::Goal goal;*goal.mutable_request()=request;goal.set_task_id("coverage-cli-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));goal.set_client_id("coverage-cli");goal.set_navigate_to_start(true);goal.set_tool_required(true);goal.set_timeout_sec(duration);
  typename autolink::action::Client<Action>::SendGoalOptions options;
  options.feedback_callback=[&](auto,const auto& feedback){if(!feedback)return;std::cout<<"coverage "<<feedback->progress()*100<<"% remaining="<<feedback->distance_remaining()<<" time="<<feedback->navigation_time()<<'\n';};
  auto acceptance=client->AsyncSendGoal(goal,options);
  if(acceptance.wait_for(std::chrono::seconds(10))!=std::future_status::ready){std::cerr<<"Coverage goal acceptance deadline exceeded\n";return 1;}
  auto handle=acceptance.get();if(!handle){std::cerr<<"Navigator rejected\n";return 1;}auto result=handle->AsyncGetResult();
  if(result.wait_for(std::chrono::milliseconds(static_cast<int64_t>((duration+10)*1000)))!=std::future_status::ready){client->AsyncCancelGoal(handle);std::cerr<<"Navigation result timeout\n";return 1;}
  auto wrapped=result.get();if(!wrapped.result)return 1;std::ofstream(output+"-result.pb.txt")<<wrapped.result->DebugString();std::cout<<wrapped.result->DebugString();return wrapped.result->error_code()==0?0:1;
}
