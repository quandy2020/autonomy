/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/manipulation/motion/execution/joint_state_subscriber.hpp"

#include <automsgs/msgs/sensor_msgs/joint_state.pb.h>

#include "autolink/node/reader.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace manipulation {
namespace execution {

bool JointStateSubscriber::Init(const std::shared_ptr<autolink::Node>& node,
                                const std::string& topic) {
  node_ = node;
  topic_ = topic.empty() ? "/joint_states" : topic;
  if (!node_) {
    AWARN << "JointStateSubscriber: null node; use Update() injection";
    return true;
  }
  reader_ = node_->CreateReader<automsgs::msgs::sensor_msgs::JointState>(
      topic_,
      [this](const std::shared_ptr<automsgs::msgs::sensor_msgs::JointState>&
                 msg) {
        if (!msg) {
          return;
        }
        Update(*msg);
      });
  if (reader_) {
    AINFO << "JointStateSubscriber subscribed topic=" << topic_;
  } else {
    AWARN << "JointStateSubscriber: subscribe failed topic=" << topic_
          << "; use Update() injection";
  }
  return true;
}

void JointStateSubscriber::SetScene(
    std::shared_ptr<scene::PlanningScene> scene) {
  std::lock_guard<std::mutex> lock(mutex_);
  scene_ = std::move(scene);
}

void JointStateSubscriber::SetCallback(Callback cb) {
  std::lock_guard<std::mutex> lock(mutex_);
  callback_ = std::move(cb);
}

void JointStateSubscriber::Update(const core::JointState& state) {
  std::shared_ptr<scene::PlanningScene> scene;
  Callback cb;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = state;
    scene = scene_;
    cb = callback_;
  }
  if (scene) {
    scene->SetCurrentState(state);
  }
  if (cb) {
    cb(state);
  }
}

core::JointState JointStateSubscriber::Latest() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_;
}

}  // namespace execution
}  // namespace manipulation
}  // namespace autonomy
