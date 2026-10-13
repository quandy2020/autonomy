/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins transformation::TFFrameTransformer
 * (Bosch Software Innovations / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/transformation/tf_frame_transformer.hpp"

#include "autoviz/transform/buffer.hpp"
#include "autoviz/transform/tf2/exceptions.h"

namespace autoviz {
namespace display {
namespace transformation {

TFFrameTransformer::TFFrameTransformer()
    : tf_wrapper_(std::make_shared<TFWrapper>()) {}

TFFrameTransformer::TFFrameTransformer(std::shared_ptr<TFWrapper> wrapper)
    : tf_wrapper_(std::move(wrapper)) {
  if (tf_wrapper_ == nullptr) {
    tf_wrapper_ = std::make_shared<TFWrapper>();
  }
}

TFFrameTransformer::TFFrameTransformer(transform::Buffer* buffer)
    : tf_wrapper_(std::make_shared<TFWrapper>(buffer)) {}

common::PluginInfo TFFrameTransformer::info() const {
  common::PluginInfo plugin;
  plugin.class_id = "autoviz/AutolinkTf";
  plugin.name = "Autolink TF";
  plugin.description =
      "Transform using the Autolink TF buffer (RViz TFFrameTransformer).";
  plugin.package = "autoviz";
  return plugin;
}

transform::Buffer* TFFrameTransformer::buffer() const {
  return tf_wrapper_ != nullptr ? tf_wrapper_->getBuffer() : nullptr;
}

void TFFrameTransformer::initialize(transform::Buffer* buffer) {
  if (tf_wrapper_ == nullptr) {
    tf_wrapper_ = std::make_shared<TFWrapper>();
  }
  tf_wrapper_->initialize(buffer);
}

void TFFrameTransformer::clear() {
  if (tf_wrapper_ != nullptr) {
    tf_wrapper_->clear();
  }
}

std::vector<std::string> TFFrameTransformer::getAllFrameNames() const {
  return tf_wrapper_ != nullptr ? tf_wrapper_->getFrameStrings()
                                : std::vector<std::string>{};
}

void TFFrameTransformer::RethrowAsFrameTransformerException(
    const char* prefix, const std::exception& exception) {
  throw common::transformation::FrameTransformerException(std::string(prefix) +
                                          exception.what());
}

automsgs::msgs::geometry_msgs::PoseStamped TFFrameTransformer::transform(
    const automsgs::msgs::geometry_msgs::PoseStamped& pose_in,
    const std::string& target_frame) const {
  try {
    automsgs::msgs::geometry_msgs::PoseStamped out;
    tf_wrapper_->transform(pose_in, &out, target_frame);
    return out;
  } catch (const transform::tf2::LookupException& exception) {
    RethrowAsFrameTransformerException("[tf2::LookupException]: ", exception);
  } catch (const transform::tf2::ConnectivityException& exception) {
    RethrowAsFrameTransformerException("[tf2::ConnectivityException]: ",
                                       exception);
  } catch (const transform::tf2::ExtrapolationException& exception) {
    RethrowAsFrameTransformerException("[tf2::ExtrapolationException]: ",
                                       exception);
  } catch (const transform::tf2::InvalidArgumentException& exception) {
    RethrowAsFrameTransformerException("[tf2::InvalidArgumentException]: ",
                                       exception);
  } catch (const transform::tf2::TimeoutException& exception) {
    RethrowAsFrameTransformerException("[tf2::TimeoutException]: ", exception);
  }
}

automsgs::msgs::geometry_msgs::TransformStamped
TFFrameTransformer::lookupTransform(
    const std::string& target_frame, const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& time) const {
  try {
    return tf_wrapper_->lookupTransform(target_frame, source_frame, time);
  } catch (const transform::tf2::LookupException& exception) {
    RethrowAsFrameTransformerException("[tf2::LookupException]: ", exception);
  } catch (const transform::tf2::ConnectivityException& exception) {
    RethrowAsFrameTransformerException("[tf2::ConnectivityException]: ",
                                       exception);
  } catch (const transform::tf2::ExtrapolationException& exception) {
    RethrowAsFrameTransformerException("[tf2::ExtrapolationException]: ",
                                       exception);
  } catch (const transform::tf2::InvalidArgumentException& exception) {
    RethrowAsFrameTransformerException("[tf2::InvalidArgumentException]: ",
                                       exception);
  } catch (const transform::tf2::TimeoutException& exception) {
    RethrowAsFrameTransformerException("[tf2::TimeoutException]: ", exception);
  }
}

automsgs::msgs::geometry_msgs::TransformStamped
TFFrameTransformer::lookupTransform(
    const std::string& target_frame,
    const automsgs::msgs::builtin_interfaces::Time& target_time,
    const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& source_time,
    const std::string& fixed_frame) const {
  try {
    return tf_wrapper_->lookupTransform(target_frame, target_time, source_frame,
                                        source_time, fixed_frame);
  } catch (const transform::tf2::LookupException& exception) {
    RethrowAsFrameTransformerException("[tf2::LookupException]: ", exception);
  } catch (const transform::tf2::ConnectivityException& exception) {
    RethrowAsFrameTransformerException("[tf2::ConnectivityException]: ",
                                       exception);
  } catch (const transform::tf2::ExtrapolationException& exception) {
    RethrowAsFrameTransformerException("[tf2::ExtrapolationException]: ",
                                       exception);
  } catch (const transform::tf2::InvalidArgumentException& exception) {
    RethrowAsFrameTransformerException("[tf2::InvalidArgumentException]: ",
                                       exception);
  } catch (const transform::tf2::TimeoutException& exception) {
    RethrowAsFrameTransformerException("[tf2::TimeoutException]: ", exception);
  }
}

bool TFFrameTransformer::canTransform(
    const std::string& target_frame, const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& time,
    std::string* error) const {
  std::string tf2_error;
  if (tf_wrapper_->canTransform(target_frame, source_frame, time, &tf2_error)) {
    return true;
  }
  if (error == nullptr) {
    return false;
  }
  std::string frame_error;
  const bool target_ok = !frameHasProblems(target_frame, &frame_error);
  const bool source_ok = target_ok && !frameHasProblems(source_frame, &frame_error);
  if (source_ok) {
    *error = "No transform to fixed frame [" + target_frame +
             "]. TF error: [" + tf2_error + "]";
    return false;
  }
  *error = target_ok ? ("For frame [" + source_frame + "]: " + frame_error)
                     : ("For frame [" + source_frame + "]: Fixed " + frame_error);
  return false;
}

bool TFFrameTransformer::canTransform(
    const std::string& target_frame,
    const automsgs::msgs::builtin_interfaces::Time& target_time,
    const std::string& source_frame,
    const automsgs::msgs::builtin_interfaces::Time& source_time,
    const std::string& fixed_frame, std::string* error) const {
  std::string tf2_error;
  if (tf_wrapper_->canTransform(target_frame, target_time, source_frame,
                                source_time, fixed_frame, &tf2_error)) {
    return true;
  }
  if (error == nullptr) {
    return false;
  }
  std::string frame_error;
  const bool target_ok = !frameHasProblems(target_frame, &frame_error);
  const bool source_ok = target_ok && !frameHasProblems(source_frame, &frame_error);
  if (source_ok) {
    *error = "No transform to fixed frame [" + target_frame +
             "]. TF error: [" + tf2_error + "]";
    return false;
  }
  *error = target_ok ? ("For frame [" + source_frame + "]: " + frame_error)
                     : ("For frame [" + source_frame + "]: Fixed " + frame_error);
  return false;
}

bool TFFrameTransformer::frameHasProblems(const std::string& frame,
                                          std::string* error) const {
  if (tf_wrapper_ != nullptr && tf_wrapper_->frameExists(frame)) {
    return false;
  }
  if (error != nullptr) {
    *error = "Frame [" + frame + "] does not exist";
  }
  return true;
}

std::weak_ptr<common::transformation::TransformationLibraryConnector>
TFFrameTransformer::getConnector()
    const {
  return tf_wrapper_;
}

}  // namespace transformation
}  // namespace display
}  // namespace autoviz
