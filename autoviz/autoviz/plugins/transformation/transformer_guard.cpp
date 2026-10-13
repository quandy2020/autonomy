/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins transformation::TransformerGuard
 * (Bosch / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/transformation/transformer_guard.hpp"

namespace autoviz {
namespace display {
namespace transformation {

TransformerGuardBase::TransformerGuardBase(Display* display,
                                           std::string transformer_name)
    : display_(display),
      allowed_transformer_name_(std::move(transformer_name)) {}

TransformerGuardBase::~TransformerGuardBase() {
  if (unsubscribe_) {
    unsubscribe_();
  }
}

void TransformerGuardBase::initialize(
    common::DisplayContext* context,
    common::transformation::TransformationManager* transformation_manager) {
  context_ = context;
  transformation_manager_ = transformation_manager;
  if (unsubscribe_) {
    unsubscribe_();
    unsubscribe_ = {};
  }
  if (transformation_manager_ != nullptr) {
    unsubscribe_ = transformation_manager_->addTransformerChangedCallback(
        [this](common::transformation::FrameTransformer* transformer) {
          transformerChanged(transformer);
        });
  }
  if (display_ != nullptr) {
    display_->setEnabledChangedCallback(
        [this]() { notifyEnabledChanged(); });
  }
  // Defer disable to the caller (onEnable) via checkTransformer() — do not
  // call setEnabled() from inside Display::setEnabled → onEnable.
  using_allowed_transformer_ = checkTransformer();
}

void TransformerGuardBase::notifyEnabledChanged() { displayEnabledChanged(); }

}  // namespace transformation
}  // namespace display
}  // namespace autoviz
