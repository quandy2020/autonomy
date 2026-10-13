/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins transformation::TransformerGuard
 * (Bosch Software Innovations / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file transformer_guard.hpp
 * @brief Disables a Display when the active FrameTransformer type mismatches.
 *
 * RViz uses Qt signals from TransformationManager / Display. Autoviz wires
 * @ref TransformationManager change callbacks and a @ref notifyEnabledChanged
 * hook instead (Display is not a QObject).
 *
 * @see TFFrameTransformer
 * @see TransformationManager
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "autoviz/common/display_context.hpp"
#include "autoviz/common/transformation/frame_transformer.hpp"
#include "autoviz/common/transformation/transformation_manager.hpp"
#include "autoviz/plugins/displays/display.hpp"

namespace autoviz {
namespace display {
namespace transformation {

/**
 * @class TransformerGuardBase
 * @brief Non-template base so TransformationManager can notify without Qt moc.
 */
class TransformerGuardBase {
 public:
  TransformerGuardBase(Display* display, std::string transformer_name);
  virtual ~TransformerGuardBase();

  void initialize(common::DisplayContext* context,
                  common::transformation::TransformationManager*
                      transformation_manager);

  virtual bool checkTransformer() = 0;

  /** Call when the host Display enable state may have changed externally. */
  void notifyEnabledChanged();

 protected:
  virtual void transformerChanged(
      common::transformation::FrameTransformer* new_transformer) = 0;
  virtual void displayEnabledChanged() = 0;

  Display* display_ = nullptr;
  std::string allowed_transformer_name_;
  bool using_allowed_transformer_ = true;
  bool display_disabled_by_user_ = false;
  common::DisplayContext* context_ = nullptr;
  common::transformation::TransformationManager* transformation_manager_ =
      nullptr;
  std::function<void()> unsubscribe_;
};

/**
 * @class TransformerGuard
 * @brief Template guard requiring @tparam AllowedTransformerType.
 */
template <typename AllowedTransformerType>
class TransformerGuard : public TransformerGuardBase {
 public:
  TransformerGuard(Display* display, const std::string& transformer_name)
      : TransformerGuardBase(display, transformer_name) {}

  ~TransformerGuard() override = default;

  bool checkTransformer() override {
    if (transformation_manager_ == nullptr) {
      return false;
    }
    return isAllowedTransformer(
        transformation_manager_->currentTransformer());
  }

  void updateDisplayAccordingToTransformerType(
      common::transformation::FrameTransformer* transformer) {
    using_allowed_transformer_ = isAllowedTransformer(transformer);
    if (!using_allowed_transformer_) {
      display_disabled_by_user_ =
          display_ != nullptr && !display_->enabled();
      disableDisplayAndSetErrorStatus();
    } else {
      enableDisplayAndDeleteErrorStatus();
    }
  }

 private:
  void transformerChanged(
      common::transformation::FrameTransformer* new_transformer) override {
    if (using_allowed_transformer_ != isAllowedTransformer(new_transformer)) {
      updateDisplayAccordingToTransformerType(
          transformation_manager_ != nullptr
              ? transformation_manager_->currentTransformer()
              : new_transformer);
    }
  }

  void displayEnabledChanged() override {
    if (!using_allowed_transformer_) {
      disableDisplayAndSetErrorStatus();
    }
  }

  bool isAllowedTransformer(
      common::transformation::FrameTransformer* transformer) const {
    return dynamic_cast<AllowedTransformerType*>(transformer) != nullptr;
  }

  void setErrorStatus() {
    if (display_ == nullptr) {
      return;
    }
    display_->setStatusError("The display works only with " +
                             allowed_transformer_name_ + " Transformer");
  }

  void enableDisplayAndDeleteErrorStatus() {
    if (display_ == nullptr) {
      return;
    }
    display_->setStatusOk();
    if (!display_disabled_by_user_) {
      display_->setEnabled(true);
    }
  }

  void disableDisplayAndSetErrorStatus() {
    if (display_ == nullptr) {
      return;
    }
    // Avoid re-entrancy storms when called from Display::setEnabled → onEnable.
    if (updating_) {
      return;
    }
    updating_ = true;
    display_->setEnabled(false);
    setErrorStatus();
    updating_ = false;
  }

  bool updating_ = false;
};

}  // namespace transformation
}  // namespace display
}  // namespace autoviz
