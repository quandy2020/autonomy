/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <gtest/gtest.h>

#include <string>

#include <QGuiApplication>

#include "autoviz/common/display_context.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/transform/buffer.hpp"

namespace autoviz {
namespace tests {

/** Exposes ChannelDisplay::processMessage / onDraw for headless display tests. */
template <typename DisplayT>
class TestableDisplay : public DisplayT {
 public:
  using DisplayT::DisplayT;

  template <typename ProtoT>
  void feedMessage(const ProtoT& message) {
    this->processMessage(message);
  }

  void updateDisplay() { this->onUpdate(); }
  void enableDisplay() { this->onEnable(); }
  void disableDisplay() { this->onDisable(); }
  void drawOverlay(rendering::SceneOverlay& scene) { this->onDraw(scene); }
};

class DisplayTestFixture : public ::testing::Test {
 public:
  template <typename DisplayT, typename ProtoT>
  void feedAndDraw(DisplayT& display, const ProtoT& message,
                   const std::string& display_name = "") {
    display.setContext(&context_);
    display.setDisplayName(display_name.empty() ? display.typeId() : display_name);
    display.feedMessage(message);
    scene_.clear();
    display.drawOverlay(scene_);
  }

  common::DisplayContext& context() { return context_; }
  rendering::SceneOverlay& scene() { return scene_; }

 protected:
  void SetUp() override {
    EnsureQtApplication();
    tf_buffer_ = transform::Buffer::Instance();
    context_.fixed_frame = "map";
    context_.tf_buffer = tf_buffer_;
    context_.request_redraw = []() {};
  }

  static void EnsureQtApplication() {
    if (QGuiApplication::instance() != nullptr) {
      return;
    }
    static int argc = 1;
    static char arg0[] = "autoviz_bicmap_test";
    static char* argv[] = {arg0, nullptr};
    static QGuiApplication app(argc, argv);
    (void)app;
  }

 private:
  transform::Buffer* tf_buffer_ = nullptr;
  common::DisplayContext context_;
  rendering::SceneOverlay scene_;
};

}  // namespace tests
}  // namespace autoviz
