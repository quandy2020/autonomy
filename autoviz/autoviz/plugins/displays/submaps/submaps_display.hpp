// Copyright 2026 The Openbot Authors
#pragma once

#ifdef AUTOVIZ_HAS_CARTOGRAPHER
#include <map>
#include <utility>
#include <vector>
#include <QMatrix4x4>
#include <QImage>
#include "autonomy/localization/cartographer/proto/cartographer_services.pb.h"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz::display {

class SubmapsDisplay : public ChannelDisplay<autonomy::localization::cartographer::proto::SubmapList> {
 public:
  explicit SubmapsDisplay(std::string channel);
  ~SubmapsDisplay() override;
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onDisable() override;
  void onDraw(rendering::SceneOverlay& scene) override;
  void processMessage(const autonomy::localization::cartographer::proto::SubmapList& message) override;
  void clearReceivedData() override;
  void onPropertyChanged(const std::string& key) override;

 private:
  using Key = std::pair<int, int>;
  bool submapVisible(const Key& key) const;
  struct Slice {
    QImage image;
    QMatrix4x4 pose;
    float width = 0.f;
    float height = 0.f;
  };
  struct Submap {
    autonomy::localization::cartographer::proto::SubmapEntry metadata;
    QImage label;
    int texture_version = -1;
    std::vector<Slice> slices;
  };
  std::map<Key, Submap> submaps_;
  std::string frame_;
};
}  // namespace autoviz::display
#endif
