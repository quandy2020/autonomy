// Copyright 2026 The Openbot Authors
#pragma once

#ifdef AUTOVIZ_HAS_CARTOGRAPHER
#include <chrono>
#include <map>
#include <utility>
#include <vector>

#include <QImage>
#include <QMatrix4x4>
#include "autolink/service/client.hpp"
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
  void onUpdate() override;
  void onDraw(rendering::SceneOverlay& scene) override;
  void onPropertyChanged(const std::string& key) override;
  void processMessage(const autonomy::localization::cartographer::proto::SubmapList& message) override;
  void clearReceivedData() override;

 private:
  using Key = std::pair<int, int>;
  using Clock = std::chrono::steady_clock;
  using Client = autolink::Client<autonomy::localization::cartographer::proto::SubmapQueryRequest,
                                  autonomy::localization::cartographer::proto::SubmapQueryResponse>;
  struct Slice {
    QImage image;
    QMatrix4x4 pose;
    float width = 0.f;
    float height = 0.f;
  };
  struct Submap {
    autonomy::localization::cartographer::proto::SubmapEntry metadata;
    int texture_version = -1;
    std::vector<Slice> slices;
    QImage label;
    Clock::time_point next_query{};
  };
  struct Pending {
    Client::SharedFuture future;
    Clock::time_point started;
  };
  void resetClient();
  bool visible(int trajectory) const;
  bool decodeTextures(const autonomy::localization::cartographer::proto::SubmapQueryResponse& response,
                      Submap* submap);
  std::map<Key, Submap> submaps_;
  std::map<Key, Pending> pending_;
  std::shared_ptr<Client> client_;
  std::string frame_;
};
}  // namespace autoviz::display
#endif
