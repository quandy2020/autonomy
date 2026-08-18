/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <QImage>
#include <QQuaternion>
#include <QRgb>
#include <QVector3D>

#include <automsgs/msgs/tf2_msgs/tf_message.pb.h>

#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"
#include "autoviz/transform/tf2/signal.hpp"

namespace autoviz {
namespace display {

struct TfFrameSnapshot {
  std::string name;
  std::string parent;
  bool enabled = true;
  QVector3D position;
  QQuaternion orientation;
  QVector3D rel_position;
  QQuaternion rel_orientation;
  bool have_fixed_pose = false;
};

class TfDisplay : public ChannelDisplay<automsgs::msgs::tf2_msgs::TFMessage> {
 public:
  explicit TfDisplay(std::string channel = "/tf");
  ~TfDisplay() override;

  std::string typeId() const override { return "TF"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

  std::vector<TfFrameSnapshot> frameSnapshots() const;
  /** Edges as (child, parent); empty parent means root. */
  std::vector<std::pair<std::string, std::string>> treeEdges() const;

  void setAllFramesEnabled(bool enabled);
  void setFrameEnabled(const std::string& frame, bool enabled);
  bool allFramesEnabled() const { return all_enabled_; }

  void reset() override;
  void load(const common::Config& config) override;
  void save(common::Config config) const override;
  void saveToConfig(common::DisplayConfig* config) const override;

 protected:
  void onEnable() override;
  void onDisable() override;
  void onUpdate() override;
  void onPropertyChanged(const std::string& key) override;
  void processMessage(
      const automsgs::msgs::tf2_msgs::TFMessage& message) override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct FrameInfo {
    std::string name;
    std::string parent;
    bool enabled = true;
    QVector3D position;
    QQuaternion orientation = QQuaternion(1.f, 0.f, 0.f, 0.f);
    QVector3D rel_position;
    QQuaternion rel_orientation = QQuaternion(1.f, 0.f, 0.f, 0.f);
    /** Wall time when transform-to-fixed last changed (RViz last_update_). */
    int64_t last_update_ns = 0;
    /** Last observed TF stamp / common time for aging (RViz last_time_to_fixed_). */
    int64_t last_time_to_fixed_ns = -1;
    bool have_fixed_pose = false;
    /** Cached Show Names label (avoid QImage raster every draw). */
    QImage name_label;
    QRgb name_label_rgba = 0;
    int name_label_pixel_height = 0;
  };

  struct CachedProps {
    bool show_names = false;
    bool show_axes = true;
    bool show_arrows = true;
    float marker_scale = 1.f;
    float update_interval = 0.f;
    float frame_timeout = 15.f;
    std::string filter_whitelist;
    std::string filter_blacklist;
  };

  void updateFrames();
  void refreshCachedProps();
  void ensureTransformsListener();
  void markFramesDirty();
  void applyEnabledFromConfig(FrameInfo* info);
  void persistEnabledIntoProperties();
  void noteFrameTransformTime(FrameInfo* info, int64_t latest_time_ns);
  void setOkStatus();

  std::map<std::string, FrameInfo> frames_;
  std::map<std::string, bool> frame_enabled_from_config_;
  CachedProps props_;
  bool all_enabled_ = true;
  bool changing_single_frame_ = false;
  bool frames_dirty_ = true;
  double update_timer_sec_ = 0.0;
  double last_wall_sec_ = 0.0;
  int64_t last_msg_wall_ns_ = 0;
  uint64_t tf_messages_received_ = 0;
  int last_drew_frames_ = 0;
  int last_posed_frames_ = 0;
  std::string filter_error_;
  std::string fixed_frame_cached_;
  bool transforms_listener_attached_ = false;
  transform::tf2::VoidSignal::Connection transforms_changed_connection_;
};

}  // namespace display
}  // namespace autoviz
