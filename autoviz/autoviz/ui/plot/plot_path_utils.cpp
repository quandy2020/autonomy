/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/plot_path_utils.hpp"

#include <map>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_manager.hpp"
#include "autoviz/ui/plot/message_field_tree.hpp"

#include "autoviz/commsgs/message_type_utils.hpp"

namespace autoviz {
namespace plot {
namespace {

const std::map<QString, std::string>& TutorialChannelMessageTypes() {
  static const std::map<QString, std::string> kTutorialChannels = {
      {QStringLiteral("/fake/path"), "automsgs.msgs.nav_msgs.Path"},
      {QStringLiteral("/fake/odom"), "automsgs.msgs.nav_msgs.Odometry"},
      {QStringLiteral("/fake/scan"), "automsgs.msgs.sensor_msgs.LaserScan"},
      {QStringLiteral("/fake/marker"), "automsgs.msgs.visualization_msgs.Marker"},
      {QStringLiteral("/fake/marker_array"),
       "automsgs.msgs.visualization_msgs.MarkerArray"},
      {QStringLiteral("/fake/occupancy_grid"), "automsgs.msgs.map_msgs.OccupancyGrid"},
      {QStringLiteral("/fake/point_cloud2"), "automsgs.msgs.sensor_msgs.PointCloud2"},
      {QStringLiteral("/fake/point_cloud"), "automsgs.msgs.sensor_msgs.PointCloud2"},
      {QStringLiteral("/fake/pose"), "automsgs.msgs.geometry_msgs.PoseStamped"},
      {QStringLiteral("/fake/pose_array"), "automsgs.msgs.geometry_msgs.PoseArray"},
      {QStringLiteral("/fake/image"), "automsgs.msgs.sensor_msgs.Image"},
      {QStringLiteral("/fake/twist"), "automsgs.msgs.geometry_msgs.TwistStamped"},
      {QStringLiteral("/fake/wrench"), "automsgs.msgs.geometry_msgs.WrenchStamped"},
      {QStringLiteral("/fake/grid_cells"), "automsgs.msgs.map_msgs.GridCells"},
      {QStringLiteral("/fake/point"), "automsgs.msgs.geometry_msgs.PointStamped"},
      {QStringLiteral("/fake/polygon"), "automsgs.msgs.geometry_msgs.PolygonStamped"},
      {QStringLiteral("/fake/range"), "automsgs.msgs.sensor_msgs.Range"},
      {QStringLiteral("/fake/pose_with_covariance"),
       "automsgs.msgs.geometry_msgs.PoseWithCovarianceStamped"},
      {QStringLiteral("/fake/camera_info"), "automsgs.msgs.sensor_msgs.CameraInfo"},
      {QStringLiteral("/fake/imu"), "automsgs.msgs.sensor_msgs.Imu"},
      {QStringLiteral("/fake/accel"), "automsgs.msgs.geometry_msgs.AccelStamped"},
      {QStringLiteral("/fake/plot/sine"), "automsgs.msgs.std_msgs.Float64"},
      {QStringLiteral("/fake/plot/cosine"), "automsgs.msgs.std_msgs.Float64"},
      {QStringLiteral("/fake/plot/cmd"), "automsgs.msgs.geometry_msgs.Twist"},
      {QStringLiteral("/cmd_vel"), "automsgs.msgs.geometry_msgs.Twist"},
  };
  return kTutorialChannels;
}

std::string DefaultMessageTypeForChannel(const QString& channel) {
  for (const auto& entry : TutorialChannelMessageTypes()) {
    if (channel.compare(entry.first, Qt::CaseInsensitive) == 0) {
      return entry.second;
    }
  }
  return {};
}

QString NormalizeKnownChannel(const QString& channel,
                              const QStringList& known_channels) {
  for (const QString& candidate : known_channels) {
    if (channel.compare(candidate, Qt::CaseInsensitive) == 0) {
      return candidate;
    }
  }
  return channel;
}

}  // namespace

QString CombinedPlotValuePath(const QString& channel, const QString& field_path) {
  const QString trimmed_channel = channel.trimmed();
  const QString trimmed_field = field_path.trimmed();
  if (trimmed_channel.isEmpty()) {
    return trimmed_field;
  }
  if (trimmed_field.isEmpty()) {
    return trimmed_channel;
  }
  return trimmed_channel + QLatin1Char('.') + trimmed_field;
}

bool SplitPlotValuePath(const QString& combined, const QStringList& known_channels,
                        QString* channel, QString* field_path) {
  if (channel == nullptr || field_path == nullptr) {
    return false;
  }
  const QString path = combined.trimmed();
  if (path.isEmpty()) {
    *channel = QString();
    *field_path = QString();
    return true;
  }

  if (path.startsWith(QLatin1Char('/'))) {
    const int dot = path.indexOf(QLatin1Char('.'));
    if (dot < 0) {
      *channel = NormalizeKnownChannel(path, known_channels);
      *field_path = QString();
      return true;
    }
    *channel = NormalizeKnownChannel(path.left(dot), known_channels);
    *field_path = path.mid(dot + 1);
    return true;
  }

  *channel = QString();
  *field_path = path;
  return false;
}

bool IsPlottablePlotValuePath(const QString& channel, const QString& field_path) {
  return !channel.trimmed().isEmpty() && !field_path.trimmed().isEmpty();
}

std::string MessageTypeForChannel(common::VisualizationManager* manager,
                                  const QString& channel) {
  if (channel.trimmed().isEmpty()) {
    return {};
  }
  std::string discovered;
  if (manager != nullptr) {
    for (const integration::ChannelInfo& info : manager->channels()) {
      if (QString::fromStdString(info.channel_name).compare(channel, Qt::CaseInsensitive) == 0 &&
          !info.message_type.empty()) {
        discovered = info.message_type;
        break;
      }
    }
  }
  const std::string fallback = DefaultMessageTypeForChannel(channel);
  const auto type_has_fields = [](const std::string& type) {
    if (type.empty()) {
      return false;
    }
    const std::string normalized = commsgs::NormalizeMessageType(type);
    return !PlotNextLevelFieldPaths(normalized, QString()).isEmpty() ||
           !PlotBrowsePathsForMessageType(normalized).isEmpty();
  };

  if (type_has_fields(discovered)) {
    return commsgs::NormalizeMessageType(discovered);
  }
  if (type_has_fields(fallback)) {
    return fallback;
  }
  if (!discovered.empty()) {
    return commsgs::NormalizeMessageType(discovered);
  }
  return fallback;
}

QStringList AllKnownChannels(common::VisualizationManager* manager) {
  QStringList out;
  if (manager != nullptr) {
    for (const integration::ChannelInfo& info : manager->channels()) {
      out.push_back(QString::fromStdString(info.channel_name));
    }
  }
  for (const auto& entry : TutorialChannelMessageTypes()) {
    if (!out.contains(entry.first, Qt::CaseInsensitive)) {
      out.push_back(entry.first);
    }
  }
  out.sort(Qt::CaseInsensitive);
  out.removeDuplicates();
  return out;
}

QStringList ChannelNameSuggestions(common::VisualizationManager* manager,
                                   const QString& prefix) {
  QStringList out;
  const QString needle = prefix.trimmed();
  for (const QString& ch : AllKnownChannels(manager)) {
    if (needle.isEmpty() || ch.startsWith(needle, Qt::CaseInsensitive)) {
      out.push_back(ch);
    }
  }
  out.sort(Qt::CaseInsensitive);
  out.removeDuplicates();
  return out;
}

QStringList PlotValuePathSuggestions(common::VisualizationManager* manager,
                                     const QString& typed_prefix) {
  const QString prefix = typed_prefix.trimmed();
  if (prefix.isEmpty()) {
    return ChannelNameSuggestions(manager, QString());
  }

  const QStringList channels = AllKnownChannels(manager);

  QString channel;
  QString field_path;
  SplitPlotValuePath(prefix, channels, &channel, &field_path);

  if (channel.isEmpty()) {
    return ChannelNameSuggestions(manager, prefix);
  }

  const std::string message_type = MessageTypeForChannel(manager, channel);
  if (message_type.empty()) {
    if (prefix.endsWith(QLatin1Char('.'))) {
      return QStringList{};
    }
    return ChannelNameSuggestions(manager, prefix);
  }

  if (prefix.endsWith(QLatin1Char('.'))) {
    QString parent_field = field_path;
    while (parent_field.endsWith(QLatin1Char('.'))) {
      parent_field.chop(1);
    }
    const QStringList all_relative = PlotAllFieldPathsForMessageType(message_type);
    QStringList out;
    for (const QString& relative : all_relative) {
      bool include = false;
      if (parent_field.isEmpty()) {
        include = true;
      } else if (relative.compare(parent_field, Qt::CaseInsensitive) == 0 ||
                 relative.startsWith(parent_field + QLatin1Char('.'),
                                     Qt::CaseInsensitive)) {
        include = true;
      }
      if (!include) {
        continue;
      }
      out.push_back(channel + QLatin1Char('.') + relative);
      if (out.size() >= 80) {
        break;
      }
    }
    out.sort(Qt::CaseInsensitive);
    return out;
  }

  // Channel resolved without a trailing dot: show next-level fields or channel matches.
  if (field_path.isEmpty()) {
    if (prefix.compare(channel, Qt::CaseInsensitive) == 0) {
      const QStringList next_segments = PlotNextLevelFieldPaths(message_type, QString());
      QStringList out;
      for (const QString& segment : next_segments) {
        out.push_back(channel + QLatin1Char('.') + segment);
      }
      out.sort(Qt::CaseInsensitive);
      return out;
    }
    return ChannelNameSuggestions(manager, prefix);
  }

  // Exact field prefix without trailing dot: show next-level children (e.g. /fake/path.poses).
  if (!field_path.isEmpty() && !prefix.endsWith(QLatin1Char('.'))) {
    const QString expected = channel + QLatin1Char('.') + field_path;
    if (prefix.compare(expected, Qt::CaseInsensitive) == 0) {
      const QStringList next_segments = PlotNextLevelFieldPaths(message_type, field_path);
      QStringList out;
      for (const QString& segment : next_segments) {
        out.push_back(expected + QLatin1Char('.') + segment);
      }
      out.sort(Qt::CaseInsensitive);
      return out;
    }
  }

  // Partial field path: return matching leaf/browse paths only (bounded).
  const QStringList all = PlotBrowsePathsForChannel(channel, message_type);
  QStringList out;
  for (const QString& path : all) {
    if (path.startsWith(prefix, Qt::CaseInsensitive)) {
      out.push_back(path);
      if (out.size() >= 40) {
        break;
      }
    }
  }
  out.sort(Qt::CaseInsensitive);
  return out;
}

QStringList PlotBrowsePathsForChannel(const QString& channel,
                                      const std::string& message_type) {
  QStringList paths;
  const QString trimmed_channel = channel.trimmed();
  if (trimmed_channel.isEmpty()) {
    return paths;
  }
  paths.push_back(trimmed_channel);
  const QStringList relative = PlotBrowsePathsForMessageType(message_type);
  for (const QString& entry : relative) {
    if (entry.isEmpty()) {
      continue;
    }
    paths.push_back(trimmed_channel + QLatin1Char('.') + entry);
  }
  paths.sort(Qt::CaseInsensitive);
  paths.removeDuplicates();
  return paths;
}

QStringList AllPlotBrowsePaths(common::VisualizationManager* manager) {
  QStringList paths;
  if (manager == nullptr) {
    return paths;
  }
  for (const integration::ChannelInfo& info : manager->channels()) {
    paths.append(PlotBrowsePathsForChannel(QString::fromStdString(info.channel_name),
                                           info.message_type));
  }
  paths.sort(Qt::CaseInsensitive);
  paths.removeDuplicates();
  return paths;
}

}  // namespace plot
}  // namespace autoviz
