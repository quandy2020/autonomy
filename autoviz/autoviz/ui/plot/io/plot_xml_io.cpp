/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/io/plot_xml_io.hpp"

#include <QColor>
#include <QDomDocument>
#include <QDomElement>
#include <QXmlStreamWriter>

namespace autoviz {
namespace plot {
namespace {

QString ChildText(const QDomElement& parent, const QString& tag) {
  const QDomElement child = parent.firstChildElement(tag);
  return child.isNull() ? QString() : child.text().trimmed();
}

double ChildDouble(const QDomElement& parent, const QString& tag,
                   double fallback) {
  const QString text = ChildText(parent, tag);
  if (text.isEmpty()) {
    return fallback;
  }
  bool ok = false;
  const double value = text.toDouble(&ok);
  return ok ? value : fallback;
}

bool ChildBool(const QDomElement& parent, const QString& tag, bool fallback) {
  const QString text = ChildText(parent, tag).toLower();
  if (text.isEmpty()) {
    return fallback;
  }
  return text == QLatin1String("true") || text == QLatin1String("1") ||
         text == QLatin1String("on");
}

QColor ParseColor(const QDomElement& color_el, const QColor& fallback) {
  if (color_el.isNull()) {
    return fallback;
  }
  const QString named = ChildText(color_el, QStringLiteral("rgb"));
  if (!named.isEmpty()) {
    const QColor color(named);
    if (color.isValid()) {
      return color;
    }
  }
  const int r = static_cast<int>(ChildDouble(color_el, QStringLiteral("r"), -1));
  const int g = static_cast<int>(ChildDouble(color_el, QStringLiteral("g"), -1));
  const int b = static_cast<int>(ChildDouble(color_el, QStringLiteral("b"), -1));
  if (r >= 0 && g >= 0 && b >= 0) {
    return QColor(r, g, b);
  }
  const QString custom = ChildText(color_el, QStringLiteral("custom"));
  if (!custom.isEmpty()) {
    const QColor color(custom);
    if (color.isValid()) {
      return color;
    }
  }
  return fallback;
}

PlotSeriesConfig ParseCurve(const QDomElement& curve) {
  PlotSeriesConfig series;
  series.label = ChildText(curve, QStringLiteral("title"));
  series.channel = ChildText(curve, QStringLiteral("topic"));
  if (series.channel.isEmpty()) {
    series.channel = ChildText(curve, QStringLiteral("channel"));
  }
  series.channel_pattern = ChildText(curve, QStringLiteral("channel_pattern"));
  series.field_path = ChildText(curve, QStringLiteral("field"));
  if (series.field_path.isEmpty()) {
    const QDomElement axes = curve.firstChildElement(QStringLiteral("axes"));
    const QDomElement y_axis = axes.firstChildElement(QStringLiteral("y_axis"));
    series.field_path = ChildText(y_axis, QStringLiteral("field"));
    const QDomElement x_axis = axes.firstChildElement(QStringLiteral("x_axis"));
    series.x_field_path = ChildText(x_axis, QStringLiteral("field"));
    if (series.channel.isEmpty()) {
      series.channel = ChildText(y_axis, QStringLiteral("topic"));
    }
  }
  series.x_field_path = series.x_field_path.isEmpty()
                            ? ChildText(curve, QStringLiteral("x_field_path"))
                            : series.x_field_path;
  series.x_channel = ChildText(curve, QStringLiteral("x_channel"));
  if (series.x_channel.isEmpty()) {
    const QDomElement axes = curve.firstChildElement(QStringLiteral("axes"));
    const QDomElement x_axis = axes.firstChildElement(QStringLiteral("x_axis"));
    series.x_channel = ChildText(x_axis, QStringLiteral("topic"));
  }
  series.custom_timestamp_path =
      ChildText(curve, QStringLiteral("custom_timestamp_path"));
  series.line_size = ChildText(curve, QStringLiteral("line_size"));
  if (series.line_size.isEmpty()) {
    series.line_size = QStringLiteral("auto");
  }
  series.curve_style = static_cast<PlotCurveStyle>(static_cast<int>(
      ChildDouble(curve, QStringLiteral("curve_style"),
                  static_cast<double>(series.curve_style))));
  series.show_line = ChildBool(curve, QStringLiteral("show_line"), true);
  series.show_points = ChildBool(curve, QStringLiteral("show_points"), true);
  series.point_size = ChildDouble(curve, QStringLiteral("point_size"), 3.0);
  series.y_scale = ChildDouble(curve, QStringLiteral("y_scale"), 1.0);
  series.y_offset = ChildDouble(curve, QStringLiteral("y_offset"), 0.0);
  series.use_right_y = ChildBool(curve, QStringLiteral("use_right_y"), false);
  series.enabled = ChildBool(curve, QStringLiteral("enabled"), true);
  series.color =
      ParseColor(curve.firstChildElement(QStringLiteral("color")), series.color);
  const QString style = ChildText(curve, QStringLiteral("style")).toLower();
  if (style.contains(QStringLiteral("steps"))) {
    series.curve_style = PlotCurveStyle::kSteps;
    series.show_line = true;
  } else if (style.contains(QStringLiteral("sticks"))) {
    series.curve_style = PlotCurveStyle::kSticks;
    series.show_line = true;
  }
  if (series.label.isEmpty()) {
    series.label = series.channel;
  }
  return series;
}

void WriteCurve(QXmlStreamWriter* xml, int index, const PlotSeriesConfig& series) {
  xml->writeStartElement(QStringLiteral("curve_%1").arg(index));
  xml->writeTextElement(QStringLiteral("title"), series.label);
  xml->writeTextElement(QStringLiteral("topic"), series.channel);
  xml->writeTextElement(QStringLiteral("channel"), series.channel);
  xml->writeTextElement(QStringLiteral("channel_pattern"), series.channel_pattern);
  xml->writeTextElement(QStringLiteral("field"), series.field_path);
  xml->writeTextElement(QStringLiteral("x_field_path"), series.x_field_path);
  xml->writeTextElement(QStringLiteral("x_channel"), series.x_channel);
  xml->writeTextElement(QStringLiteral("custom_timestamp_path"),
                        series.custom_timestamp_path);
  xml->writeTextElement(QStringLiteral("line_size"), series.line_size);
  xml->writeTextElement(QStringLiteral("curve_style"),
                        QString::number(static_cast<int>(series.curve_style)));
  xml->writeTextElement(QStringLiteral("show_line"),
                        series.show_line ? QStringLiteral("true")
                                         : QStringLiteral("false"));
  xml->writeTextElement(QStringLiteral("show_points"),
                        series.show_points ? QStringLiteral("true")
                                           : QStringLiteral("false"));
  xml->writeTextElement(QStringLiteral("point_size"),
                        QString::number(series.point_size));
  xml->writeTextElement(QStringLiteral("y_scale"),
                        QString::number(series.y_scale));
  xml->writeTextElement(QStringLiteral("y_offset"),
                        QString::number(series.y_offset));
  xml->writeTextElement(QStringLiteral("use_right_y"),
                        series.use_right_y ? QStringLiteral("true")
                                           : QStringLiteral("false"));
  xml->writeTextElement(QStringLiteral("enabled"),
                        series.enabled ? QStringLiteral("true")
                                       : QStringLiteral("false"));
  xml->writeStartElement(QStringLiteral("color"));
  xml->writeTextElement(QStringLiteral("custom"), series.color.name());
  xml->writeTextElement(QStringLiteral("r"),
                        QString::number(series.color.red()));
  xml->writeTextElement(QStringLiteral("g"),
                        QString::number(series.color.green()));
  xml->writeTextElement(QStringLiteral("b"),
                        QString::number(series.color.blue()));
  xml->writeEndElement();
  xml->writeStartElement(QStringLiteral("axes"));
  xml->writeStartElement(QStringLiteral("y_axis"));
  xml->writeTextElement(QStringLiteral("topic"), series.channel);
  xml->writeTextElement(QStringLiteral("field"), series.field_path);
  xml->writeEndElement();
  if (!series.x_field_path.isEmpty()) {
    xml->writeStartElement(QStringLiteral("x_axis"));
    xml->writeTextElement(
        QStringLiteral("topic"),
        series.x_channel.isEmpty() ? series.channel : series.x_channel);
    xml->writeTextElement(QStringLiteral("field"), series.x_field_path);
    xml->writeEndElement();
  }
  xml->writeEndElement();
  xml->writeEndElement();
}

}  // namespace

QByteArray ExportPlotXml(const PlotPanelConfig& config) {
  QByteArray bytes;
  QXmlStreamWriter xml(&bytes);
  xml.setAutoFormatting(true);
  xml.writeStartDocument();
  xml.writeStartElement(QStringLiteral("rqt_multiplot"));
  xml.writeStartElement(QStringLiteral("plots"));
  xml.writeStartElement(QStringLiteral("plot_0"));
  xml.writeTextElement(QStringLiteral("title"), config.title);
  xml.writeTextElement(QStringLiteral("x_axis_mode"),
                       QString::number(static_cast<int>(config.x_axis_mode)));
  xml.writeTextElement(QStringLiteral("message_path_mode"),
                       QString::number(static_cast<int>(config.message_path_mode)));
  xml.writeTextElement(QStringLiteral("sync_group"), config.sync_group);
  xml.writeTextElement(QStringLiteral("sync_with_other_plots"),
                       config.sync_with_other_plots ? QStringLiteral("true")
                                                    : QStringLiteral("false"));
  xml.writeTextElement(QStringLiteral("sync_y_axis"),
                       config.sync_y_axis ? QStringLiteral("true")
                                         : QStringLiteral("false"));
  xml.writeTextElement(QStringLiteral("sync_cursor"),
                       config.sync_cursor ? QStringLiteral("true")
                                         : QStringLiteral("false"));
  xml.writeTextElement(QStringLiteral("x_window_sec"),
                       QString::number(config.x_window_sec));
  xml.writeTextElement(QStringLiteral("max_points_per_series"),
                       QString::number(config.max_points_per_series));
  xml.writeTextElement(QStringLiteral("keep_ratio_xy"),
                       config.keep_ratio_xy ? QStringLiteral("true")
                                            : QStringLiteral("false"));
  xml.writeTextElement(QStringLiteral("show_reference_tracker"),
                       config.show_reference_tracker ? QStringLiteral("true")
                                                     : QStringLiteral("false"));
  xml.writeTextElement(QStringLiteral("reference_tracker_sec"),
                       QString::number(config.reference_tracker_sec));
  xml.writeTextElement(QStringLiteral("y_auto_scale"),
                       config.y_auto_scale ? QStringLiteral("true")
                                          : QStringLiteral("false"));
  xml.writeTextElement(QStringLiteral("y_min"), QString::number(config.y_min));
  xml.writeTextElement(QStringLiteral("y_max"), QString::number(config.y_max));
  xml.writeStartElement(QStringLiteral("curves"));
  for (int i = 0; i < config.series.size(); ++i) {
    WriteCurve(&xml, i, config.series.at(i));
  }
  xml.writeEndElement();
  xml.writeEndElement();
  xml.writeEndElement();
  xml.writeEndElement();
  xml.writeEndDocument();
  return bytes;
}

bool ImportPlotXml(const QByteArray& xml, PlotPanelConfig* config,
                   QString* error) {
  if (config == nullptr) {
    if (error != nullptr) {
      *error = QStringLiteral("Null config");
    }
    return false;
  }
  QDomDocument doc;
  QString parse_error;
  int line = 0;
  int column = 0;
  if (!doc.setContent(xml, &parse_error, &line, &column)) {
    if (error != nullptr) {
      *error = QStringLiteral("%1 (line %2, column %3)")
                   .arg(parse_error)
                   .arg(line)
                   .arg(column);
    }
    return false;
  }
  QDomElement root = doc.documentElement();
  if (root.tagName() != QLatin1String("rqt_multiplot") &&
      root.tagName() != QLatin1String("autoviz_plot")) {
    if (error != nullptr) {
      *error = QStringLiteral("Expected <rqt_multiplot> root");
    }
    return false;
  }
  PlotPanelConfig imported = *config;
  imported.series.clear();
  QDomElement plots = root.firstChildElement(QStringLiteral("plots"));
  QDomElement plot = plots.isNull() ? root.firstChildElement(QStringLiteral("plot_0"))
                                    : plots.firstChildElement();
  if (plot.isNull()) {
    plot = root;
  }
  const QString title = ChildText(plot, QStringLiteral("title"));
  if (!title.isEmpty()) {
    imported.title = title;
  }
  imported.x_axis_mode = static_cast<PlotXAxisMode>(static_cast<int>(
      ChildDouble(plot, QStringLiteral("x_axis_mode"),
                  static_cast<double>(imported.x_axis_mode))));
  imported.message_path_mode = static_cast<PlotMessagePathMode>(static_cast<int>(
      ChildDouble(plot, QStringLiteral("message_path_mode"),
                  static_cast<double>(imported.message_path_mode))));
  imported.sync_group = ChildText(plot, QStringLiteral("sync_group"));
  imported.sync_with_other_plots =
      ChildBool(plot, QStringLiteral("sync_with_other_plots"),
                !imported.sync_group.isEmpty());
  imported.sync_y_axis = ChildBool(plot, QStringLiteral("sync_y_axis"), false);
  imported.sync_cursor = ChildBool(plot, QStringLiteral("sync_cursor"), true);
  imported.x_window_sec =
      ChildDouble(plot, QStringLiteral("x_window_sec"), imported.x_window_sec);
  imported.max_points_per_series = static_cast<int>(ChildDouble(
      plot, QStringLiteral("max_points_per_series"),
      static_cast<double>(imported.max_points_per_series)));
  imported.keep_ratio_xy =
      ChildBool(plot, QStringLiteral("keep_ratio_xy"), imported.keep_ratio_xy);
  imported.show_reference_tracker = ChildBool(
      plot, QStringLiteral("show_reference_tracker"),
      imported.show_reference_tracker);
  imported.reference_tracker_sec =
      ChildDouble(plot, QStringLiteral("reference_tracker_sec"),
                  imported.reference_tracker_sec);
  imported.y_auto_scale =
      ChildBool(plot, QStringLiteral("y_auto_scale"), imported.y_auto_scale);
  imported.y_min = ChildDouble(plot, QStringLiteral("y_min"), imported.y_min);
  imported.y_max = ChildDouble(plot, QStringLiteral("y_max"), imported.y_max);

  QDomElement curves = plot.firstChildElement(QStringLiteral("curves"));
  if (curves.isNull()) {
    curves = root.firstChildElement(QStringLiteral("curves"));
  }
  for (QDomElement curve = curves.firstChildElement(); !curve.isNull();
       curve = curve.nextSiblingElement()) {
    if (!curve.tagName().startsWith(QLatin1String("curve"))) {
      continue;
    }
    const PlotSeriesConfig series = ParseCurve(curve);
    if (series.channel.isEmpty() && series.channel_pattern.isEmpty() &&
        series.field_path.isEmpty()) {
      continue;
    }
    imported.series.push_back(series);
  }
  if (imported.series.isEmpty()) {
    if (error != nullptr) {
      *error = QStringLiteral("No curves found in plot XML");
    }
    return false;
  }
  *config = imported;
  return true;
}

}  // namespace plot
}  // namespace autoviz
