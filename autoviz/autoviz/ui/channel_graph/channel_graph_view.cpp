/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/channel_graph/channel_graph_view.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <unordered_map>
#include <vector>

#include <QBrush>
#include <QFont>
#include <QFontMetricsF>
#include <QGraphicsObject>
#include <QGraphicsScene>
#include <QGraphicsSceneHoverEvent>
#include <QLinearGradient>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPainterPathStroker>
#include <QPen>
#include <QPolygonF>
#include <QRadialGradient>
#include <QSet>
#include <QShowEvent>
#include <QStyle>
#include <QStyleOptionGraphicsItem>
#include <QTimer>
#include <QWheelEvent>

#include "autoviz/integration/channel_stats_registry.hpp"

namespace autoviz {
namespace channel_graph {
namespace {

constexpr double kMinZoom = 0.15;
constexpr double kMaxZoom = 4.0;
constexpr double kNodeDiameterMin = 76.0;
constexpr double kNodeDiameterMax = 108.0;
constexpr double kChannelWidthMin = 280.0;
constexpr double kChannelWidthMax = 420.0;
constexpr double kChannelHeight = 46.0;
constexpr double kServiceWidth = 168.0;
constexpr double kServiceHeight = 148.0;
constexpr double kTextPadding = 10.0;
constexpr double kArrowLength = 11.0;
constexpr double kArrowWidth = 5.5;
constexpr double kConnectionSpread = 13.0;

enum class GraphVisualMode { kNormal, kEmphasized, kDimmed };

bool IsPubSubEdge(integration::GraphEdgeKind kind) {
  return kind == integration::GraphEdgeKind::kPublish ||
         kind == integration::GraphEdgeKind::kSubscribe;
}

bool IsServiceEdge(integration::GraphEdgeKind kind) {
  return kind == integration::GraphEdgeKind::kServiceServer ||
         kind == integration::GraphEdgeKind::kServiceClient;
}

/** Edges that show traveling-packet flow when a vertex is focused. */
bool IsAnimatedFlowEdge(integration::GraphEdgeKind kind) {
  return IsPubSubEdge(kind) || IsServiceEdge(kind);
}

/** Which edge kinds belong to a focused vertex of the given kind. */
bool IsFocusRelatedEdge(integration::GraphVertexKind focus_kind,
                        integration::GraphEdgeKind edge_kind) {
  switch (focus_kind) {
    case integration::GraphVertexKind::kNode:
    case integration::GraphVertexKind::kChannel:
      return IsPubSubEdge(edge_kind);
    case integration::GraphVertexKind::kService:
      return IsServiceEdge(edge_kind);
  }
  return false;
}

struct NodeLayoutSlot {
  std::string id;
  double preferred_y = 0.0;
  int pub_count = 0;
  int sub_count = 0;
};

double MedianOf(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
  return values[values.size() / 2];
}

/** Pack nodes top-to-bottom with a minimum gap, biased toward preferred Y. */
std::vector<double> PackPreferredPositions(std::vector<double> preferred,
                                           double min_gap) {
  if (preferred.empty()) {
    return {};
  }
  std::vector<std::size_t> order(preferred.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
    return preferred[a] < preferred[b];
  });

  std::vector<double> packed(preferred.size(), 0.0);
  packed[order.front()] = preferred[order.front()];
  for (std::size_t i = 1; i < order.size(); ++i) {
    const std::size_t idx = order[i];
    const std::size_t prev = order[i - 1];
    packed[idx] = std::max(preferred[idx], packed[prev] + min_gap);
  }

  // Shift the packed column so its median stays near the preferred median.
  std::vector<double> preferred_sorted = preferred;
  std::vector<double> packed_sorted;
  packed_sorted.reserve(order.size());
  for (std::size_t idx : order) {
    packed_sorted.push_back(packed[idx]);
  }
  const double shift = MedianOf(std::move(preferred_sorted)) - MedianOf(std::move(packed_sorted));
  for (double& value : packed) {
    value += shift;
  }

  // Re-resolve collisions after the shift.
  for (std::size_t i = 1; i < order.size(); ++i) {
    const std::size_t idx = order[i];
    const std::size_t prev = order[i - 1];
    packed[idx] = std::max(packed[idx], packed[prev] + min_gap);
  }
  return packed;
}

ChannelGraphLayoutOptions ComputeAdaptiveLayoutOptions(
    int channel_count, int node_count, int service_count, int edge_count,
    double max_channel_half_width, double viewport_width,
    VertexArrangeMode channel_mode, VertexArrangeMode service_mode) {
  ChannelGraphLayoutOptions options;
  const int channels = std::max(channel_count, 0);
  const int services = std::max(service_count, 0);

  if (channel_mode == VertexArrangeMode::kColumn || channels <= 1) {
    options.channel_columns = 1;
    options.channel_rows = std::max(channels, 1);
  } else {
    // Prefer a readable height; wrap channels into multiple columns instead of
    // making one ultra-tall stack.
    const int target_channel_rows =
        channels <= 12
            ? std::max(channels, 1)
            : std::clamp(static_cast<int>(std::ceil(std::sqrt(channels * 2.2))),
                         10, 16);
    options.channel_columns = std::max(
        1, static_cast<int>(std::ceil(static_cast<double>(channels) /
                                      static_cast<double>(target_channel_rows))));
    options.channel_rows = static_cast<int>(
        std::ceil(static_cast<double>(channels) /
                  static_cast<double>(options.channel_columns)));
  }

  options.channel_row_gap = kChannelHeight + 32.0;
  options.node_row_gap = kNodeDiameterMax + 48.0;
  options.section_gap = 260.0;

  const double channel_half =
      std::max(kChannelWidthMin * 0.5, max_channel_half_width);
  options.channel_col_gap = channel_half * 2.0 + 64.0;

  const double channel_block_width =
      (options.channel_columns - 1) * options.channel_col_gap;
  options.channel_block_left_x = -channel_block_width * 0.5;
  options.channel_block_right_x = channel_block_width * 0.5;
  options.channel_column_x = 0.0;

  const double side_lane =
      std::clamp(220.0 + node_count * 12.0 + edge_count * 0.15, 240.0, 420.0);
  options.writer_column_x =
      options.channel_block_left_x - channel_half - side_lane;
  options.reader_column_x =
      options.channel_block_right_x + channel_half + side_lane;
  options.both_writer_column_x =
      (options.writer_column_x + options.channel_block_left_x) * 0.5;
  options.both_reader_column_x =
      (options.reader_column_x + options.channel_block_right_x) * 0.5;

  // Services sit under the channel band.
  const double channel_band_height =
      std::max(0, options.channel_rows - 1) * options.channel_row_gap;
  options.service_origin_y = channel_band_height + options.section_gap;
  options.service_col_gap = kServiceWidth + 56.0;
  options.service_row_gap = kServiceHeight + 48.0;

  if (service_mode == VertexArrangeMode::kColumn || services <= 1) {
    options.service_columns = 1;
  } else {
    const double usable_width = std::max(
        viewport_width * 0.92,
        options.reader_column_x - options.writer_column_x + 200.0);
    const int max_service_cols_by_width = std::max(
        1, static_cast<int>(usable_width / options.service_col_gap));
    if (services <= 4) {
      options.service_columns = std::max(services, 1);
    } else {
      options.service_columns = std::clamp(
          static_cast<int>(std::ceil(std::sqrt(services * 1.35))), 3,
          std::min(8, max_service_cols_by_width));
    }
  }
  const double service_block_width =
      (options.service_columns - 1) * options.service_col_gap;
  options.service_origin_x = -service_block_width * 0.5;
  return options;
}

QPointF ChannelGridPosition(const ChannelGraphLayoutOptions& layout, int index) {
  const int cols = std::max(1, layout.channel_columns);
  const int col = index % cols;
  const int row = index / cols;
  return QPointF(layout.channel_block_left_x + col * layout.channel_col_gap,
                 row * layout.channel_row_gap);
}

QPointF ServiceGridPosition(const ChannelGraphLayoutOptions& layout, int index) {
  const int cols = std::max(1, layout.service_columns);
  const int col = index % cols;
  const int row = index / cols;
  return QPointF(layout.service_origin_x + col * layout.service_col_gap,
                 layout.service_origin_y + row * layout.service_row_gap);
}

QColor ColorForKind(integration::GraphVertexKind kind) {
  switch (kind) {
    case integration::GraphVertexKind::kNode:
      return QColor(66, 133, 244);
    case integration::GraphVertexKind::kChannel:
      return QColor(156, 39, 176);
    case integration::GraphVertexKind::kService:
      return QColor(229, 57, 53);
  }
  return QColor(120, 120, 120);
}

QColor ColorForEdge(integration::GraphEdgeKind kind) {
  switch (kind) {
    case integration::GraphEdgeKind::kPublish:
      return QColor(66, 133, 244, 210);
    case integration::GraphEdgeKind::kSubscribe:
      return QColor(156, 39, 176, 210);
    case integration::GraphEdgeKind::kRelay:
      return QColor(96, 125, 139, 190);
    case integration::GraphEdgeKind::kServiceServer:
      return QColor(229, 57, 53, 220);
    case integration::GraphEdgeKind::kServiceClient:
      return QColor(255, 112, 67, 220);
  }
  return QColor(120, 120, 120, 180);
}

Qt::PenStyle PenStyleForEdge(integration::GraphEdgeKind kind) {
  switch (kind) {
    case integration::GraphEdgeKind::kRelay:
      return Qt::DashLine;
    case integration::GraphEdgeKind::kServiceClient:
      return Qt::DotLine;
    default:
      return Qt::SolidLine;
  }
}

QString EdgeKindLabel(integration::GraphEdgeKind kind) {
  switch (kind) {
    case integration::GraphEdgeKind::kPublish:
      return QStringLiteral("pub");
    case integration::GraphEdgeKind::kSubscribe:
      return QStringLiteral("sub");
    case integration::GraphEdgeKind::kRelay:
      return QStringLiteral("relay");
    case integration::GraphEdgeKind::kServiceServer:
      return QStringLiteral("srv");
    case integration::GraphEdgeKind::kServiceClient:
      return QStringLiteral("cli");
  }
  return QString();
}

QString KindBadge(integration::GraphVertexKind kind) {
  switch (kind) {
    case integration::GraphVertexKind::kNode:
      return QStringLiteral("Node");
    case integration::GraphVertexKind::kChannel:
      return QStringLiteral("Channel");
    case integration::GraphVertexKind::kService:
      return QStringLiteral("Service");
  }
  return QString();
}

QString ShortPathTail(const QString& path) {
  const int slash = path.lastIndexOf(QLatin1Char('/'));
  if (slash >= 0 && slash + 1 < path.size()) {
    return path.mid(slash + 1);
  }
  return path;
}

QString ShortTypeName(const QString& type_name) {
  if (type_name.isEmpty()) {
    return QString();
  }
  const int dot = type_name.lastIndexOf(QLatin1Char('.'));
  if (dot >= 0 && dot + 1 < type_name.size()) {
    return type_name.mid(dot + 1);
  }
  return ShortPathTail(type_name);
}

QString FormatChannelHz(double hz) {
  if (hz <= 0.0) {
    return QStringLiteral("— Hz");
  }
  if (hz >= 100.0) {
    return QString::number(hz, 'f', 0) + QStringLiteral(" Hz");
  }
  if (hz >= 10.0) {
    return QString::number(hz, 'f', 1) + QStringLiteral(" Hz");
  }
  return QString::number(hz, 'f', 2) + QStringLiteral(" Hz");
}

QFont MakeTitleFont() {
  QFont font;
  font.setPointSize(9);
  font.setBold(true);
  return font;
}

QFont MakeDetailFont() {
  QFont font;
  font.setPointSize(8);
  return font;
}

QFont MakeKindFont() {
  QFont font;
  font.setPointSize(7);
  font.setBold(true);
  return font;
}

QPainterPath CapsulePath(const QRectF& rect) {
  QPainterPath path;
  const double radius = rect.height() * 0.5;
  path.addRoundedRect(rect, radius, radius);
  return path;
}

QPainterPath CirclePath(const QRectF& rect) {
  QPainterPath path;
  path.addEllipse(rect);
  return path;
}

QPainterPath HexagonPath(const QRectF& rect) {
  const QPointF center = rect.center();
  const double rx = rect.width() * 0.5;
  const double ry = rect.height() * 0.5;
  QPolygonF hex;
  for (int i = 0; i < 6; ++i) {
    const double angle = M_PI / 3.0 * static_cast<double>(i) - M_PI / 6.0;
    hex << center + QPointF(std::cos(angle) * rx, std::sin(angle) * ry);
  }
  QPainterPath path;
  path.addPolygon(hex);
  return path;
}

QPointF CubicBezierPoint(const QPointF& p0, const QPointF& p1, const QPointF& p2,
                         const QPointF& p3, double t) {
  const double u = 1.0 - t;
  const double uu = u * u;
  const double tt = t * t;
  return uu * u * p0 + 3.0 * uu * t * p1 + 3.0 * u * tt * p2 + tt * t * p3;
}

void DrawArrowHead(QPainter* painter, const QPointF& tip, const QPointF& direction,
                   const QColor& color) {
  const double length = std::hypot(direction.x(), direction.y());
  if (length < 0.001) {
    return;
  }
  const QPointF unit = direction / length;
  const QPointF ortho(-unit.y(), unit.x());
  const QPointF base = tip - unit * kArrowLength;
  QPolygonF head;
  head << tip << base + ortho * kArrowWidth << base - ortho * kArrowWidth;
  painter->setPen(Qt::NoPen);
  painter->setBrush(color);
  painter->drawPolygon(head);
}

class GraphEdgeItem;

class GraphVertexItem : public QGraphicsObject {
 public:
  GraphVertexItem(const QString& id, integration::GraphVertexKind kind,
                  const QString& label, const QString& detail,
                  QGraphicsItem* parent = nullptr)
      : QGraphicsObject(parent),
        id_(id),
        kind_(kind),
        full_label_(label),
        full_detail_(detail),
        kind_font_(MakeKindFont()),
        title_font_(MakeTitleFont()),
        detail_font_(MakeDetailFont()) {
    setFlags(QGraphicsItem::ItemIsMovable | QGraphicsItem::ItemIsSelectable |
             QGraphicsItem::ItemSendsGeometryChanges);
    setCursor(Qt::OpenHandCursor);
    setAcceptHoverEvents(true);
    layoutLabel();
    updateToolTip();
  }

  QString vertexId() const { return id_; }
  QString fullLabel() const { return full_label_; }
  QString fullDetail() const { return full_detail_; }
  integration::GraphVertexKind vertexKind() const { return kind_; }
  const std::vector<GraphEdgeItem*>& connectedEdges() const { return edges_; }

  void setVisualMode(GraphVisualMode mode) {
    if (visual_mode_ == mode) {
      return;
    }
    visual_mode_ = mode;
    update();
  }

  GraphVisualMode visualMode() const { return visual_mode_; }

  void setActivityHz(double hz) {
    if (kind_ != integration::GraphVertexKind::kChannel) {
      return;
    }
    if (std::abs(activity_hz_ - hz) < 0.01 && activity_initialized_) {
      return;
    }
    activity_hz_ = hz;
    activity_initialized_ = true;
    const QFontMetricsF detail_metrics(detail_font_);
    detail_text_ = detail_metrics.elidedText(
        FormatChannelHz(activity_hz_), Qt::ElideRight,
        static_cast<int>(detail_rect_.width()));
    updateToolTip();
    update();
  }

  double activityHz() const { return activity_hz_; }

  QRectF boundingRect() const override {
    return content_rect_.adjusted(-6.0, -6.0, 6.0, 6.0);
  }

  QPainterPath shape() const override {
    if (kind_ == integration::GraphVertexKind::kChannel) {
      return CapsulePath(shape_rect_);
    }
    if (kind_ == integration::GraphVertexKind::kNode) {
      return CirclePath(shape_rect_);
    }
    return HexagonPath(shape_rect_);
  }

  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget) override {
    Q_UNUSED(widget);
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setRenderHint(QPainter::TextAntialiasing, true);
    if (visual_mode_ == GraphVisualMode::kDimmed) {
      painter->setOpacity(0.22);
    } else if (visual_mode_ == GraphVisualMode::kEmphasized) {
      painter->setOpacity(1.0);
    }

    const QColor fill = ColorForKind(kind_);
    const bool selected = option != nullptr && (option->state & QStyle::State_Selected);
    const bool hovered = option != nullptr && (option->state & QStyle::State_MouseOver);
    const bool emphasized = visual_mode_ == GraphVisualMode::kEmphasized;
    const double pen_width = selected || emphasized ? 2.4 : 1.5;
    QPen outline(fill.darker(selected ? 145 : 128), pen_width);

    if (kind_ == integration::GraphVertexKind::kNode) {
      QRadialGradient gradient(shape_rect_.center(), shape_rect_.width() * 0.55);
      gradient.setColorAt(0.0, fill.lighter(hovered ? 130 : 118));
      gradient.setColorAt(1.0, fill.darker(hovered ? 108 : 120));
      painter->setPen(outline);
      painter->setBrush(gradient);
      painter->drawPath(CirclePath(shape_rect_));
      painter->setPen(QColor(255, 255, 255, 230));
      painter->setFont(kind_font_);
      painter->drawText(kind_badge_rect_, Qt::AlignCenter, KindBadge(kind_));
      painter->setPen(QColor(210, 220, 235));
      painter->setFont(title_font_);
      painter->drawText(title_rect_, Qt::AlignHCenter | Qt::AlignTop, title_text_);
      if (!detail_text_.isEmpty()) {
        painter->setPen(QColor(170, 180, 195));
        painter->setFont(detail_font_);
        painter->drawText(detail_rect_, Qt::AlignHCenter | Qt::AlignTop, detail_text_);
      }
    } else if (kind_ == integration::GraphVertexKind::kChannel) {
      QColor fill_color = fill;
      if (activity_initialized_ && activity_hz_ <= 0.0) {
        fill_color = fill_color.darker(130);
      } else if (activity_hz_ > 0.0) {
        fill_color = fill_color.lighter(108);
      }
      QLinearGradient gradient(shape_rect_.topLeft(), shape_rect_.bottomLeft());
      gradient.setColorAt(0.0, fill_color.lighter(hovered ? 122 : 110));
      gradient.setColorAt(1.0, fill_color.darker(hovered ? 108 : 118));
      painter->setPen(outline);
      painter->setBrush(gradient);
      painter->drawPath(CapsulePath(shape_rect_));
      const double cy = shape_rect_.center().y();
      painter->setBrush(QColor(255, 255, 255, 220));
      painter->setPen(Qt::NoPen);
      painter->drawEllipse(QPointF(shape_rect_.left() + 9.0, cy), 4.0, 4.0);
      painter->drawEllipse(QPointF(shape_rect_.right() - 9.0, cy), 4.0, 4.0);
      painter->setPen(Qt::white);
      painter->setFont(title_font_);
      painter->drawText(title_rect_, Qt::AlignVCenter | Qt::AlignLeft, title_text_);
      if (!detail_text_.isEmpty()) {
        painter->setPen(activity_hz_ > 0.0 ? QColor(200, 255, 210, 230)
                                           : QColor(255, 255, 255, 205));
        painter->setFont(detail_font_);
        painter->drawText(detail_rect_, Qt::AlignVCenter | Qt::AlignRight, detail_text_);
      }
    } else {
      QLinearGradient gradient(shape_rect_.topLeft(), shape_rect_.bottomRight());
      gradient.setColorAt(0.0, fill.lighter(hovered ? 120 : 108));
      gradient.setColorAt(1.0, fill.darker(hovered ? 110 : 122));
      painter->setPen(outline);
      painter->setBrush(gradient);
      painter->drawPath(HexagonPath(shape_rect_));
      painter->setPen(QColor(255, 255, 255, 225));
      painter->setFont(kind_font_);
      painter->drawText(kind_badge_rect_, Qt::AlignCenter, KindBadge(kind_));
      painter->setFont(title_font_);
      painter->drawText(title_rect_, Qt::AlignCenter, title_text_);
      if (!detail_text_.isEmpty()) {
        painter->setPen(QColor(255, 255, 255, 205));
        painter->setFont(detail_font_);
        painter->drawText(detail_rect_, Qt::AlignCenter, detail_text_);
      }
    }

    if (selected || emphasized) {
      QPen highlight(QColor(255, 214, 90), emphasized ? 2.4 : 2.0);
      painter->setPen(highlight);
      painter->setBrush(Qt::NoBrush);
      painter->drawPath(shape().translated(0, 0));
    }
  }

  QPointF connectionPoint(const GraphVertexItem* other,
                        const GraphEdgeItem* via_edge) const;

  void setMoveNotifier(const std::function<void()>& notifier);

 protected:
  QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

 private:
  friend class GraphEdgeItem;

  void registerEdge(GraphEdgeItem* edge) { edges_.push_back(edge); }

  void layoutLabel() {
    const QFontMetricsF title_metrics(title_font_);
    const QFontMetricsF detail_metrics(detail_font_);

    if (kind_ == integration::GraphVertexKind::kChannel) {
      const double title_width =
          title_metrics.horizontalAdvance(full_label_) + kTextPadding * 2.0 + 28.0;
      const double width =
          std::clamp(title_width, kChannelWidthMin, kChannelWidthMax);
      const double height = kChannelHeight;
      // Keep Hz clear of the decorative end-caps (radius ~4 at ±9 from edges).
      constexpr double kPortClearance = 20.0;
      const double detail_width = std::min(width * 0.28, 88.0);
      const double title_width_budget =
          width - kPortClearance * 2.0 - detail_width - 8.0;
      shape_rect_ = QRectF(-width * 0.5, -height * 0.5, width, height);
      title_text_ = title_metrics.elidedText(full_label_, Qt::ElideMiddle,
                                             static_cast<int>(title_width_budget));
      detail_text_ = detail_metrics.elidedText(
          activity_initialized_ ? FormatChannelHz(activity_hz_)
                                : ShortTypeName(full_detail_),
          Qt::ElideRight, static_cast<int>(detail_width));
      title_rect_ = QRectF(shape_rect_.left() + kPortClearance, shape_rect_.top(),
                           title_width_budget, height);
      detail_rect_ =
          QRectF(shape_rect_.right() - kPortClearance - detail_width,
                 shape_rect_.top(), detail_width, height);
      content_rect_ = shape_rect_;
      return;
    }

    if (kind_ == integration::GraphVertexKind::kNode) {
      title_text_ = title_metrics.elidedText(full_label_, Qt::ElideMiddle, 180);
      detail_text_ = detail_metrics.elidedText(ShortTypeName(full_detail_), Qt::ElideMiddle,
                                               180);
      const double diameter = std::clamp(
          std::max(kNodeDiameterMin,
                   std::max(title_metrics.height() + 28.0,
                            detail_text_.isEmpty() ? kNodeDiameterMin
                                                   : detail_metrics.height() + 16.0)),
          kNodeDiameterMin, kNodeDiameterMax);
      shape_rect_ = QRectF(-diameter * 0.5, -diameter * 0.5, diameter, diameter);
      kind_badge_rect_ = QRectF(shape_rect_.left() + 8.0, shape_rect_.top() + 8.0,
                                shape_rect_.width() - 16.0, 12.0);
      const double label_top = shape_rect_.bottom() + 6.0;
      title_rect_ =
          QRectF(-90.0, label_top, 180.0, title_metrics.height() + 2.0);
      detail_rect_ = detail_text_.isEmpty()
                         ? QRectF()
                         : QRectF(-90.0, title_rect_.bottom() + 2.0, 180.0,
                                  detail_metrics.height() + 2.0);
      content_rect_ = shape_rect_.united(title_rect_);
      if (!detail_rect_.isEmpty()) {
        content_rect_ = content_rect_.united(detail_rect_);
      }
      return;
    }

    title_text_ = title_metrics.elidedText(ShortPathTail(full_label_), Qt::ElideMiddle,
                                           static_cast<int>(kServiceWidth - 20.0));
    detail_text_ = detail_metrics.elidedText(ShortTypeName(full_detail_), Qt::ElideMiddle,
                                             static_cast<int>(kServiceWidth - 20.0));
    shape_rect_ = QRectF(-kServiceWidth * 0.5, -kServiceHeight * 0.5, kServiceWidth,
                         kServiceHeight);
    kind_badge_rect_ = QRectF(-40.0, shape_rect_.top() + 18.0, 80.0, 12.0);
    title_rect_ = QRectF(shape_rect_.left() + 10.0, shape_rect_.center().y() - 8.0,
                         shape_rect_.width() - 20.0, title_metrics.height() + 2.0);
    detail_rect_ = detail_text_.isEmpty()
                       ? QRectF()
                       : QRectF(shape_rect_.left() + 10.0, title_rect_.bottom() + 2.0,
                                shape_rect_.width() - 20.0, detail_metrics.height() + 2.0);
    content_rect_ = shape_rect_;
  }

  int edgeBundleIndex(const GraphVertexItem* peer, const GraphEdgeItem* via_edge,
                      int* total) const;

  void updateToolTip() {
    QString tip = full_label_;
    if (!full_detail_.isEmpty()) {
      tip += QStringLiteral("\n") + full_detail_;
    }
    if (kind_ == integration::GraphVertexKind::kChannel && activity_initialized_) {
      tip += QStringLiteral("\n") + FormatChannelHz(activity_hz_);
    }
    tip += QStringLiteral("\n(") + KindBadge(kind_) + QLatin1Char(')');
    setToolTip(tip);
  }

  QString id_;
  integration::GraphVertexKind kind_;
  QString full_label_;
  QString full_detail_;
  QString title_text_;
  QString detail_text_;
  QRectF shape_rect_;
  QRectF content_rect_;
  QRectF kind_badge_rect_;
  QRectF title_rect_;
  QRectF detail_rect_;
  QFont kind_font_;
  QFont title_font_;
  QFont detail_font_;
  std::vector<GraphEdgeItem*> edges_;
  std::function<void()> move_notifier_;
  GraphVisualMode visual_mode_ = GraphVisualMode::kNormal;
  double activity_hz_ = 0.0;
  bool activity_initialized_ = false;
};

class GraphEdgeItem : public QGraphicsObject {
 public:
  GraphEdgeItem(GraphVertexItem* from, GraphVertexItem* to,
                integration::GraphEdgeKind kind, QGraphicsItem* parent = nullptr)
      : QGraphicsObject(parent), from_(from), to_(to), kind_(kind) {
    setZValue(-1.0);
    setAcceptHoverEvents(true);
    setFlag(QGraphicsItem::ItemIsSelectable, false);
    if (from_ != nullptr) {
      from_->registerEdge(this);
    }
    if (to_ != nullptr) {
      to_->registerEdge(this);
    }
    updatePath();
    setToolTip(EdgeKindLabel(kind_));
  }

  bool connects(const GraphVertexItem* a, const GraphVertexItem* b) const {
    return (from_ == a && to_ == b) || (from_ == b && to_ == a);
  }

  bool touches(const GraphVertexItem* vertex) const {
    return from_ == vertex || to_ == vertex;
  }

  GraphVertexItem* fromVertex() const { return from_; }
  GraphVertexItem* toVertex() const { return to_; }
  integration::GraphEdgeKind edgeKind() const { return kind_; }

  void setVisualMode(GraphVisualMode mode) {
    if (visual_mode_ == mode) {
      return;
    }
    visual_mode_ = mode;
    setZValue(mode == GraphVisualMode::kEmphasized ? 1.0 : -1.0);
    update();
  }

  void setShowLabelAlways(bool enabled) {
    if (show_label_always_ == enabled) {
      return;
    }
    show_label_always_ = enabled;
    update();
  }

  void setFlowPhase(double phase) {
    flow_phase_ = phase;
    if (visual_mode_ == GraphVisualMode::kEmphasized && IsAnimatedFlowEdge(kind_)) {
      update();
    }
  }

  QRectF boundingRect() const override {
    return path_.boundingRect().adjusted(-16, -16, 16, 16);
  }

  QPainterPath shape() const override {
    QPainterPath hit;
    if (path_.isEmpty()) {
      return hit;
    }
    QPainterPathStroker stroker;
    stroker.setWidth(8.0);
    return stroker.createStroke(path_);
  }

  void paint(QPainter* painter, const QStyleOptionGraphicsItem* option,
             QWidget* widget) override {
    Q_UNUSED(option);
    Q_UNUSED(widget);
    if (path_.isEmpty()) {
      return;
    }

    painter->setRenderHint(QPainter::Antialiasing, true);
    QColor color = ColorForEdge(kind_);
    double width = kind_ == integration::GraphEdgeKind::kRelay ? 1.4 : 1.8;
    if (visual_mode_ == GraphVisualMode::kDimmed) {
      color.setAlpha(28);
      width = 1.2;
    } else if (visual_mode_ == GraphVisualMode::kEmphasized) {
      color = color.lighter(125);
      color.setAlpha(255);
      width = IsAnimatedFlowEdge(kind_) ? 3.4 : 2.6;
    }

    QPen pen(color, width, PenStyleForEdge(kind_), Qt::RoundCap, Qt::RoundJoin);
    painter->setPen(pen);
    painter->setBrush(Qt::NoBrush);
    painter->drawPath(path_);

    if (visual_mode_ == GraphVisualMode::kEmphasized && IsAnimatedFlowEdge(kind_)) {
      // Moving dash overlay — direction follows the edge arrow.
      QPen flow_pen(QColor(255, 255, 255, 220), std::max(1.6, width * 0.55),
                    Qt::CustomDashLine, Qt::RoundCap, Qt::RoundJoin);
      flow_pen.setDashPattern({5.0, 12.0});
      flow_pen.setDashOffset(-flow_phase_);
      painter->setPen(flow_pen);
      painter->drawPath(path_);

      // Traveling packets along the cubic.
      const int packet_count = 3;
      for (int i = 0; i < packet_count; ++i) {
        const double t =
            std::fmod(flow_phase_ * 0.035 + static_cast<double>(i) / packet_count,
                      1.0);
        const QPointF packet =
            CubicBezierPoint(start_, c1_, c2_, end_, static_cast<float>(t));
        painter->setPen(Qt::NoPen);
        painter->setBrush(QColor(255, 255, 255, 240));
        painter->drawEllipse(packet, 3.4, 3.4);
        painter->setBrush(color.lighter(145));
        painter->drawEllipse(packet, 2.1, 2.1);
      }
    }

    const QPointF tip = end_;
    const QPointF direction = tip - CubicBezierPoint(start_, c1_, c2_, end_, 0.90);
    DrawArrowHead(painter, tip, direction, color);

    const QString label = EdgeKindLabel(kind_);
    const bool show_label =
        !label.isEmpty() && visual_mode_ != GraphVisualMode::kDimmed &&
        (show_label_always_ || visual_mode_ == GraphVisualMode::kEmphasized ||
         hovered_);
    if (!show_label) {
      return;
    }

    const QPointF label_pos = CubicBezierPoint(start_, c1_, c2_, end_, 0.50);
    QFont label_font;
    label_font.setPointSize(visual_mode_ == GraphVisualMode::kEmphasized || hovered_
                                ? 9
                                : 8);
    label_font.setBold(true);
    const QFontMetricsF metrics(label_font);
    const QRectF text_rect =
        metrics.boundingRect(label).adjusted(-4.0, -2.0, 4.0, 2.0);
    const QRectF bg_rect = text_rect.translated(label_pos - text_rect.center());

    painter->setPen(Qt::NoPen);
    painter->setBrush(QColor(18, 18, 20, 225));
    painter->drawRoundedRect(bg_rect, 3.0, 3.0);
    painter->setPen(color.lighter(135));
    painter->setFont(label_font);
    painter->drawText(bg_rect, Qt::AlignCenter, label);
  }

  void updatePath() {
    prepareGeometryChange();
    path_ = QPainterPath();
    if (from_ == nullptr || to_ == nullptr) {
      update();
      return;
    }
    start_ = from_->connectionPoint(to_, this);
    end_ = to_->connectionPoint(from_, this);
    const QPointF delta = end_ - start_;
    const double curve = std::clamp(std::abs(delta.x()) * 0.42, 48.0, 200.0);
    const double lane = laneOffset(from_, to_);
    c1_ = start_ + QPointF((delta.x() >= 0.0 ? curve : -curve), lane);
    c2_ = end_ - QPointF((delta.x() >= 0.0 ? curve : -curve), lane);
    path_.moveTo(start_);
    path_.cubicTo(c1_, c2_, end_);
    update();
  }

 protected:
  void hoverEnterEvent(QGraphicsSceneHoverEvent* event) override {
    Q_UNUSED(event);
    hovered_ = true;
    update();
  }

  void hoverLeaveEvent(QGraphicsSceneHoverEvent* event) override {
    Q_UNUSED(event);
    hovered_ = false;
    update();
  }

 private:
  double laneOffset(const GraphVertexItem* from, const GraphVertexItem* to) const {
    int total = 1;
    const int from_index = from->edgeBundleIndex(to, this, &total);
    const int to_index = to->edgeBundleIndex(from, this, &total);
    const int index = std::max(from_index, to_index);
    const int count = std::max(total, 1);
    return (index - (count - 1) * 0.5) * kConnectionSpread;
  }

  GraphVertexItem* from_ = nullptr;
  GraphVertexItem* to_ = nullptr;
  integration::GraphEdgeKind kind_ = integration::GraphEdgeKind::kPublish;
  QPainterPath path_;
  QPointF start_;
  QPointF end_;
  QPointF c1_;
  QPointF c2_;
  GraphVisualMode visual_mode_ = GraphVisualMode::kNormal;
  double flow_phase_ = 0.0;
  bool show_label_always_ = false;
  bool hovered_ = false;
};

int GraphVertexItem::edgeBundleIndex(const GraphVertexItem* peer,
                                     const GraphEdgeItem* via_edge,
                                     int* total) const {
  std::vector<GraphEdgeItem*> bundle;
  for (GraphEdgeItem* edge : edges_) {
    if (edge->connects(this, peer)) {
      bundle.push_back(edge);
    }
  }
  std::sort(bundle.begin(), bundle.end(),
            [](const GraphEdgeItem* lhs, const GraphEdgeItem* rhs) {
              return lhs < rhs;
            });
  *total = static_cast<int>(bundle.size());
  for (int i = 0; i < static_cast<int>(bundle.size()); ++i) {
    if (bundle[static_cast<std::size_t>(i)] == via_edge) {
      return i;
    }
  }
  return 0;
}

QPointF GraphVertexItem::connectionPoint(const GraphVertexItem* other,
                                         const GraphEdgeItem* via_edge) const {
  const QRectF rect = shape_rect_.translated(pos());
  const QPointF center = rect.center();
  if (other == nullptr) {
    return center;
  }

  int total = 1;
  const int index = edgeBundleIndex(other, via_edge, &total);
  const double spread_offset = (index - (total - 1) * 0.5) * kConnectionSpread;

  const QPointF other_center = other->shape_rect_.translated(other->pos()).center();
  QPointF delta = other_center - center;
  if (std::abs(delta.x()) < 0.001 && std::abs(delta.y()) < 0.001) {
    delta = QPointF(1.0, 0.0);
  }

  if (kind_ == integration::GraphVertexKind::kChannel) {
    const bool to_right = delta.x() >= 0.0;
    return QPointF(to_right ? rect.right() : rect.left(), center.y() + spread_offset);
  }

  if (kind_ == integration::GraphVertexKind::kNode) {
    const double radius = rect.width() * 0.5;
    if (std::abs(delta.x()) >= std::abs(delta.y()) * 0.65) {
      return QPointF(center.x() + (delta.x() >= 0.0 ? radius : -radius),
                     center.y() + spread_offset);
    }
    const double y = center.y() + (delta.y() >= 0.0 ? radius : -radius);
    return QPointF(center.x() + spread_offset, y);
  }

  const double rx = rect.width() * 0.5;
  const double ry = rect.height() * 0.5;
  if (std::abs(delta.x()) >= std::abs(delta.y())) {
    return QPointF(center.x() + (delta.x() >= 0.0 ? rx : -rx),
                   center.y() + spread_offset);
  }
  return QPointF(center.x() + spread_offset,
                 center.y() + (delta.y() >= 0.0 ? ry : -ry));
}

void GraphVertexItem::setMoveNotifier(const std::function<void()>& notifier) {
  move_notifier_ = notifier;
}

QVariant GraphVertexItem::itemChange(GraphicsItemChange change, const QVariant& value) {
  if (change == ItemPositionHasChanged) {
    for (GraphEdgeItem* edge : edges_) {
      edge->updatePath();
    }
    if (move_notifier_) {
      move_notifier_();
    }
  }
  return QGraphicsObject::itemChange(change, value);
}

}  // namespace

ChannelGraphView::ChannelGraphView(QWidget* parent) : QGraphicsView(parent) {
  scene_ = new QGraphicsScene(this);
  setScene(scene_);
  setRenderHint(QPainter::Antialiasing, true);
  setRenderHint(QPainter::TextAntialiasing, true);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setResizeAnchor(QGraphicsView::AnchorViewCenter);
  setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
  setBackgroundBrush(QColor(18, 18, 20));

  flow_timer_ = new QTimer(this);
  flow_timer_->setInterval(33);
  connect(flow_timer_, &QTimer::timeout, this, &ChannelGraphView::onFlowTick);
  activity_timer_ = new QTimer(this);
  activity_timer_->setInterval(500);
  connect(activity_timer_, &QTimer::timeout, this, &ChannelGraphView::onActivityTick);
  activity_timer_->start();
  connect(scene_, &QGraphicsScene::selectionChanged, this,
          &ChannelGraphView::onSelectionChanged);
}

void ChannelGraphView::setGraph(const integration::TopologyGraph& graph,
                                bool preserve_positions) {
  if (preserve_positions &&
      last_topology_hash_ == QString::fromStdString(graph.topology_hash)) {
    return;
  }

  focused_vertex_id_.clear();
  setFlowAnimationActive(false);

  QHash<QString, QPointF> positions =
      preserve_positions ? saved_positions_ : QHash<QString, QPointF>();
  if (!preserve_positions) {
    positions.clear();
  }

  scene_->clear();
  std::unordered_map<std::string, GraphVertexItem*> items;
  items.reserve(graph.vertices.size());

  int channel_index = 0;
  int service_index = 0;
  const int channel_count = static_cast<int>(std::count_if(
      graph.vertices.begin(), graph.vertices.end(), [](const integration::GraphVertex& vertex) {
        return vertex.kind == integration::GraphVertexKind::kChannel;
      }));
  const int service_count = static_cast<int>(std::count_if(
      graph.vertices.begin(), graph.vertices.end(), [](const integration::GraphVertex& vertex) {
        return vertex.kind == integration::GraphVertexKind::kService;
      }));

  for (const integration::GraphVertex& vertex : graph.vertices) {
    auto* item = new GraphVertexItem(
        QString::fromStdString(vertex.id), vertex.kind,
        QString::fromStdString(vertex.label),
        QString::fromStdString(vertex.detail));
    scene_->addItem(item);

    const QString key = QString::fromStdString(vertex.id);
    if (positions.contains(key)) {
      item->setPos(positions.value(key));
    } else {
      int local_channel_index = channel_index;
      int local_service_index = service_index;
      if (vertex.kind == integration::GraphVertexKind::kChannel) {
        ++channel_index;
      } else if (vertex.kind == integration::GraphVertexKind::kService) {
        ++service_index;
      }
      item->setPos(defaultPositionForVertex(vertex, local_channel_index,
                                            local_service_index, channel_count,
                                            service_count));
    }

    item->setMoveNotifier([this]() { onVertexMoved(); });
    items.emplace(vertex.id, item);
  }

  if (!preserve_positions || saved_positions_.isEmpty()) {
    applyAutoLayout(graph);
    for (const integration::GraphVertex& vertex : graph.vertices) {
      if (auto it = items.find(vertex.id); it != items.end()) {
        positions.insert(QString::fromStdString(vertex.id), it->second->pos());
      }
    }
  }

  std::vector<GraphEdgeItem*> edge_items;
  edge_items.reserve(graph.edges.size());
  for (const integration::GraphEdge& edge : graph.edges) {
    auto from_it = items.find(edge.from_id);
    auto to_it = items.find(edge.to_id);
    if (from_it == items.end() || to_it == items.end()) {
      continue;
    }
    auto* edge_item =
        new GraphEdgeItem(from_it->second, to_it->second, edge.kind);
    scene_->addItem(edge_item);
    edge_items.push_back(edge_item);
  }
  for (GraphEdgeItem* edge_item : edge_items) {
    edge_item->updatePath();
    edge_item->setShowLabelAlways(show_edge_labels_);
  }

  if (!graph.vertices.empty()) {
    updateColumnHeaders(service_count);
  }

  saved_positions_ = positions;
  last_topology_hash_ = QString::fromStdString(graph.topology_hash);
  onActivityTick();
  emit graphRendered(static_cast<int>(graph.vertices.size()),
                     static_cast<int>(graph.edges.size()));
}

QHash<QString, QPointF> ChannelGraphView::savedPositions() const {
  return saved_positions_;
}

void ChannelGraphView::resetSavedPositions() {
  saved_positions_.clear();
  last_topology_hash_.clear();
}

void ChannelGraphView::setArrangeModes(VertexArrangeMode channel_mode,
                                       VertexArrangeMode service_mode) {
  channel_arrange_mode_ = channel_mode;
  service_arrange_mode_ = service_mode;
}

void ChannelGraphView::setNeighborhoodMode(bool enabled) {
  if (neighborhood_mode_ == enabled) {
    return;
  }
  neighborhood_mode_ = enabled;
  if (!focused_vertex_id_.isEmpty()) {
    applyFocusHighlight(focused_vertex_id_);
  } else {
    clearFocusHighlight();
  }
}

void ChannelGraphView::setShowEdgeLabels(bool enabled) {
  if (show_edge_labels_ == enabled) {
    return;
  }
  show_edge_labels_ = enabled;
  applyEdgeLabelPolicy();
}

void ChannelGraphView::zoomToFit() {
  const QRectF bounds = scene_->itemsBoundingRect().adjusted(-120, -100, 120, 120);
  if (bounds.isEmpty()) {
    return;
  }
  fitInView(bounds, Qt::KeepAspectRatio);
  zoom_factor_ = transform().m11();
}

void ChannelGraphView::wheelEvent(QWheelEvent* event) {
  const double factor = event->angleDelta().y() > 0 ? 1.12 : 1.0 / 1.12;
  const double next_zoom = std::clamp(zoom_factor_ * factor, kMinZoom, kMaxZoom);
  const double applied = next_zoom / zoom_factor_;
  zoom_factor_ = next_zoom;
  scale(applied, applied);
  event->accept();
}

void ChannelGraphView::showEvent(QShowEvent* event) {
  QGraphicsView::showEvent(event);
  zoomToFit();
}

void ChannelGraphView::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    QGraphicsItem* item = itemAt(event->pos());
    auto* vertex = dynamic_cast<GraphVertexItem*>(item);
    if (vertex == nullptr && item != nullptr) {
      vertex = dynamic_cast<GraphVertexItem*>(item->topLevelItem());
    }
    if (vertex != nullptr) {
      scene_->clearSelection();
      vertex->setSelected(true);
      applyFocusHighlight(vertex->vertexId());
      event->accept();
      // Still allow the base class to begin a potential pan if the user drags.
      QGraphicsView::mousePressEvent(event);
      return;
    }
    if (item == nullptr) {
      scene_->clearSelection();
      clearFocusHighlight();
    }
  }
  QGraphicsView::mousePressEvent(event);
}

void ChannelGraphView::mouseDoubleClickEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    QGraphicsItem* item = itemAt(event->pos());
    auto* vertex = dynamic_cast<GraphVertexItem*>(item);
    if (vertex == nullptr && item != nullptr) {
      vertex = dynamic_cast<GraphVertexItem*>(item->topLevelItem());
    }
    if (vertex != nullptr) {
      scene_->clearSelection();
      vertex->setSelected(true);
      applyFocusHighlight(vertex->vertexId());
      emit vertexDoubleClicked(vertex->vertexId(), vertex->vertexKind(),
                               vertex->fullLabel(), vertex->fullDetail());
      event->accept();
      return;
    }
  }
  QGraphicsView::mouseDoubleClickEvent(event);
}

void ChannelGraphView::onVertexMoved() {
  const QList<QGraphicsItem*> items = scene_->items();
  for (QGraphicsItem* item : items) {
    auto* vertex = dynamic_cast<GraphVertexItem*>(item);
    if (vertex == nullptr) {
      continue;
    }
    saved_positions_.insert(vertex->vertexId(), vertex->pos());
  }
}

void ChannelGraphView::onSelectionChanged() {
  const QList<QGraphicsItem*> selected = scene_->selectedItems();
  for (QGraphicsItem* item : selected) {
    auto* vertex = dynamic_cast<GraphVertexItem*>(item);
    if (vertex != nullptr) {
      applyFocusHighlight(vertex->vertexId());
      return;
    }
  }
  if (selected.isEmpty()) {
    clearFocusHighlight();
  }
}

void ChannelGraphView::onFlowTick() {
  flow_phase_ += 1.6;
  if (flow_phase_ > 100000.0) {
    flow_phase_ = 0.0;
  }
  for (QGraphicsItem* item : scene_->items()) {
    if (auto* edge = dynamic_cast<GraphEdgeItem*>(item)) {
      if (edge->isVisible()) {
        edge->setFlowPhase(flow_phase_);
      }
    }
  }
}

void ChannelGraphView::onActivityTick() {
  if (scene_ == nullptr) {
    return;
  }
  for (QGraphicsItem* item : scene_->items()) {
    auto* vertex = dynamic_cast<GraphVertexItem*>(item);
    if (vertex == nullptr ||
        vertex->vertexKind() != integration::GraphVertexKind::kChannel) {
      continue;
    }
    const integration::ChannelStats stats =
        integration::ChannelStatsRegistry::instance().stats(
            vertex->fullLabel().toStdString());
    vertex->setActivityHz(stats.frequency_hz);
  }
}

void ChannelGraphView::setFlowAnimationActive(bool active) {
  if (flow_timer_ == nullptr) {
    return;
  }
  if (active) {
    if (!flow_timer_->isActive()) {
      flow_phase_ = 0.0;
      flow_timer_->start();
    }
  } else if (flow_timer_->isActive()) {
    flow_timer_->stop();
  }
}

void ChannelGraphView::applyEdgeLabelPolicy() {
  if (scene_ == nullptr) {
    return;
  }
  for (QGraphicsItem* item : scene_->items()) {
    if (auto* edge = dynamic_cast<GraphEdgeItem*>(item)) {
      edge->setShowLabelAlways(show_edge_labels_);
    }
  }
}

void ChannelGraphView::clearFocusHighlight() {
  focused_vertex_id_.clear();
  setFlowAnimationActive(false);
  for (QGraphicsItem* item : scene_->items()) {
    if (auto* vertex = dynamic_cast<GraphVertexItem*>(item)) {
      vertex->setVisible(true);
      vertex->setVisualMode(GraphVisualMode::kNormal);
    } else if (auto* edge = dynamic_cast<GraphEdgeItem*>(item)) {
      edge->setVisible(true);
      edge->setVisualMode(GraphVisualMode::kNormal);
      edge->setFlowPhase(0.0);
    }
  }
}

void ChannelGraphView::applyFocusHighlight(const QString& vertex_id) {
  focused_vertex_id_ = vertex_id;
  GraphVertexItem* focus_vertex = nullptr;
  for (QGraphicsItem* item : scene_->items()) {
    auto* vertex = dynamic_cast<GraphVertexItem*>(item);
    if (vertex != nullptr && vertex->vertexId() == vertex_id) {
      focus_vertex = vertex;
      break;
    }
  }
  if (focus_vertex == nullptr) {
    clearFocusHighlight();
    return;
  }

  QSet<GraphVertexItem*> related_vertices;
  QSet<GraphEdgeItem*> related_edges;
  related_vertices.insert(focus_vertex);
  for (GraphEdgeItem* edge : focus_vertex->connectedEdges()) {
    if (edge == nullptr ||
        !IsFocusRelatedEdge(focus_vertex->vertexKind(), edge->edgeKind())) {
      continue;
    }
    related_edges.insert(edge);
    if (edge->fromVertex() != nullptr) {
      related_vertices.insert(edge->fromVertex());
    }
    if (edge->toVertex() != nullptr) {
      related_vertices.insert(edge->toVertex());
    }
  }

  for (QGraphicsItem* item : scene_->items()) {
    if (auto* vertex = dynamic_cast<GraphVertexItem*>(item)) {
      const bool related = related_vertices.contains(vertex);
      if (neighborhood_mode_) {
        vertex->setVisible(related);
        vertex->setVisualMode(related ? GraphVisualMode::kEmphasized
                                      : GraphVisualMode::kNormal);
      } else {
        vertex->setVisible(true);
        vertex->setVisualMode(related ? GraphVisualMode::kEmphasized
                                      : GraphVisualMode::kDimmed);
      }
    } else if (auto* edge = dynamic_cast<GraphEdgeItem*>(item)) {
      const bool related = related_edges.contains(edge);
      if (neighborhood_mode_) {
        edge->setVisible(related);
        edge->setVisualMode(related ? GraphVisualMode::kEmphasized
                                    : GraphVisualMode::kNormal);
      } else {
        edge->setVisible(true);
        edge->setVisualMode(related ? GraphVisualMode::kEmphasized
                                    : GraphVisualMode::kDimmed);
      }
    }
  }
  setFlowAnimationActive(!related_edges.isEmpty());
}

void ChannelGraphView::updateColumnHeaders(int service_count) {
  QFont header_font;
  header_font.setPointSize(10);
  header_font.setBold(true);
  const QColor header_color(150, 150, 158);
  const double channel_header_y = -42.0;
  auto add_header = [&](const QString& text, double x, double y) {
    auto* label = scene_->addText(text, header_font);
    label->setDefaultTextColor(header_color);
    const QRectF bounds = label->boundingRect();
    label->setPos(x - bounds.width() * 0.5, y);
    label->setZValue(-2);
    label->setFlag(QGraphicsItem::ItemIsSelectable, false);
    label->setFlag(QGraphicsItem::ItemIsMovable, false);
  };
  add_header(tr("Writers"), layout_.writer_column_x, channel_header_y);
  add_header(tr("Channels"), layout_.channel_column_x, channel_header_y);
  add_header(tr("Readers"), layout_.reader_column_x, channel_header_y);
  if (service_count > 0) {
    add_header(tr("Services"),
               layout_.service_origin_x +
                   (std::max(1, layout_.service_columns) - 1) *
                       layout_.service_col_gap * 0.5,
               layout_.service_origin_y - 48.0);
  }
}

void ChannelGraphView::applyAutoLayout(const integration::TopologyGraph& graph) {
  std::unordered_map<std::string, GraphVertexItem*> items;
  const QList<QGraphicsItem*> scene_items = scene_->items();
  for (QGraphicsItem* item : scene_items) {
    if (auto* vertex = dynamic_cast<GraphVertexItem*>(item)) {
      items.emplace(vertex->vertexId().toStdString(), vertex);
    }
  }

  const int channel_count = static_cast<int>(std::count_if(
      graph.vertices.begin(), graph.vertices.end(), [](const integration::GraphVertex& vertex) {
        return vertex.kind == integration::GraphVertexKind::kChannel;
      }));
  const int node_count = static_cast<int>(std::count_if(
      graph.vertices.begin(), graph.vertices.end(), [](const integration::GraphVertex& vertex) {
        return vertex.kind == integration::GraphVertexKind::kNode;
      }));
  const int service_count = static_cast<int>(std::count_if(
      graph.vertices.begin(), graph.vertices.end(), [](const integration::GraphVertex& vertex) {
        return vertex.kind == integration::GraphVertexKind::kService;
      }));

  double max_channel_half_width = kChannelWidthMin * 0.5;
  for (const integration::GraphVertex& vertex : graph.vertices) {
    if (vertex.kind != integration::GraphVertexKind::kChannel) {
      continue;
    }
    if (auto it = items.find(vertex.id); it != items.end()) {
      max_channel_half_width =
          std::max(max_channel_half_width, it->second->boundingRect().width() * 0.5);
    }
  }

  const double viewport_width =
      viewport() != nullptr
          ? static_cast<double>(viewport()->width()) /
                std::max(0.2, transform().m11())
          : 1400.0;
  layout_ = ComputeAdaptiveLayoutOptions(
      channel_count, node_count, service_count,
      static_cast<int>(graph.edges.size()), max_channel_half_width, viewport_width,
      channel_arrange_mode_, service_arrange_mode_);

  std::unordered_map<std::string, double> channel_y;
  std::unordered_map<std::string, std::vector<double>> node_channel_ys;
  std::unordered_map<std::string, int> node_pub_count;
  std::unordered_map<std::string, int> node_sub_count;

  int channel_index = 0;
  for (const integration::GraphVertex& vertex : graph.vertices) {
    if (vertex.kind != integration::GraphVertexKind::kChannel) {
      continue;
    }
    const QPointF pos = ChannelGridPosition(layout_, channel_index);
    channel_y[vertex.id] = pos.y();
    if (auto it = items.find(vertex.id); it != items.end()) {
      it->second->setPos(pos);
    }
    ++channel_index;
  }

  for (const integration::GraphEdge& edge : graph.edges) {
    if (edge.kind == integration::GraphEdgeKind::kPublish) {
      auto channel_it = channel_y.find(edge.to_id);
      if (channel_it == channel_y.end()) {
        continue;
      }
      node_pub_count[edge.from_id] += 1;
      node_channel_ys[edge.from_id].push_back(channel_it->second);
    } else if (edge.kind == integration::GraphEdgeKind::kSubscribe) {
      auto channel_it = channel_y.find(edge.from_id);
      if (channel_it == channel_y.end()) {
        continue;
      }
      node_sub_count[edge.to_id] += 1;
      node_channel_ys[edge.to_id].push_back(channel_it->second);
    }
  }

  std::vector<NodeLayoutSlot> writer_slots;
  std::vector<NodeLayoutSlot> reader_slots;
  std::vector<NodeLayoutSlot> both_writer_slots;
  std::vector<NodeLayoutSlot> both_reader_slots;
  writer_slots.reserve(static_cast<std::size_t>(node_count));
  reader_slots.reserve(static_cast<std::size_t>(node_count));

  const double channel_band_mid_y =
      std::max(0, layout_.channel_rows - 1) * layout_.channel_row_gap * 0.5;

  for (const integration::GraphVertex& vertex : graph.vertices) {
    if (vertex.kind != integration::GraphVertexKind::kNode) {
      continue;
    }
    NodeLayoutSlot slot;
    slot.id = vertex.id;
    slot.pub_count = node_pub_count[vertex.id];
    slot.sub_count = node_sub_count[vertex.id];
    auto ys_it = node_channel_ys.find(vertex.id);
    if (ys_it != node_channel_ys.end() && !ys_it->second.empty()) {
      slot.preferred_y = MedianOf(ys_it->second);
    } else {
      slot.preferred_y = channel_band_mid_y;
    }

    if (slot.pub_count > 0 && slot.sub_count > 0) {
      if (slot.pub_count >= slot.sub_count) {
        both_writer_slots.push_back(slot);
      } else {
        both_reader_slots.push_back(slot);
      }
    } else if (slot.pub_count > 0) {
      writer_slots.push_back(slot);
    } else if (slot.sub_count > 0) {
      reader_slots.push_back(slot);
    }
  }

  auto place_column = [&](std::vector<NodeLayoutSlot>& node_slots, double x) {
    if (node_slots.empty()) {
      return;
    }
    std::vector<double> preferred;
    preferred.reserve(node_slots.size());
    for (const NodeLayoutSlot& slot : node_slots) {
      preferred.push_back(slot.preferred_y);
    }
    const std::vector<double> packed =
        PackPreferredPositions(std::move(preferred), layout_.node_row_gap);
    for (std::size_t i = 0; i < node_slots.size(); ++i) {
      if (auto it = items.find(node_slots[i].id); it != items.end()) {
        it->second->setPos(x, packed[i]);
      }
    }
  };

  place_column(writer_slots, layout_.writer_column_x);
  place_column(reader_slots, layout_.reader_column_x);
  place_column(both_writer_slots, layout_.both_writer_column_x);
  place_column(both_reader_slots, layout_.both_reader_column_x);

  int service_index = 0;
  for (const integration::GraphVertex& vertex : graph.vertices) {
    if (vertex.kind != integration::GraphVertexKind::kService) {
      continue;
    }
    if (auto it = items.find(vertex.id); it != items.end()) {
      it->second->setPos(ServiceGridPosition(layout_, service_index));
    }
    ++service_index;
  }

  int orphan_index = 0;
  const double orphan_x =
      layout_.service_origin_x - kServiceWidth * 0.5 - kNodeDiameterMax - 80.0;
  for (const integration::GraphVertex& vertex : graph.vertices) {
    if (vertex.kind != integration::GraphVertexKind::kNode) {
      continue;
    }
    if (node_channel_ys.find(vertex.id) != node_channel_ys.end()) {
      continue;
    }
    if (node_pub_count[vertex.id] > 0 || node_sub_count[vertex.id] > 0) {
      continue;
    }
    if (auto it = items.find(vertex.id); it != items.end()) {
      it->second->setPos(
          orphan_x,
          layout_.service_origin_y + orphan_index * layout_.node_row_gap);
      ++orphan_index;
    }
  }

  for (QGraphicsItem* item : scene_->items()) {
    if (auto* edge = dynamic_cast<GraphEdgeItem*>(item)) {
      edge->updatePath();
    }
  }
}

QPointF ChannelGraphView::defaultPositionForVertex(
    const integration::GraphVertex& vertex, int channel_index, int service_index,
    int channel_count, int service_count) const {
  Q_UNUSED(channel_count);
  Q_UNUSED(service_count);
  switch (vertex.kind) {
    case integration::GraphVertexKind::kChannel:
      return ChannelGridPosition(layout_, channel_index);
    case integration::GraphVertexKind::kService:
      return ServiceGridPosition(layout_, service_index);
    case integration::GraphVertexKind::kNode:
    default:
      return QPointF(layout_.writer_column_x,
                     channel_index * layout_.channel_row_gap);
  }
}

}  // namespace channel_graph
}  // namespace autoviz
