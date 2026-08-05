/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/channel_graph/channel_graph_view.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <unordered_map>
#include <vector>

#include <QBrush>
#include <QFont>
#include <QFontMetricsF>
#include <QGraphicsObject>
#include <QGraphicsScene>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QPainterPathStroker>
#include <QPen>
#include <QPolygonF>
#include <QShowEvent>
#include <QStyle>
#include <QStyleOptionGraphicsItem>
#include <QWheelEvent>

namespace autoviz {
namespace channel_graph {
namespace {

constexpr double kMinZoom = 0.15;
constexpr double kMaxZoom = 4.0;
constexpr double kNodeDiameterMin = 76.0;
constexpr double kNodeDiameterMax = 108.0;
constexpr double kChannelWidthMin = 280.0;
constexpr double kChannelHeight = 46.0;
constexpr double kServiceWidth = 168.0;
constexpr double kServiceHeight = 148.0;
constexpr double kTextPadding = 10.0;
constexpr double kArrowLength = 11.0;
constexpr double kArrowWidth = 5.5;
constexpr double kConnectionSpread = 13.0;

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
  integration::GraphVertexKind vertexKind() const { return kind_; }

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

    const QColor fill = ColorForKind(kind_);
    const bool selected = option != nullptr && (option->state & QStyle::State_Selected);
    const bool hovered = option != nullptr && (option->state & QStyle::State_MouseOver);
    const double pen_width = selected ? 2.2 : 1.5;
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
      QLinearGradient gradient(shape_rect_.topLeft(), shape_rect_.bottomLeft());
      gradient.setColorAt(0.0, fill.lighter(hovered ? 122 : 110));
      gradient.setColorAt(1.0, fill.darker(hovered ? 108 : 118));
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
        painter->setPen(QColor(255, 255, 255, 205));
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

    if (selected) {
      QPen highlight(QColor(255, 214, 90), 2.0);
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
      const double width = std::max(kChannelWidthMin, title_width);
      const double height = kChannelHeight;
      shape_rect_ = QRectF(-width * 0.5, -height * 0.5, width, height);
      title_text_ = title_metrics.elidedText(full_label_, Qt::ElideMiddle,
                                             static_cast<int>(width - 56.0));
      detail_text_ = detail_metrics.elidedText(ShortTypeName(full_detail_), Qt::ElideRight,
                                               static_cast<int>(width * 0.35));
      title_rect_ = QRectF(shape_rect_.left() + 20.0, shape_rect_.top(),
                           shape_rect_.width() - 40.0 - width * 0.35, height);
      detail_rect_ = QRectF(shape_rect_.right() - width * 0.35 - 8.0, shape_rect_.top(),
                            width * 0.35, height);
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
};

class GraphEdgeItem : public QGraphicsObject {
 public:
  GraphEdgeItem(GraphVertexItem* from, GraphVertexItem* to,
                integration::GraphEdgeKind kind, QGraphicsItem* parent = nullptr)
      : QGraphicsObject(parent), from_(from), to_(to), kind_(kind) {
    setZValue(-1.0);
    setAcceptHoverEvents(true);
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

  QRectF boundingRect() const override { return path_.boundingRect().adjusted(-12, -12, 12, 12); }

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
    const QColor color = ColorForEdge(kind_);
    QPen pen(color, kind_ == integration::GraphEdgeKind::kRelay ? 1.4 : 1.8,
             PenStyleForEdge(kind_), Qt::RoundCap, Qt::RoundJoin);
    painter->setPen(pen);
    painter->setBrush(Qt::NoBrush);
    painter->drawPath(path_);

    const QPointF tip = end_;
    const QPointF direction = tip - CubicBezierPoint(start_, c1_, c2_, end_, 0.90);
    DrawArrowHead(painter, tip, direction, color);

    const QPointF label_pos = CubicBezierPoint(start_, c1_, c2_, end_, 0.50);
    const QString label = EdgeKindLabel(kind_);
    if (label.isEmpty()) {
      return;
    }

    QFont label_font;
    label_font.setPointSize(8);
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
  setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
  setBackgroundBrush(QColor(18, 18, 20));
}

void ChannelGraphView::setGraph(const integration::TopologyGraph& graph,
                                bool preserve_positions) {
  if (preserve_positions &&
      last_topology_hash_ == QString::fromStdString(graph.topology_hash)) {
    return;
  }

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
  }

  if (!graph.vertices.empty()) {
    QFont header_font;
    header_font.setPointSize(9);
    header_font.setBold(true);
    const QColor header_color(150, 150, 158);
    const double header_y =
        scene_->itemsBoundingRect().top() - 28.0;
    auto add_header = [&](const QString& text, double x) {
      auto* label = scene_->addText(text, header_font);
      label->setDefaultTextColor(header_color);
      const QRectF bounds = label->boundingRect();
      label->setPos(x - bounds.width() * 0.5, header_y);
      label->setZValue(-2);
      label->setFlag(QGraphicsItem::ItemIsSelectable, false);
      label->setFlag(QGraphicsItem::ItemIsMovable, false);
    };
    add_header(tr("Writers"), layout_.writer_column_x);
    add_header(tr("Channels"), layout_.channel_column_x);
    add_header(tr("Readers"), layout_.reader_column_x);
    if (service_count > 0) {
      add_header(tr("Services"), layout_.service_column_x);
    }
  }

  saved_positions_ = positions;
  last_topology_hash_ = QString::fromStdString(graph.topology_hash);
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

void ChannelGraphView::zoomToFit() {
  const QRectF bounds = scene_->itemsBoundingRect().adjusted(-80, -80, 80, 80);
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

void ChannelGraphView::applyAutoLayout(const integration::TopologyGraph& graph) {
  std::unordered_map<std::string, GraphVertexItem*> items;
  const QList<QGraphicsItem*> scene_items = scene_->items();
  for (QGraphicsItem* item : scene_items) {
    if (auto* vertex = dynamic_cast<GraphVertexItem*>(item)) {
      items.emplace(vertex->vertexId().toStdString(), vertex);
    }
  }

  std::unordered_map<std::string, double> node_y;
  std::unordered_map<std::string, int> node_pub_count;
  std::unordered_map<std::string, int> node_sub_count;

  int channel_index = 0;
  for (const integration::GraphVertex& vertex : graph.vertices) {
    if (vertex.kind != integration::GraphVertexKind::kChannel) {
      continue;
    }
    const double y = channel_index * layout_.channel_row_gap;
    if (auto it = items.find(vertex.id); it != items.end()) {
      it->second->setPos(layout_.channel_column_x, y);
    }
    ++channel_index;

    for (const integration::GraphEdge& edge : graph.edges) {
      if (edge.to_id != vertex.id && edge.from_id != vertex.id) {
        continue;
      }
      if (edge.kind == integration::GraphEdgeKind::kPublish) {
        node_pub_count[edge.from_id] += 1;
        node_y[edge.from_id] += y;
      } else if (edge.kind == integration::GraphEdgeKind::kSubscribe) {
        node_sub_count[edge.to_id] += 1;
        node_y[edge.to_id] += y;
      }
    }
  }

  for (auto& entry : node_y) {
    const int pub = node_pub_count[entry.first];
    const int sub = node_sub_count[entry.first];
    const int count = std::max(pub + sub, 1);
    entry.second /= count;
    auto it = items.find(entry.first);
    if (it == items.end()) {
      continue;
    }
    const double x = (pub > 0 && sub > 0) ? (layout_.writer_column_x + layout_.reader_column_x) * 0.5
                      : pub > 0 ? layout_.writer_column_x
                                : layout_.reader_column_x;
    it->second->setPos(x, entry.second);
  }

  int service_index = 0;
  const double service_base_y =
      std::max(0, channel_index) * layout_.channel_row_gap + layout_.service_row_gap;
  for (const integration::GraphVertex& vertex : graph.vertices) {
    if (vertex.kind != integration::GraphVertexKind::kService) {
      continue;
    }
    if (auto it = items.find(vertex.id); it != items.end()) {
      it->second->setPos(layout_.service_column_x,
                         service_base_y + service_index * layout_.service_row_gap);
    }
    ++service_index;
  }

  for (const integration::GraphVertex& vertex : graph.vertices) {
    if (vertex.kind != integration::GraphVertexKind::kNode) {
      continue;
    }
    if (node_y.find(vertex.id) != node_y.end()) {
      continue;
    }
    if (auto it = items.find(vertex.id); it != items.end()) {
      it->second->setPos(layout_.writer_column_x, service_base_y + service_index * 48.0);
      ++service_index;
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
      return QPointF(layout_.channel_column_x, channel_index * layout_.channel_row_gap);
    case integration::GraphVertexKind::kService:
      return QPointF(layout_.service_column_x,
                     channel_count * layout_.channel_row_gap + layout_.service_row_gap +
                         service_index * layout_.service_row_gap);
    case integration::GraphVertexKind::kNode:
    default:
      return QPointF(layout_.writer_column_x, channel_index * layout_.channel_row_gap);
  }
}

}  // namespace channel_graph
}  // namespace autoviz
