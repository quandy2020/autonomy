/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_graph_view.hpp"
#include "autoviz/ui/behavior_tree/bt_groot_style.hpp"
#include "autoviz/ui/behavior_tree/bt_icon_loader.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include <QAction>
#include <QContextMenuEvent>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QFile>
#include <QFileDialog>
#include <QFont>
#include <QFontMetrics>
#include <QGraphicsObject>
#include <QGraphicsScene>
#include <QKeyEvent>
#include <QKeySequence>
#include <QMenu>
#include <QMessageBox>
#include <QMimeData>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QScrollBar>
#include <QSet>
#include <QSize>
#include <QStyleOptionGraphicsItem>
#include <QSvgGenerator>
#include <QTimer>
#include <QWheelEvent>

namespace autoviz {
namespace behavior_tree {
namespace {

constexpr double kMinNodeWidth = 140.0;
constexpr double kCaptionHeight = 24.0;
constexpr double kInstanceHeight = 22.0;
constexpr double kPortRowHeight = 20.0;
constexpr double kSubTreeButtonHeight = 22.0;
constexpr double kNodeRadius = 6.0;
constexpr double kNodeIconSize = 20.0;
constexpr double kNodePadX = 8.0;
constexpr double kNodePadY = 6.0;
constexpr double kPortGap = 2.0;
constexpr double kMinZoom = 0.15;
constexpr double kMaxZoom = 4.0;
constexpr int kMaxUndoSnapshots = 50;

constexpr char kCanvasBg[] = "#2b2b35";
constexpr char kEdgeIdleTeal[] = "#00b8bc";
constexpr double kEdgeWidth = 3.0;
constexpr double kPortRadius = 5.0;

/** Match QtNodes / Groot connection geometry (flow-axis handles only). */
void BuildTreeEdgePath(QPainterPath& path, const QPointF& start, const QPointF& end,
                       bool horizontal) {
  path.moveTo(start);

  constexpr double kDefaultOffset = 200.0;
  const double dx = end.x() - start.x();
  const double dy = end.y() - start.y();

  if (horizontal) {
    if (std::abs(dy) < 1.0) {
      path.lineTo(end);
      return;
    }
    // QtNodes ConnectionGeometry::pointsC1C2 (left→right ports).
    double horizontal_offset = std::min(kDefaultOffset, std::abs(dx));
    double vertical_offset = 0.0;
    double ratio_x = 0.5;
    if (dx <= 0.0) {
      const double y_distance = dy + 20.0;
      vertical_offset = std::min(kDefaultOffset, std::abs(y_distance)) *
                        (y_distance < 0.0 ? -1.0 : 1.0);
      ratio_x = 1.0;
    }
    horizontal_offset *= ratio_x;
    path.cubicTo(QPointF(start.x() + horizontal_offset, start.y() + vertical_offset),
                 QPointF(end.x() - horizontal_offset, end.y() - vertical_offset), end);
    return;
  }

  if (std::abs(dx) < 1.0) {
    path.lineTo(end);
    return;
  }
  // Vertical ports: rotate the same QtNodes formula (top→bottom flow).
  double vertical_offset = std::min(kDefaultOffset, std::abs(dy));
  double horizontal_offset = 0.0;
  double ratio_y = 0.5;
  if (dy <= 0.0) {
    const double x_distance = dx + 20.0;
    horizontal_offset = std::min(kDefaultOffset, std::abs(x_distance)) *
                        (x_distance < 0.0 ? -1.0 : 1.0);
    ratio_y = 1.0;
  }
  vertical_offset *= ratio_y;
  path.cubicTo(QPointF(start.x() + horizontal_offset, start.y() + vertical_offset),
               QPointF(end.x() - horizontal_offset, end.y() - vertical_offset), end);
}

struct PortDisplayRow {
  QString label;
  QString value;
};

bool ShowInstanceName(const BtAbsNode& node) {
  // Match Groot: SubTree hides alias when it equals registration id.
  if (node.kind == BtNodeKind::kSubTree) {
    return !node.instance_name.isEmpty() && node.instance_name != node.registration_id;
  }
  return !node.instance_name.isEmpty();
}

QString PortDirectionLabel(BtPortDirection direction, const QString& name) {
  switch (direction) {
    case BtPortDirection::kInput:
      return QStringLiteral("[IN] %1").arg(name);
    case BtPortDirection::kOutput:
      return QStringLiteral("[OUT] %1").arg(name);
    case BtPortDirection::kInOut:
      return QStringLiteral("[INOUT] %1").arg(name);
  }
  return name;
}

QVector<PortDisplayRow> BuildPortRows(const BtAbsNode& node, const BtNodeModel* model) {
  QVector<PortDisplayRow> rows;
  QSet<QString> seen;

  auto append_port = [&](const QString& name, BtPortDirection direction, const QString& value) {
    if (name.isEmpty() || name == QLatin1String("name")) {
      return;
    }
    // SubTree identity is the registration id, not a remappable ID port row.
    if (node.kind == BtNodeKind::kSubTree && name == QLatin1String("ID")) {
      return;
    }
    if (seen.contains(name)) {
      return;
    }
    seen.insert(name);
    rows.push_back(PortDisplayRow{PortDirectionLabel(direction, name), value});
  };

  if (model != nullptr) {
    // Groot order: INPUT → OUTPUT → INOUT
    const BtPortDirection order[] = {BtPortDirection::kInput, BtPortDirection::kOutput,
                                     BtPortDirection::kInOut};
    for (BtPortDirection direction : order) {
      for (const BtPortModel& port : model->ports) {
        if (port.direction != direction) {
          continue;
        }
        const QString value = node.port_remap.value(port.name, port.default_value);
        append_port(port.name, port.direction, value);
      }
    }
  }

  for (auto it = node.port_remap.constBegin(); it != node.port_remap.constEnd(); ++it) {
    append_port(it.key(), BtPortDirection::kInput, it.value());
  }
  return rows;
}

bool IsLeafKind(BtNodeKind kind) {
  return kind == BtNodeKind::kAction || kind == BtNodeKind::kCondition;
}

QString UniqueInstanceName(const BtAbsTree& tree, const QString& base) {
  if (!base.isEmpty()) {
    bool used = false;
    for (auto it = tree.nodes.constBegin(); it != tree.nodes.constEnd(); ++it) {
      if (it.value().instance_name == base) {
        used = true;
        break;
      }
    }
    if (!used) {
      return base;
    }
  }

  int suffix = 1;
  while (true) {
    const QString candidate = base + QString::number(suffix);
    bool used = false;
    for (auto it = tree.nodes.constBegin(); it != tree.nodes.constEnd(); ++it) {
      if (it.value().instance_name == candidate) {
        used = true;
        break;
      }
    }
    if (!used) {
      return candidate;
    }
    ++suffix;
  }
}

class BtEdgeItem;

class BtNodeItem : public QGraphicsObject {
 public:
  enum Anchor {
    kTopCenter,
    kBottomCenter,
    kLeftCenter,
    kRightCenter,
  };

  explicit BtNodeItem(int uid, const BtAbsNode& node, const BtNodeModel* model, bool read_only,
                      bool editor_mode, bool horizontal_layout,
                      QGraphicsItem* parent = nullptr)
      : QGraphicsObject(parent),
        uid_(uid),
        node_(node),
        read_only_(read_only || node.expanded_inline),
        editor_mode_(editor_mode),
        horizontal_layout_(horizontal_layout) {
    if (model != nullptr) {
      model_ = *model;
      has_model_ = true;
    }
    recomputeGeometry();
    setFlags(QGraphicsItem::ItemIsSelectable | QGraphicsItem::ItemSendsGeometryChanges |
             (read_only_ ? QGraphicsItem::GraphicsItemFlag() : QGraphicsItem::ItemIsMovable));
    setAcceptHoverEvents(true);
  }

  int uid() const { return uid_; }
  double visualWidth() const { return width_; }
  double visualHeight() const { return height_; }

  void setNodeData(const BtAbsNode& node, const BtNodeModel* model = nullptr) {
    node_ = node;
    if (model != nullptr) {
      model_ = *model;
      has_model_ = true;
    }
    prepareGeometryChange();
    recomputeGeometry();
    update();
  }

  void setStatus(BtNodeStatus status, BtNodeStatus prev_status) {
    if (node_.status == status && node_.prev_status == prev_status && !on_signal_path_) {
      return;
    }
    node_.prev_status = prev_status;
    node_.status = status;
    update();
  }

  void setOnSignalPath(bool on_path) {
    if (on_signal_path_ == on_path) {
      return;
    }
    on_signal_path_ = on_path;
    update();
  }

  void setReadOnly(bool read_only) {
    read_only_ = read_only;
    prepareGeometryChange();
    recomputeGeometry();
    setFlag(QGraphicsItem::ItemIsMovable, !read_only_);
    update();
  }

  void setMoveStartedNotifier(std::function<void()> notifier) {
    move_started_notifier_ = std::move(notifier);
  }

  void setPortHighlighted(bool highlighted) {
    if (port_highlighted_ == highlighted) {
      return;
    }
    port_highlighted_ = highlighted;
    update();
  }

  void setHasHook(bool has_hook) {
    if (has_hook_ == has_hook) {
      return;
    }
    has_hook_ = has_hook;
    update();
  }

  bool hitExpandButton(const QPointF& local_pos) const {
    return showSubTreeButton() && expandButtonRect().contains(local_pos);
  }

  QPointF connectionPoint(Anchor anchor) const {
    const double body_bottom = bodyBottom();
    const double mid_y = body_bottom * 0.5;
    switch (anchor) {
      case kTopCenter:
        return mapToScene(width_ * 0.5, 0.0);
      case kBottomCenter:
        return mapToScene(width_ * 0.5, body_bottom);
      case kLeftCenter:
        return mapToScene(0.0, mid_y);
      case kRightCenter:
        return mapToScene(width_, mid_y);
    }
    return mapToScene(width_ * 0.5, mid_y);
  }

  void addEdge(BtEdgeItem* edge) {
    if (!edges_.contains(edge)) {
      edges_.append(edge);
    }
  }

  void removeEdge(BtEdgeItem* edge) { edges_.removeAll(edge); }

  void refreshEdges();
  ~BtNodeItem() override;

  QRectF boundingRect() const override {
    if (horizontal_layout_) {
      return QRectF(-kPortRadius, 0.0, width_ + 2.0 * kPortRadius, height_);
    }
    return QRectF(0.0, -kPortRadius, width_, height_ + 2.0 * kPortRadius);
  }

  void paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) override {
    const QRectF rect(0.0, 0.0, width_, height_);
    const bool show_subtree_button = showSubTreeButton();
    const double body_bottom = bodyBottom();
    const QRectF body_rect(0.0, 0.0, width_, body_bottom);

    painter->setRenderHint(QPainter::Antialiasing, true);

    QLinearGradient body_grad(body_rect.topLeft(), body_rect.bottomLeft());
    if (node_.expanded_inline) {
      body_grad.setColorAt(0.0, QColor(90, 120, 200));
      body_grad.setColorAt(1.0, QColor(70, 100, 180));
    } else {
      body_grad.setColorAt(0.0, QColor(102, 102, 102));
      body_grad.setColorAt(1.0, QColor(85, 85, 85));
    }

    QPainterPath shape;
    shape.addRoundedRect(body_rect, kNodeRadius, kNodeRadius);
    painter->fillPath(shape, body_grad);

    if (node_.status != BtNodeStatus::kIdle) {
      const QColor overlay = ColorForStatus(node_.status);
      painter->fillPath(shape, QColor(overlay.red(), overlay.green(), overlay.blue(), 55));
    }

    QPen border_pen;
    if (port_highlighted_) {
      border_pen = QPen(QColor(234, 179, 8), 2.5);
    } else if (isSelected()) {
      border_pen = QPen(QColor(38, 198, 218), 2.5);
    } else if (node_.status != BtNodeStatus::kIdle) {
      // Groot2: active status uses dashed glowing border.
      border_pen = QPen(MonitorNodeBorderColor(node_.status, node_.prev_status), 2.5,
                        Qt::DashLine);
    } else if (on_signal_path_) {
      // Ancestor / pipeline child on the live path — green dashed outline.
      border_pen = QPen(QColor(0, 200, 83), 2.5, Qt::DashLine);
    } else if (node_.prev_status != BtNodeStatus::kIdle) {
      // Idle after a transition: faded dashed border (Groot history hint).
      border_pen = QPen(MonitorNodeBorderColor(node_.status, node_.prev_status), 2.0,
                        Qt::DashLine);
    } else {
      border_pen = QPen(QColor(200, 200, 210), 1.2);
    }
    painter->setPen(border_pen);
    painter->setBrush(Qt::NoBrush);
    painter->drawPath(shape);

    if (port_highlighted_) {
      painter->fillPath(shape, QColor(234, 179, 8, 35));
    } else if (on_signal_path_ && node_.status == BtNodeStatus::kIdle) {
      painter->fillPath(shape, QColor(0, 200, 83, 40));
    }

    double y = kNodePadY;

    // Caption: icon + colored label (Groot BehaviorTreeDataModel)
    QColor title_color = QColor(240, 240, 240);
    if (const auto caption_color =
            BtIconLoader::nodeCaptionColor(node_.registration_id, node_.kind)) {
      title_color = *caption_color;
    } else if (node_.kind == BtNodeKind::kAction) {
      title_color = QColor(QStringLiteral("#ffee77"));
    } else if (node_.kind == BtNodeKind::kCondition) {
      title_color = QColor(QStringLiteral("#44bb44"));
    }

    const QPixmap node_icon =
        BtIconLoader::nodePixmap(node_.registration_id, node_.kind,
                                 static_cast<int>(kNodeIconSize));
    const QString caption =
        BtIconLoader::nodeCaptionLabel(node_.registration_id, node_.kind);
    QFont title_font = painter->font();
    title_font.setBold(true);
    title_font.setPointSize(11);
    painter->setFont(title_font);
    painter->setPen(title_color);

    const QRectF caption_rect(kNodePadX, y, width_ - 2.0 * kNodePadX, kCaptionHeight);
    if (!node_icon.isNull()) {
      painter->drawPixmap(
          QRectF(caption_rect.left(), caption_rect.top() + 2.0, kNodeIconSize, kNodeIconSize),
          node_icon, node_icon.rect());
      painter->drawText(caption_rect.adjusted(kNodeIconSize + 4.0, 0.0, 0.0, 0.0),
                        Qt::AlignLeft | Qt::AlignVCenter, caption);
    } else {
      painter->drawText(caption_rect, Qt::AlignCenter, caption);
    }
    y += kCaptionHeight;

    if (show_instance_) {
      QFont instance_font = painter->font();
      instance_font.setBold(false);
      instance_font.setPointSize(10);
      painter->setFont(instance_font);
      painter->setPen(QColor(245, 245, 245));
      painter->drawText(QRectF(kNodePadX, y, width_ - 2.0 * kNodePadX, kInstanceHeight),
                        Qt::AlignCenter, node_.instance_name);
      y += kInstanceHeight;
    }

    // Port rows: [IN]/[OUT] label + grey value field
    QFont port_font = painter->font();
    port_font.setBold(false);
    port_font.setPointSize(9);
    painter->setFont(port_font);
    const QFontMetrics port_metrics(port_font);

    for (const PortDisplayRow& row : port_rows_) {
      const QRectF row_rect(kNodePadX, y, width_ - 2.0 * kNodePadX, kPortRowHeight);
      const double label_w = label_column_width_;
      const QRectF label_rect(row_rect.left(), row_rect.top(), label_w, row_rect.height());
      const QRectF field_rect(row_rect.left() + label_w + 4.0, row_rect.top() + 1.0,
                              row_rect.width() - label_w - 4.0, row_rect.height() - 2.0);

      painter->setPen(QColor(245, 245, 245));
      painter->drawText(label_rect, Qt::AlignLeft | Qt::AlignVCenter,
                        port_metrics.elidedText(row.label, Qt::ElideRight,
                                                  static_cast<int>(label_rect.width())));

      painter->setPen(Qt::NoPen);
      painter->setBrush(QColor(200, 200, 200));
      painter->drawRoundedRect(field_rect, 2.0, 2.0);
      painter->setPen(QColor(30, 30, 30));
      painter->drawText(field_rect.adjusted(4.0, 0.0, -4.0, 0.0), Qt::AlignCenter,
                        port_metrics.elidedText(row.value, Qt::ElideRight,
                                                  static_cast<int>(field_rect.width() - 8)));
      y += kPortRowHeight + kPortGap;
    }

    if (show_subtree_button) {
      const QRectF button_rect = expandButtonRect();
      painter->setPen(Qt::NoPen);
      painter->setBrush(QColor(255, 255, 255));
      painter->drawRoundedRect(button_rect, 3.0, 3.0);
      QFont button_font = painter->font();
      button_font.setPointSize(9);
      painter->setFont(button_font);
      painter->setPen(QColor(0, 0, 0));
      const QString label =
          node_.subtree_expanded ? QStringLiteral("Collapse") : QStringLiteral("Expand");
      painter->drawText(button_rect, Qt::AlignCenter, label);
    }

    // Groot2-style port dots sit on the node border, same teal as edges.
    painter->setPen(Qt::NoPen);
    painter->setBrush(QColor(QLatin1String(kEdgeIdleTeal)));
    if (horizontal_layout_) {
      if (node_.kind != BtNodeKind::kRoot) {
        painter->drawEllipse(QPointF(0.0, body_bottom * 0.5), kPortRadius, kPortRadius);
      }
      if (!IsLeafKind(node_.kind)) {
        painter->drawEllipse(QPointF(width_, body_bottom * 0.5), kPortRadius, kPortRadius);
      }
    } else {
      if (node_.kind != BtNodeKind::kRoot) {
        painter->drawEllipse(QPointF(width_ * 0.5, 0.0), kPortRadius, kPortRadius);
      }
      if (!IsLeafKind(node_.kind)) {
        painter->drawEllipse(QPointF(width_ * 0.5, body_bottom), kPortRadius, kPortRadius);
      }
    }

    if (!node_.description.isEmpty()) {
      // Groot2 yellow comment bubble (top-right, left of hook badge if present).
      const double badge_x = has_hook_ ? width_ - 28.0 : width_ - 12.0;
      const QRectF bubble(badge_x - 8.0, 4.0, 14.0, 12.0);
      painter->setPen(Qt::NoPen);
      painter->setBrush(QColor(255, 193, 7));
      painter->drawRoundedRect(bubble, 3.0, 3.0);
      QPainterPath tail;
      tail.moveTo(bubble.left() + 3.0, bubble.bottom());
      tail.lineTo(bubble.left() + 1.0, bubble.bottom() + 4.0);
      tail.lineTo(bubble.left() + 7.0, bubble.bottom());
      painter->drawPath(tail);
    }

    if (has_hook_) {
      // Groot2-style breakpoint badge (top-right).
      const QPointF badge(width_ - 10.0, 10.0);
      painter->setPen(QPen(QColor(255, 255, 255), 1.5));
      painter->setBrush(QColor(220, 38, 38));
      painter->drawEllipse(badge, 6.0, 6.0);
    }
  }

 protected:
  QVariant itemChange(GraphicsItemChange change, const QVariant& value) override;

 private:
  bool showSubTreeButton() const {
    return editor_mode_ && !read_only_ && node_.kind == BtNodeKind::kSubTree &&
           !node_.expanded_inline;
  }

  double bodyBottom() const {
    return showSubTreeButton() ? height_ - kSubTreeButtonHeight - 4.0 : height_;
  }

  QRectF expandButtonRect() const {
    const double btn_w = std::min(100.0, width_ - 24.0);
    return QRectF((width_ - btn_w) * 0.5, height_ - kSubTreeButtonHeight - 2.0, btn_w,
                  kSubTreeButtonHeight);
  }

  void recomputeGeometry() {
    port_rows_ = BuildPortRows(node_, has_model_ ? &model_ : nullptr);
    show_instance_ = ShowInstanceName(node_);

    QFont title_font;
    title_font.setBold(true);
    title_font.setPointSize(11);
    const QFontMetrics title_metrics(title_font);
    const QString caption =
        BtIconLoader::nodeCaptionLabel(node_.registration_id, node_.kind);
    double content_w = kNodeIconSize + 8.0 + title_metrics.horizontalAdvance(caption);

    if (show_instance_) {
      QFont instance_font;
      instance_font.setPointSize(10);
      content_w = std::max(
          content_w,
          static_cast<double>(QFontMetrics(instance_font).horizontalAdvance(node_.instance_name)));
    }

    QFont port_font;
    port_font.setPointSize(9);
    const QFontMetrics port_metrics(port_font);
    label_column_width_ = 50.0;
    double field_column_width = 50.0;
    for (const PortDisplayRow& row : port_rows_) {
      label_column_width_ =
          std::max(label_column_width_,
                   static_cast<double>(port_metrics.horizontalAdvance(row.label) + 4));
      field_column_width =
          std::max(field_column_width,
                   static_cast<double>(port_metrics.horizontalAdvance(row.value) + 16));
    }
    if (!port_rows_.isEmpty()) {
      content_w = std::max(content_w, label_column_width_ + 4.0 + field_column_width);
    }

    width_ = std::max(kMinNodeWidth, content_w + 2.0 * kNodePadX);

    height_ = kNodePadY + kCaptionHeight;
    if (show_instance_) {
      height_ += kInstanceHeight;
    }
    if (!port_rows_.isEmpty()) {
      height_ += port_rows_.size() * (kPortRowHeight + kPortGap) + 2.0;
    }
    height_ += kNodePadY;
    if (showSubTreeButton()) {
      height_ += kSubTreeButtonHeight + 4.0;
    }
    // Ensure a minimum body even for empty Root/Control nodes.
    height_ = std::max(height_, kCaptionHeight + kInstanceHeight + 2.0 * kNodePadY);
  }

  int uid_ = 0;
  BtAbsNode node_;
  BtNodeModel model_;
  bool has_model_ = false;
  bool read_only_ = false;
  bool editor_mode_ = true;
  bool horizontal_layout_ = false;
  bool show_instance_ = true;
  double width_ = kMinNodeWidth;
  double height_ = 56.0;
  double label_column_width_ = 50.0;
  QVector<PortDisplayRow> port_rows_;
  bool port_highlighted_ = false;
  bool has_hook_ = false;
  bool on_signal_path_ = false;
  std::function<void()> move_started_notifier_;
  QVector<BtEdgeItem*> edges_;
};

class BtEdgeItem : public QGraphicsObject {
 public:
  BtEdgeItem(BtNodeItem* from, BtNodeItem* to, int child_uid, bool horizontal_layout,
             QGraphicsItem* parent = nullptr)
      : QGraphicsObject(parent),
        from_(from),
        to_(to),
        child_uid_(child_uid),
        horizontal_layout_(horizontal_layout) {
    setZValue(-1.0);
    if (from_ != nullptr) {
      from_->addEdge(this);
    }
    if (to_ != nullptr) {
      to_->addEdge(this);
    }
    updatePath();
  }

  ~BtEdgeItem() override {
    if (from_ != nullptr) {
      from_->removeEdge(this);
      from_ = nullptr;
    }
    if (to_ != nullptr) {
      to_->removeEdge(this);
      to_ = nullptr;
    }
  }

  void detachNode(BtNodeItem* node) {
    if (from_ == node) {
      from_ = nullptr;
    }
    if (to_ == node) {
      to_ = nullptr;
    }
  }

  void updatePath() {
    if (from_ == nullptr || to_ == nullptr) {
      path_ = QPainterPath();
      prepareGeometryChange();
      update();
      return;
    }

    path_ = QPainterPath();
    if (horizontal_layout_) {
      BuildTreeEdgePath(path_, from_->connectionPoint(BtNodeItem::kRightCenter),
                        to_->connectionPoint(BtNodeItem::kLeftCenter), true);
    } else {
      BuildTreeEdgePath(path_, from_->connectionPoint(BtNodeItem::kBottomCenter),
                        to_->connectionPoint(BtNodeItem::kTopCenter), false);
    }
    prepareGeometryChange();
    update();
  }

  int childUid() const { return child_uid_; }

  void setChildStatus(BtNodeStatus status, BtNodeStatus prev_status) {
    if (child_status_ == status && child_prev_status_ == prev_status && !force_flow_ &&
        !on_signal_path_) {
      return;
    }
    child_status_ = status;
    child_prev_status_ = prev_status;
    update();
  }

  void setSignalFlow(bool on_path, bool force_flow) {
    if (on_signal_path_ == on_path && force_flow_ == force_flow) {
      return;
    }
    on_signal_path_ = on_path;
    force_flow_ = force_flow;
    update();
  }

  void setFlowPhase(double phase) {
    if (!isFlowing()) {
      return;
    }
    flow_phase_ = phase;
    update();
  }

  bool isFlowing() const {
    return force_flow_ || child_status_ == BtNodeStatus::kRunning ||
           (child_status_ == BtNodeStatus::kIdle &&
            child_prev_status_ == BtNodeStatus::kRunning);
  }

  QRectF boundingRect() const override {
    return path_.boundingRect().adjusted(-12.0, -12.0, 12.0, 12.0);
  }

  void paint(QPainter* painter, const QStyleOptionGraphicsItem*, QWidget*) override {
    if (path_.isEmpty()) {
      return;
    }

    painter->setRenderHint(QPainter::Antialiasing, true);

    const bool flowing = isFlowing();
    const bool active = flowing || on_signal_path_ || child_status_ != BtNodeStatus::kIdle ||
                        child_prev_status_ != BtNodeStatus::kIdle;
    QColor color{QColor(QLatin1String(kEdgeIdleTeal))};
    if (flowing || on_signal_path_) {
      // Active signal path always uses the RUNNING flow color (overrides SUCCESS green
      // on ancestor edges that sit above a still-RUNNING descendant).
      color = MonitorEdgeColor(BtNodeStatus::kRunning, BtNodeStatus::kIdle);
    } else if (child_status_ != BtNodeStatus::kIdle || child_prev_status_ != BtNodeStatus::kIdle) {
      color = MonitorEdgeColor(child_status_, child_prev_status_);
    }

    if (active) {
      QColor glow = color;
      glow.setAlpha(flowing ? 100 : 55);
      QPen glow_pen{glow};
      glow_pen.setWidthF(kEdgeWidth + (flowing ? 6.0 : 4.0));
      glow_pen.setCapStyle(Qt::RoundCap);
      glow_pen.setJoinStyle(Qt::RoundJoin);
      painter->setPen(glow_pen);
      painter->setBrush(Qt::NoBrush);
      painter->drawPath(path_);
    }

    QPen pen{color};
    pen.setWidthF(flowing ? kEdgeWidth + 1.5 : kEdgeWidth);
    pen.setCapStyle(Qt::RoundCap);
    pen.setJoinStyle(Qt::RoundJoin);
    if (flowing) {
      pen.setStyle(Qt::CustomDashLine);
      pen.setDashPattern({8.0, 5.0});
      pen.setDashOffset(-flow_phase_);
    }
    painter->setPen(pen);
    painter->setBrush(Qt::NoBrush);
    painter->drawPath(path_);

    if (flowing && path_.length() > 1.0 && std::isfinite(flow_phase_)) {
      const double len = path_.length();
      const double t = std::fmod(std::abs(flow_phase_) * 3.5, len);
      const double ratio = std::min(1.0, t / len);
      if (std::isfinite(ratio)) {
        const QPointF tip = path_.pointAtPercent(ratio);
        painter->setPen(Qt::NoPen);
        painter->setBrush(color.lighter(130));
        painter->drawEllipse(tip, 4.5, 4.5);
        painter->setBrush(QColor(255, 255, 255, 200));
        painter->drawEllipse(tip, 2.0, 2.0);
      }
    }
  }

 private:
  BtNodeItem* from_ = nullptr;
  BtNodeItem* to_ = nullptr;
  int child_uid_ = -1;
  bool horizontal_layout_ = false;
  QPainterPath path_;
  BtNodeStatus child_status_ = BtNodeStatus::kIdle;
  BtNodeStatus child_prev_status_ = BtNodeStatus::kIdle;
  bool on_signal_path_ = false;
  bool force_flow_ = false;
  double flow_phase_ = 0.0;
};

void BtNodeItem::refreshEdges() {
  for (BtEdgeItem* edge : edges_) {
    if (edge != nullptr) {
      edge->updatePath();
    }
  }
}

BtNodeItem::~BtNodeItem() {
  const QVector<BtEdgeItem*> edges = edges_;
  edges_.clear();
  for (BtEdgeItem* edge : edges) {
    if (edge != nullptr) {
      edge->detachNode(this);
    }
  }
}

QVariant BtNodeItem::itemChange(GraphicsItemChange change, const QVariant& value) {
  if (change == ItemPositionHasChanged && !read_only_ && move_started_notifier_) {
    move_started_notifier_();
    move_started_notifier_ = nullptr;
  }
  if (change == ItemPositionHasChanged) {
    refreshEdges();
  }
  return QGraphicsObject::itemChange(change, value);
}

}  // namespace

QMimeData* MakeBtNodeDragPayload(const QString& registration_id, BtNodeKind kind) {
  auto* mime = new QMimeData();
  const QByteArray payload =
      registration_id.toUtf8() + '|' + QByteArray::number(static_cast<int>(kind));
  mime->setData(QString::fromLatin1(kBtNodeDragMime), payload);
  return mime;
}

bool ReadBtNodeDragPayload(const QMimeData* mime, QString* registration_id, BtNodeKind* kind) {
  if (mime == nullptr || registration_id == nullptr || kind == nullptr ||
      !mime->hasFormat(QString::fromLatin1(kBtNodeDragMime))) {
    return false;
  }
  const QString text = QString::fromUtf8(mime->data(QString::fromLatin1(kBtNodeDragMime)));
  const int sep = text.lastIndexOf('|');
  if (sep <= 0) {
    return false;
  }
  bool ok = false;
  const int kind_int = text.mid(sep + 1).toInt(&ok);
  if (!ok) {
    return false;
  }
  *registration_id = text.left(sep);
  *kind = static_cast<BtNodeKind>(kind_int);
  return !registration_id->isEmpty();
}

BtGraphView::BtGraphView(QWidget* parent) : QGraphicsView(parent) {
  scene_ = new QGraphicsScene(this);
  setScene(scene_);
  setRenderHint(QPainter::Antialiasing, true);
  setRenderHint(QPainter::TextAntialiasing, true);
  setDragMode(QGraphicsView::NoDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setResizeAnchor(QGraphicsView::AnchorViewCenter);
  setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
  setBackgroundBrush(QColor(QLatin1String(kCanvasBg)));
  setAcceptDrops(true);
  setFocusPolicy(Qt::StrongFocus);

  flow_timer_ = new QTimer(this);
  flow_timer_->setInterval(33);
  connect(flow_timer_, &QTimer::timeout, this, [this]() {
    if (scene_ == nullptr) {
      return;
    }
    bool any_flowing = false;
    flow_phase_ += 2.5;
    for (QGraphicsItem* item : scene_->items()) {
      if (auto* edge = dynamic_cast<BtEdgeItem*>(item)) {
        if (edge->isFlowing()) {
          any_flowing = true;
          edge->setFlowPhase(flow_phase_);
        }
      }
    }
    if (!any_flowing && flow_timer_->isActive()) {
      // Keep timer cheap when idle; caller may leave it running in Monitor.
    }
  });

  connect(scene_, &QGraphicsScene::selectionChanged, this, &BtGraphView::emitSelectionFromScene);
}

void BtGraphView::setFlowAnimationEnabled(bool enabled) {
  if (flow_timer_ == nullptr) {
    return;
  }
  if (enabled) {
    if (!flow_timer_->isActive()) {
      flow_timer_->start();
    }
  } else {
    flow_timer_->stop();
    flow_phase_ = 0.0;
  }
}

bool BtGraphView::applyNodeEditor(int uid, const QString& instance_name, const QString& skip_if,
                                  const QString& success_if, const QString& failure_if,
                                  const QString& while_script, const QString& on_success,
                                  const QString& on_failure, const QString& on_halted,
                                  const QString& post_script, const QString& description) {
  if (read_only_ || !tree_.nodes.contains(uid)) {
    return false;
  }
  BtAbsNode& node = tree_.nodes[uid];
  if (node.kind == BtNodeKind::kRoot || node.expanded_inline) {
    return false;
  }
  pushSnapshot();
  node.instance_name = instance_name;
  node.skip_if = skip_if;
  node.success_if = success_if;
  node.failure_if = failure_if;
  node.while_script = while_script;
  node.on_success = on_success;
  node.on_failure = on_failure;
  node.on_halted = on_halted;
  node.post_script = post_script;
  node.description = description;

  for (QGraphicsItem* item : scene_->items()) {
    if (auto* node_item = dynamic_cast<BtNodeItem*>(item)) {
      if (node_item->uid() == uid) {
        const BtNodeModel* model = nullptr;
        auto model_it = models_.constFind(node.registration_id);
        if (model_it != models_.constEnd()) {
          model = &model_it.value();
        }
        node_item->setNodeData(node, model);
        break;
      }
    }
  }
  emit treeChanged();
  return true;
}

void BtGraphView::drawBackground(QPainter* painter, const QRectF& rect) {
  painter->fillRect(rect, QColor(QLatin1String(kCanvasBg)));
}

void BtGraphView::setTree(BtAbsTree tree, const QHash<QString, BtNodeModel>& models) {
  tree_ = std::move(tree);
  models_ = models;
  ReresolveTreeNodeKinds(tree_, models_);
  undo_stack_.clear();
  redo_stack_.clear();
  rebuildScene();
}

BtAbsTree BtGraphView::tree() const {
  BtAbsTree result = tree_;
  for (QGraphicsItem* item : scene_->items()) {
    auto* node_item = dynamic_cast<BtNodeItem*>(item);
    if (node_item == nullptr || !result.nodes.contains(node_item->uid())) {
      continue;
    }
    result.nodes[node_item->uid()].pos = node_item->pos();
  }
  return result;
}

BtAbsTree BtGraphView::serializableTree() const {
  return StripExpandedInlineNodes(tree());
}

void BtGraphView::setSubTreeLookup(SubTreeLookup lookup) {
  subtree_lookup_ = std::move(lookup);
}

bool BtGraphView::nodeIsLocked(const BtAbsNode& node) const {
  return node.expanded_inline;
}

bool BtGraphView::toggleSubTreeExpand(int uid) {
  if (!tree_.nodes.contains(uid)) {
    return false;
  }
  if (tree_.nodes.value(uid).subtree_expanded) {
    return collapseSubTree(uid);
  }
  return expandSubTree(uid);
}

bool BtGraphView::expandSubTree(int uid) {
  if (read_only_ || !tree_.nodes.contains(uid)) {
    return false;
  }
  BtAbsNode& subtree_node = tree_.nodes[uid];
  if (subtree_node.kind != BtNodeKind::kSubTree) {
    return false;
  }
  if (subtree_node.subtree_expanded) {
    return true;
  }

  const QString subtree_id = SubTreeIdFromNode(subtree_node);
  if (subtree_id.isEmpty() || !subtree_lookup_) {
    return false;
  }
  const BtAbsTree* source = subtree_lookup_(subtree_id);
  if (source == nullptr) {
    return false;
  }
  if (!IsValidBehaviorTree(*source)) {
    QMessageBox::warning(this, tr("Expand SubTree"),
                         tr("Invalid SubTree [%1]. Cannot expand.").arg(subtree_id));
    return false;
  }

  pushSnapshot();
  if (!appendSubTreeInline(uid, *source)) {
    if (!undo_stack_.isEmpty()) {
      undo_stack_.removeLast();
    }
    return false;
  }

  subtree_node.subtree_expanded = true;
  if (tree_.nodes.value(uid).children.size() > 1) {
    applyAutoLayout(false);
  }
  rebuildScene();
  fitToView();
  emit subTreeExpandToggled(uid, true);
  emit treeChanged();
  return true;
}

bool BtGraphView::collapseSubTree(int uid) {
  if (read_only_ || !tree_.nodes.contains(uid)) {
    return false;
  }
  BtAbsNode& subtree_node = tree_.nodes[uid];
  if (!subtree_node.subtree_expanded) {
    return true;
  }

  pushSnapshot();
  removeExpandedInline(uid);
  subtree_node.subtree_expanded = false;
  subtree_node.children.clear();
  rebuildScene();
  emit subTreeExpandToggled(uid, false);
  emit treeChanged();
  return true;
}

void BtGraphView::refreshExpandedSubTree(const QString& subtree_id) {
  if (subtree_id.isEmpty() || !subtree_lookup_) {
    return;
  }
  const BtAbsTree* source = subtree_lookup_(subtree_id);
  if (source == nullptr || !IsValidBehaviorTree(*source)) {
    return;
  }

  QVector<int> expanded_uids;
  for (auto it = tree_.nodes.constBegin(); it != tree_.nodes.constEnd(); ++it) {
    if (it.value().kind == BtNodeKind::kSubTree && it.value().subtree_expanded &&
        SubTreeIdFromNode(it.value()) == subtree_id) {
      expanded_uids.push_back(it.key());
    }
  }
  if (expanded_uids.isEmpty()) {
    return;
  }

  pushSnapshot();
  for (int uid : expanded_uids) {
    removeExpandedInline(uid);
    BtAbsNode& subtree_node = tree_.nodes[uid];
    subtree_node.subtree_expanded = false;
    subtree_node.children.clear();
    if (appendSubTreeInline(uid, *source)) {
      subtree_node.subtree_expanded = true;
    }
  }
  rebuildScene();
  emit treeChanged();
}

void BtGraphView::collapseExpandedSubTreesById(const QString& subtree_id) {
  if (subtree_id.isEmpty()) {
    return;
  }
  QVector<int> expanded_uids;
  for (auto it = tree_.nodes.constBegin(); it != tree_.nodes.constEnd(); ++it) {
    if (it.value().kind == BtNodeKind::kSubTree && it.value().subtree_expanded &&
        SubTreeIdFromNode(it.value()) == subtree_id) {
      expanded_uids.push_back(it.key());
    }
  }
  for (int uid : expanded_uids) {
    collapseSubTree(uid);
  }
}

bool BtGraphView::appendSubTreeInline(int subtree_uid, const BtAbsTree& source) {
  if (!tree_.nodes.contains(subtree_uid)) {
    return false;
  }

  int entry_uid = source.root_uid;
  if (!source.nodes.contains(entry_uid)) {
    return false;
  }
  const BtAbsNode& source_root = source.nodes.value(entry_uid);
  if (source_root.registration_id == QLatin1String("Root")) {
    if (source_root.children.isEmpty()) {
      return false;
    }
    if (source_root.children.size() != 1) {
      return false;
    }
    entry_uid = source_root.children.first();
  }
  if (!source.nodes.contains(entry_uid)) {
    return false;
  }

  QSet<int> copy_uids;
  std::function<void(int)> collect = [&](int node_uid) {
    if (!source.nodes.contains(node_uid) || copy_uids.contains(node_uid)) {
      return;
    }
    copy_uids.insert(node_uid);
    for (int child_uid : source.nodes.value(node_uid).children) {
      collect(child_uid);
    }
  };
  collect(entry_uid);

  QHash<int, int> uid_map;
  uid_map.reserve(copy_uids.size());
  for (int old_uid : copy_uids) {
    uid_map.insert(old_uid, NextUid(tree_));
  }

  const QPointF anchor = tree_.nodes.value(subtree_uid).pos + QPointF(120.0, 120.0);
  const QPointF entry_pos = source.nodes.value(entry_uid).pos;

  for (int old_uid : copy_uids) {
    BtAbsNode copied = source.nodes.value(old_uid);
    copied.uid = uid_map.value(old_uid);
    copied.expanded_inline = true;
    copied.expanded_owner_uid = subtree_uid;
    copied.children.clear();
    copied.subtree_expanded = false;
    copied.pos = anchor + (copied.pos - entry_pos);
    tree_.nodes.insert(copied.uid, copied);
  }

  for (int old_uid : copy_uids) {
    BtAbsNode& copied = tree_.nodes[uid_map.value(old_uid)];
    for (int old_child : source.nodes.value(old_uid).children) {
      if (uid_map.contains(old_child)) {
        copied.children.push_back(uid_map.value(old_child));
      }
    }
  }

  BtAbsNode& subtree_node = tree_.nodes[subtree_uid];
  subtree_node.children.clear();
  subtree_node.children.push_back(uid_map.value(entry_uid));
  return true;
}

void BtGraphView::removeExpandedInline(int subtree_uid) {
  QSet<int> remove_uids;
  for (int uid : CollectExpandedInlineNodes(tree_, subtree_uid)) {
    remove_uids.insert(uid);
  }
  if (tree_.nodes.contains(subtree_uid)) {
    for (int child_uid : tree_.nodes.value(subtree_uid).children) {
      if (tree_.nodes.contains(child_uid) && tree_.nodes.value(child_uid).expanded_inline) {
        const QVector<int> collected = CollectSubtree(tree_, child_uid);
        for (int uid : collected) {
          remove_uids.insert(uid);
        }
      }
    }
  }

  for (auto it = tree_.nodes.begin(); it != tree_.nodes.end(); ++it) {
    QVector<int>& children = it.value().children;
    children.erase(std::remove_if(children.begin(), children.end(),
                                  [&remove_uids](int child_uid) {
                                    return remove_uids.contains(child_uid);
                                  }),
                   children.end());
  }
  for (int uid : remove_uids) {
    tree_.nodes.remove(uid);
  }
}

void BtGraphView::setReadOnly(bool read_only) {
  read_only_ = read_only;
  for (QGraphicsItem* item : scene_->items()) {
    if (auto* node_item = dynamic_cast<BtNodeItem*>(item)) {
      node_item->setReadOnly(read_only);
    }
  }
  setAcceptDrops(!read_only);
}

void BtGraphView::setNodeStatus(int uid, BtNodeStatus status,
                                std::optional<BtNodeStatus> previous, bool refresh_flow) {
  if (!tree_.nodes.contains(uid)) {
    return;
  }
  BtAbsNode& node = tree_.nodes[uid];
  node.prev_status = previous.value_or(node.status);
  node.status = status;
  for (QGraphicsItem* item : scene_->items()) {
    auto* node_item = dynamic_cast<BtNodeItem*>(item);
    if (node_item != nullptr && node_item->uid() == uid) {
      node_item->setStatus(node.status, node.prev_status);
      break;
    }
  }
  for (QGraphicsItem* item : scene_->items()) {
    auto* edge_item = dynamic_cast<BtEdgeItem*>(item);
    if (edge_item != nullptr && edge_item->childUid() == uid) {
      edge_item->setChildStatus(node.status, node.prev_status);
    }
  }
  if (refresh_flow) {
    refreshSignalFlow();
  }
}

void BtGraphView::clearStatuses() {
  for (auto it = tree_.nodes.begin(); it != tree_.nodes.end(); ++it) {
    it.value().prev_status = BtNodeStatus::kIdle;
    it.value().status = BtNodeStatus::kIdle;
  }
  for (QGraphicsItem* item : scene_->items()) {
    if (auto* node_item = dynamic_cast<BtNodeItem*>(item)) {
      node_item->setStatus(BtNodeStatus::kIdle, BtNodeStatus::kIdle);
      node_item->setOnSignalPath(false);
    } else if (auto* edge_item = dynamic_cast<BtEdgeItem*>(item)) {
      edge_item->setChildStatus(BtNodeStatus::kIdle, BtNodeStatus::kIdle);
      edge_item->setSignalFlow(false, false);
    }
  }
}

void BtGraphView::refreshSignalFlow() {
  if (scene_ == nullptr) {
    return;
  }

  // Live path = ancestors of every RUNNING node, plus full active branches under
  // PipelineSequence/Parallel. RateController often sits SUCCESS/IDLE between
  // hz ticks while FollowPath is RUNNING — its Replan child must still light.
  QSet<int> path_uids;
  QSet<int> running_uids;

  auto is_fanout_control = [](const BtAbsNode& node) -> bool {
    const QString& id = node.registration_id;
    return id.contains(QLatin1String("Pipeline"), Qt::CaseInsensitive) ||
           id.contains(QLatin1String("Parallel"), Qt::CaseInsensitive);
  };

  for (auto it = tree_.nodes.constBegin(); it != tree_.nodes.constEnd(); ++it) {
    if (it.value().status != BtNodeStatus::kRunning) {
      continue;
    }
    running_uids.insert(it.key());
    int cursor = it.key();
    while (cursor >= 0 && tree_.nodes.contains(cursor)) {
      if (path_uids.contains(cursor)) {
        break;
      }
      path_uids.insert(cursor);
      cursor = FindParentUid(tree_, cursor);
    }
  }

  // expand_full: light the entire non-FAILURE subtree (Pipeline children, and
  // anything reached through that fan-out — e.g. RateController → Replan).
  QVector<int> down_stack;
  QSet<int> down_seen;
  QSet<int> expand_full;

  auto enqueue = [&](int uid, bool full_subtree) {
    if (!tree_.nodes.contains(uid) || down_seen.contains(uid)) {
      return;
    }
    down_seen.insert(uid);
    down_stack.push_back(uid);
    if (full_subtree) {
      expand_full.insert(uid);
    }
  };

  for (int uid : running_uids) {
    enqueue(uid, is_fanout_control(tree_.nodes.value(uid)));
  }
  for (int uid : path_uids) {
    if (tree_.nodes.contains(uid) && is_fanout_control(tree_.nodes.value(uid))) {
      enqueue(uid, true);
    }
  }

  while (!down_stack.isEmpty()) {
    const int uid = down_stack.takeLast();
    if (!tree_.nodes.contains(uid)) {
      continue;
    }
    const BtAbsNode& node = tree_.nodes.value(uid);
    const bool fanout = is_fanout_control(node);
    const bool full = expand_full.contains(uid) || fanout;
    const bool decorator_running =
        node.status == BtNodeStatus::kRunning &&
        (node.kind == BtNodeKind::kDecorator ||
         node.registration_id.contains(QLatin1String("RateController"),
                                       Qt::CaseInsensitive));

    for (int child_uid : node.children) {
      if (!tree_.nodes.contains(child_uid)) {
        continue;
      }
      const BtAbsNode& child = tree_.nodes.value(child_uid);
      // Pipeline branch: light every non-FAILURE descendant (Idle RateController
      // still opens Replan / PlanPath). Otherwise only RUNNING/SUCCESS children.
      if (child.status == BtNodeStatus::kFailure) {
        continue;
      }
      const bool take = full || decorator_running ||
                        child.status == BtNodeStatus::kRunning ||
                        child.status == BtNodeStatus::kSuccess;
      if (!take) {
        continue;
      }

      path_uids.insert(child_uid);

      // Keep walking the Pipeline subtree (and running decorator children).
      if (full || decorator_running || child.status == BtNodeStatus::kRunning) {
        enqueue(child_uid, /*full_subtree=*/full || fanout);
      }
    }
  }

  for (QGraphicsItem* item : scene_->items()) {
    if (auto* node_item = dynamic_cast<BtNodeItem*>(item)) {
      node_item->setOnSignalPath(path_uids.contains(node_item->uid()));
    } else if (auto* edge_item = dynamic_cast<BtEdgeItem*>(item)) {
      const int child_uid = edge_item->childUid();
      const bool on_path = path_uids.contains(child_uid);
      edge_item->setSignalFlow(on_path, /*force_flow=*/on_path);
    }
  }

  setFlowAnimationEnabled(!running_uids.isEmpty());
}

void BtGraphView::fitToView() {
  const QRectF bounds = scene_->itemsBoundingRect().adjusted(-80.0, -60.0, 80.0, 80.0);
  if (bounds.isEmpty()) {
    return;
  }
  fitInView(bounds, Qt::KeepAspectRatio);
  zoom_factor_ = transform().m11();
}

void BtGraphView::applyAutoLayout(bool horizontal) {
  if (read_only_) {
    return;
  }
  pushSnapshot();
  syncPositionsFromScene();
  horizontal_layout_ = horizontal;

  QHash<int, QSizeF> node_sizes;
  for (QGraphicsItem* item : scene_->items()) {
    if (auto* node_item = dynamic_cast<BtNodeItem*>(item)) {
      node_sizes.insert(node_item->uid(),
                        QSizeF(node_item->visualWidth(), node_item->visualHeight()));
    }
  }

  if (horizontal) {
    ApplyHorizontalTreeLayout(tree_, node_sizes);
  } else {
    ApplyVerticalTreeLayout(tree_, node_sizes);
  }
  rebuildScene();
  fitToView();
  emit treeChanged();
}

bool BtGraphView::exportSvg(const QString& path) const {
  if (path.isEmpty() || scene_ == nullptr) {
    return false;
  }
  const QRectF bounds = scene_->itemsBoundingRect().adjusted(-40.0, -40.0, 40.0, 40.0);
  if (bounds.isEmpty()) {
    return false;
  }

  QSvgGenerator generator;
  generator.setFileName(path);
  generator.setSize(bounds.size().toSize().expandedTo(QSize(200, 200)));
  generator.setViewBox(bounds);
  generator.setTitle(tree_.tree_id);
  generator.setDescription(QStringLiteral("BehaviorTree export"));

  QPainter painter;
  if (!painter.begin(&generator)) {
    return false;
  }
  scene_->render(&painter, bounds, bounds);
  painter.end();
  return true;
}

int BtGraphView::selectedUid() const {
  const QList<QGraphicsItem*> selected = scene_->selectedItems();
  for (QGraphicsItem* item : selected) {
    if (auto* node_item = dynamic_cast<BtNodeItem*>(item)) {
      return node_item->uid();
    }
  }
  return -1;
}

void BtGraphView::deleteSelected() {
  if (read_only_) {
    return;
  }
  const int uid = selectedUid();
  if (uid < 0 || !tree_.nodes.contains(uid)) {
    return;
  }
  if (tree_.nodes.value(uid).expanded_inline) {
    return;
  }
  pushSnapshot();
  if (tree_.nodes.value(uid).kind == BtNodeKind::kSubTree &&
      tree_.nodes.value(uid).subtree_expanded) {
    removeExpandedInline(uid);
    tree_.nodes[uid].subtree_expanded = false;
    tree_.nodes[uid].children.clear();
  }
  removeSubtree(uid);
  rebuildScene();
  emit treeChanged();
  emit selectionChanged(-1);
}

void BtGraphView::smartRemoveSelected() {
  if (read_only_) {
    return;
  }
  const int uid = selectedUid();
  if (uid < 0 || uid == tree_.root_uid) {
    return;
  }
  if (!tree_.nodes.contains(uid)) {
    return;
  }
  const BtAbsNode& node = tree_.nodes.value(uid);
  if (node.children.isEmpty()) {
    return;
  }
  const int parent_uid = FindParentUid(tree_, uid);
  if (parent_uid < 0 || !tree_.nodes.contains(parent_uid)) {
    return;
  }
  const BtAbsNode& parent = tree_.nodes.value(parent_uid);
  if (MaxChildCount(parent.kind) < parent.children.size() - 1 + node.children.size()) {
    return;
  }

  pushSnapshot();
  removeNodeKeepChildren(uid);
  rebuildScene();
  emit treeChanged();
  emit selectionChanged(-1);
}

void BtGraphView::morphSelected(const QString& registration_id) {
  if (read_only_ || registration_id.isEmpty()) {
    return;
  }
  const int uid = selectedUid();
  if (uid < 0 || !tree_.nodes.contains(uid)) {
    return;
  }
  if (tree_.nodes.value(uid).expanded_inline) {
    return;
  }

  pushSnapshot();
  BtAbsNode& node = tree_.nodes[uid];
  const QString previous_instance = node.instance_name;
  const QHash<QString, QString> previous_remap = node.port_remap;
  node.registration_id = registration_id;
  if (models_.contains(registration_id)) {
    node.kind = models_.value(registration_id).kind;
  }
  if (previous_instance == registration_id || previous_instance.isEmpty()) {
    node.instance_name = UniqueInstanceName(tree_, registration_id);
  }

  QHash<QString, QString> filtered_remap;
  if (models_.contains(registration_id)) {
    const BtNodeModel& model = models_.value(registration_id);
    for (const BtPortModel& port : model.ports) {
      if (previous_remap.contains(port.name) && !previous_remap.value(port.name).isEmpty()) {
        filtered_remap.insert(port.name, previous_remap.value(port.name));
      }
    }
  }
  node.port_remap = filtered_remap;
  rebuildScene();
  emit treeChanged();
  emit selectionChanged(uid);
}

std::optional<BtAbsTree> BtGraphView::extractSubtree(int uid, const QString& subtree_id) {
  if (read_only_ || subtree_id.isEmpty() || !tree_.nodes.contains(uid)) {
    return std::nullopt;
  }
  if (uid == tree_.root_uid) {
    return std::nullopt;
  }

  const BtAbsNode& source_root = tree_.nodes.value(uid);
  if (!KindAcceptsChildren(source_root.kind)) {
    return std::nullopt;
  }

  pushSnapshot();
  syncPositionsFromScene();

  const QVector<int> collected = CollectSubtree(tree_, uid);
  QHash<int, int> uid_map;
  BtAbsTree extracted;
  extracted.tree_id = subtree_id;

  for (int old_uid : collected) {
    uid_map.insert(old_uid, NextUid(extracted));
    BtAbsNode copied = tree_.nodes.value(old_uid);
    copied.uid = uid_map.value(old_uid);
    copied.children.clear();
    extracted.nodes.insert(copied.uid, copied);
  }
  extracted.root_uid = uid_map.value(uid);

  for (int old_uid : collected) {
    const BtAbsNode& old_node = tree_.nodes.value(old_uid);
    BtAbsNode& new_node = extracted.nodes[uid_map.value(old_uid)];
    for (int old_child : old_node.children) {
      if (uid_map.contains(old_child)) {
        new_node.children.push_back(uid_map.value(old_child));
      }
    }
  }

  const int parent_uid = FindParentUid(tree_, uid);
  int sibling_index = -1;
  if (parent_uid >= 0 && tree_.nodes.contains(parent_uid)) {
    sibling_index = tree_.nodes.value(parent_uid).children.indexOf(uid);
  }
  const QPointF keep_pos = source_root.pos;
  removeSubtree(uid);

  BtAbsNode subtree_node;
  subtree_node.uid = NextUid(tree_);
  subtree_node.registration_id = QStringLiteral("SubTree");
  subtree_node.kind = BtNodeKind::kSubTree;
  subtree_node.instance_name = UniqueInstanceName(tree_, subtree_id);
  subtree_node.pos = keep_pos;
  subtree_node.port_remap.insert(QStringLiteral("ID"), subtree_id);
  tree_.nodes.insert(subtree_node.uid, subtree_node);

  if (parent_uid >= 0 && tree_.nodes.contains(parent_uid)) {
    BtAbsNode& parent = tree_.nodes[parent_uid];
    if (sibling_index < 0 || sibling_index > parent.children.size()) {
      parent.children.push_back(subtree_node.uid);
    } else {
      parent.children.insert(sibling_index, subtree_node.uid);
    }
  } else if (tree_.root_uid < 0) {
    tree_.root_uid = subtree_node.uid;
  }

  rebuildScene();
  emit treeChanged();
  emit selectionChanged(subtree_node.uid);
  return extracted;
}

void BtGraphView::setHighlightedPortValue(const QString& value) {
  highlighted_port_value_ = value.trimmed();
  for (QGraphicsItem* item : scene_->items()) {
    auto* node_item = dynamic_cast<BtNodeItem*>(item);
    if (node_item == nullptr || !tree_.nodes.contains(node_item->uid())) {
      continue;
    }
    bool match = false;
    if (!highlighted_port_value_.isEmpty()) {
      const BtAbsNode& node = tree_.nodes.value(node_item->uid());
      for (auto it = node.port_remap.constBegin(); it != node.port_remap.constEnd(); ++it) {
        if (it.value() == highlighted_port_value_) {
          match = true;
          break;
        }
      }
    }
    node_item->setPortHighlighted(match);
  }
}

void BtGraphView::clearPortHighlight() {
  setHighlightedPortValue(QString());
}

void BtGraphView::setHookNodeUids(const QSet<int>& uids) {
  hook_node_uids_ = uids;
  for (QGraphicsItem* item : scene_->items()) {
    if (auto* node_item = dynamic_cast<BtNodeItem*>(item)) {
      node_item->setHasHook(hook_node_uids_.contains(node_item->uid()));
    }
  }
}

void BtGraphView::clearHookMarkers() {
  setHookNodeUids({});
}

void BtGraphView::addChildNode(int parent_uid, const QString& registration_id, BtNodeKind kind) {
  if (read_only_ || registration_id.isEmpty()) {
    return;
  }

  int actual_parent = parent_uid;
  if (actual_parent < 0) {
    if (tree_.root_uid < 0) {
      actual_parent = -1;
    } else {
      actual_parent = selectedUid();
      if (actual_parent < 0) {
        actual_parent = tree_.root_uid;
      }
    }
  } else if (!tree_.nodes.contains(actual_parent)) {
    return;
  }

  pushSnapshot();

  const int uid = NextUid(tree_);
  BtAbsNode node;
  node.uid = uid;
  node.registration_id = registration_id;
  node.kind = kind;
  if (models_.contains(registration_id)) {
    node.kind = models_.value(registration_id).kind;
  }
  node.instance_name = UniqueInstanceName(tree_, registration_id);
  node.status = BtNodeStatus::kIdle;

  if (actual_parent < 0) {
    node.pos = QPointF(0.0, 0.0);
    tree_.nodes.insert(uid, node);
    tree_.root_uid = uid;
  } else {
    BtAbsNode& parent = tree_.nodes[actual_parent];
    node.pos = parent.pos + QPointF(0.0, 120.0);
    tree_.nodes.insert(uid, node);
    parent.children.push_back(uid);
  }

  rebuildScene();
  for (QGraphicsItem* item : scene_->items()) {
    if (auto* node_item = dynamic_cast<BtNodeItem*>(item)) {
      if (node_item->uid() == uid) {
        scene_->clearSelection();
        node_item->setSelected(true);
        break;
      }
    }
  }
  emit treeChanged();
  emit selectionChanged(uid);
}

bool BtGraphView::canUndo() const { return !undo_stack_.isEmpty(); }

bool BtGraphView::canRedo() const { return !redo_stack_.isEmpty(); }

void BtGraphView::undo() {
  if (undo_stack_.isEmpty()) {
    return;
  }
  syncPositionsFromScene();
  redo_stack_.push_back(tree_);
  restoreSnapshot(undo_stack_.takeLast());
  emit treeChanged();
}

void BtGraphView::redo() {
  if (redo_stack_.isEmpty()) {
    return;
  }
  syncPositionsFromScene();
  undo_stack_.push_back(tree_);
  restoreSnapshot(redo_stack_.takeLast());
  emit treeChanged();
}

void BtGraphView::wheelEvent(QWheelEvent* event) {
  const double factor = event->angleDelta().y() > 0 ? 1.12 : 1.0 / 1.12;
  const double next_zoom = std::clamp(zoom_factor_ * factor, kMinZoom, kMaxZoom);
  const double applied = next_zoom / zoom_factor_;
  zoom_factor_ = next_zoom;
  scale(applied, applied);
  event->accept();
}

void BtGraphView::mousePressEvent(QMouseEvent* event) {
  const bool left = event->button() == Qt::LeftButton;
  const bool middle = event->button() == Qt::MiddleButton;

  // Left-drag pans the canvas unless the user is moving a node in Editor.
  // Middle-drag and Space+Left always pan.
  if (middle || (left && space_pan_)) {
    panning_ = true;
    last_pan_pos_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }

  if (left) {
    QGraphicsItem* raw = itemAt(event->pos());
    BtNodeItem* node_item = dynamic_cast<BtNodeItem*>(raw);
    if (node_item == nullptr && raw != nullptr) {
      node_item = dynamic_cast<BtNodeItem*>(raw->parentItem());
    }

    if (node_item != nullptr && !read_only_) {
      const int uid = node_item->uid();
      if (tree_.nodes.contains(uid)) {
        const BtAbsNode& node = tree_.nodes.value(uid);
        if (node.kind == BtNodeKind::kSubTree && !node.expanded_inline) {
          const QPointF local_pos = node_item->mapFromScene(mapToScene(event->pos()));
          if (node_item->hitExpandButton(local_pos)) {
            toggleSubTreeExpand(uid);
            event->accept();
            return;
          }
        }
      }
      // Editor: drag the node itself.
      move_snapshot_pending_ = true;
      QGraphicsView::mousePressEvent(event);
      return;
    }

    // Empty canvas, edges, or Monitor/Replay: left-drag pans the tree.
    panning_ = true;
    last_pan_pos_ = event->pos();
    setCursor(Qt::ClosedHandCursor);
    event->accept();
    return;
  }

  QGraphicsView::mousePressEvent(event);
}

void BtGraphView::mouseMoveEvent(QMouseEvent* event) {
  if (panning_) {
    const QPoint delta = event->pos() - last_pan_pos_;
    last_pan_pos_ = event->pos();
    horizontalScrollBar()->setValue(horizontalScrollBar()->value() - delta.x());
    verticalScrollBar()->setValue(verticalScrollBar()->value() - delta.y());
    event->accept();
    return;
  }
  QGraphicsView::mouseMoveEvent(event);
}

void BtGraphView::mouseReleaseEvent(QMouseEvent* event) {
  if (panning_ && (event->button() == Qt::MiddleButton || event->button() == Qt::LeftButton)) {
    panning_ = false;
    setCursor(space_pan_ ? Qt::OpenHandCursor : Qt::ArrowCursor);
    event->accept();
    return;
  }

  move_snapshot_pending_ = false;

  if (!read_only_ && event->button() == Qt::LeftButton) {
    bool moved = false;
    for (QGraphicsItem* item : scene_->items()) {
      auto* node_item = dynamic_cast<BtNodeItem*>(item);
      if (node_item == nullptr || !tree_.nodes.contains(node_item->uid())) {
        continue;
      }
      if (tree_.nodes[node_item->uid()].pos != node_item->pos()) {
        moved = true;
        break;
      }
    }
    syncPositionsFromScene();
    if (moved) {
      emit treeChanged();
    }
  }
  QGraphicsView::mouseReleaseEvent(event);
}

void BtGraphView::mouseDoubleClickEvent(QMouseEvent* event) {
  QGraphicsItem* item = itemAt(event->pos());
  if (auto* node_item = dynamic_cast<BtNodeItem*>(item)) {
    const int uid = node_item->uid();
    if (tree_.nodes.contains(uid)) {
      const BtAbsNode& node = tree_.nodes.value(uid);
      if (node.kind == BtNodeKind::kSubTree) {
        if (!read_only_) {
          toggleSubTreeExpand(uid);
        } else {
          const QString tree_id = SubTreeIdFromNode(node);
          if (!tree_id.isEmpty()) {
            emit subtreeExpandRequested(tree_id);
          }
        }
        event->accept();
        return;
      }
    }
    emit nodeDoubleClicked(uid);
    event->accept();
    return;
  }
  QGraphicsView::mouseDoubleClickEvent(event);
}

void BtGraphView::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
    space_pan_ = true;
    setCursor(Qt::OpenHandCursor);
    event->accept();
    return;
  }
  if (!read_only_ && event->matches(QKeySequence::Undo)) {
    undo();
    event->accept();
    return;
  }
  if (!read_only_ && event->matches(QKeySequence::Redo)) {
    redo();
    event->accept();
    return;
  }
  if (!read_only_ && event->key() == Qt::Key_Delete) {
    deleteSelected();
    event->accept();
    return;
  }
  QGraphicsView::keyPressEvent(event);
}

void BtGraphView::keyReleaseEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Space && !event->isAutoRepeat()) {
    space_pan_ = false;
    if (!panning_) {
      setCursor(Qt::ArrowCursor);
    }
    event->accept();
    return;
  }
  QGraphicsView::keyReleaseEvent(event);
}

void BtGraphView::contextMenuEvent(QContextMenuEvent* event) {
  QMenu menu(this);

  const int uid = selectedUid();
  BtAbsNode selected_node;
  bool has_selection = false;
  if (uid >= 0 && tree_.nodes.contains(uid)) {
    selected_node = tree_.nodes.value(uid);
    has_selection = true;
  }

  if (has_selection && selected_node.kind == BtNodeKind::kSubTree) {
    const QString subtree_id = SubTreeIdFromNode(selected_node);
    if (!subtree_id.isEmpty()) {
      if (!read_only_) {
        if (selected_node.subtree_expanded) {
          QAction* collapse_action = menu.addAction(tr("Collapse SubTree"));
          connect(collapse_action, &QAction::triggered, this,
                  [this, uid]() { collapseSubTree(uid); });
        } else {
          QAction* expand_action = menu.addAction(tr("Expand SubTree"));
          connect(expand_action, &QAction::triggered, this,
                  [this, uid]() { expandSubTree(uid); });
        }
      }
      QAction* open_tab_action = menu.addAction(tr("Open SubTree Tab"));
      connect(open_tab_action, &QAction::triggered, this,
              [this, subtree_id]() { emit subtreeExpandRequested(subtree_id); });
      menu.addSeparator();
    }
  }

  QMenu* morph_menu = menu.addMenu(tr("Morph into..."));
  morph_menu->setEnabled(!read_only_ && has_selection && !selected_node.expanded_inline);
  if (!read_only_ && has_selection) {
    const QStringList candidates = morphCandidates(selected_node);
    if (candidates.isEmpty()) {
      morph_menu->setEnabled(false);
    } else {
      for (const QString& candidate : candidates) {
        QAction* action = morph_menu->addAction(candidate);
        connect(action, &QAction::triggered, this, [this, candidate]() {
          morphSelected(candidate);
        });
      }
    }
  }

  QAction* delete_action = menu.addAction(tr("Remove"));
  delete_action->setEnabled(!read_only_ && has_selection && !selected_node.expanded_inline);

  QAction* smart_remove_action = menu.addAction(tr("Smart Remove"));
  bool smart_ok = false;
  if (!read_only_ && has_selection && uid != tree_.root_uid &&
      !selected_node.children.isEmpty()) {
    const int parent_uid = FindParentUid(tree_, uid);
    if (parent_uid >= 0 && tree_.nodes.contains(parent_uid)) {
      const BtAbsNode& parent = tree_.nodes.value(parent_uid);
      smart_ok = MaxChildCount(parent.kind) >=
                 parent.children.size() - 1 + selected_node.children.size();
    }
  }
  smart_remove_action->setEnabled(smart_ok && !selected_node.expanded_inline);

  QAction* create_subtree_action = menu.addAction(tr("Create SubTree here"));
  create_subtree_action->setEnabled(
      !read_only_ && has_selection && uid != tree_.root_uid &&
      KindAcceptsChildren(selected_node.kind));

  menu.addSeparator();
  QAction* hook_action = menu.addAction(tr("Configure Hook / Breakpoint..."));
  hook_action->setEnabled(has_selection && !selected_node.expanded_inline &&
                          selected_node.kind != BtNodeKind::kRoot);

  menu.addSeparator();
  QAction* layout_v_action = menu.addAction(tr("Layout Vertical"));
  layout_v_action->setEnabled(!read_only_);
  QAction* layout_h_action = menu.addAction(tr("Layout Horizontal"));
  layout_h_action->setEnabled(!read_only_);
  menu.addAction(tr("Fit"), this, &BtGraphView::fitToView);
  QAction* export_svg_action = menu.addAction(tr("Export SVG..."));

  QAction* chosen = menu.exec(event->globalPos());
  if (chosen == delete_action) {
    deleteSelected();
  } else if (chosen == smart_remove_action) {
    smartRemoveSelected();
  } else if (chosen == create_subtree_action) {
    emit createSubtreeRequested(uid);
  } else if (chosen == hook_action) {
    emit configureHookRequested(uid);
  } else if (chosen == layout_v_action) {
    applyAutoLayout(false);
    fitToView();
  } else if (chosen == layout_h_action) {
    applyAutoLayout(true);
    fitToView();
  } else if (chosen == export_svg_action) {
    const QString path = QFileDialog::getSaveFileName(
        this, tr("Export SVG"), tree_.tree_id + QStringLiteral(".svg"),
        tr("SVG Files (*.svg)"));
    if (!path.isEmpty()) {
      exportSvg(path);
    }
  }
}

void BtGraphView::dragEnterEvent(QDragEnterEvent* event) {
  if (read_only_) {
    event->ignore();
    return;
  }
  QString registration_id;
  BtNodeKind kind = BtNodeKind::kUndefined;
  if (ReadBtNodeDragPayload(event->mimeData(), &registration_id, &kind)) {
    event->acceptProposedAction();
    return;
  }
  event->ignore();
}

void BtGraphView::dragMoveEvent(QDragMoveEvent* event) {
  if (read_only_) {
    event->ignore();
    return;
  }
  QString registration_id;
  BtNodeKind kind = BtNodeKind::kUndefined;
  if (ReadBtNodeDragPayload(event->mimeData(), &registration_id, &kind)) {
    event->acceptProposedAction();
    return;
  }
  event->ignore();
}

void BtGraphView::dropEvent(QDropEvent* event) {
  if (read_only_) {
    event->ignore();
    return;
  }
  QString registration_id;
  BtNodeKind kind = BtNodeKind::kUndefined;
  if (!ReadBtNodeDragPayload(event->mimeData(), &registration_id, &kind)) {
    event->ignore();
    return;
  }

  const QPointF scene_pos = mapToScene(event->position().toPoint());
  if (tree_.root_uid < 0) {
    addChildNode(-1, registration_id, kind);
  } else {
    addChildNode(resolveDropParentUid(), registration_id, kind);
  }

  const int uid = selectedUid();
  for (QGraphicsItem* item : scene_->items()) {
    if (auto* node_item = dynamic_cast<BtNodeItem*>(item)) {
      if (node_item->uid() == uid) {
        node_item->setPos(scene_pos -
                          QPointF(node_item->visualWidth() * 0.5, node_item->visualHeight() * 0.5));
        syncPositionsFromScene();
        emit treeChanged();
        break;
      }
    }
  }

  event->acceptProposedAction();
}

void BtGraphView::rebuildScene() {
  scene_->blockSignals(true);
  // Delete edges first so node destructors never see dangling edge callbacks.
  const QList<QGraphicsItem*> existing = scene_->items();
  for (QGraphicsItem* item : existing) {
    if (dynamic_cast<BtEdgeItem*>(item) != nullptr) {
      scene_->removeItem(item);
      delete item;
    }
  }
  scene_->clear();

  QHash<int, BtNodeItem*> items;
  items.reserve(tree_.nodes.size());
  for (auto it = tree_.nodes.constBegin(); it != tree_.nodes.constEnd(); ++it) {
    const BtNodeModel* model = nullptr;
    auto model_it = models_.constFind(it.value().registration_id);
    if (model_it != models_.constEnd()) {
      model = &model_it.value();
    }
    auto* item =
        new BtNodeItem(it.key(), it.value(), model, read_only_, !read_only_, horizontal_layout_);
    item->setPos(it.value().pos);
    if (!read_only_) {
      item->setMoveStartedNotifier([this]() {
        if (move_snapshot_pending_) {
          pushSnapshot();
          move_snapshot_pending_ = false;
        }
      });
    }
    scene_->addItem(item);
    items.insert(it.key(), item);

    if (!highlighted_port_value_.isEmpty()) {
      bool match = false;
      for (auto remap = it.value().port_remap.constBegin();
           remap != it.value().port_remap.constEnd(); ++remap) {
        if (remap.value() == highlighted_port_value_) {
          match = true;
          break;
        }
      }
      item->setPortHighlighted(match);
    }
    item->setHasHook(hook_node_uids_.contains(it.key()));
  }

  for (auto it = tree_.nodes.constBegin(); it != tree_.nodes.constEnd(); ++it) {
    BtNodeItem* parent_item = items.value(it.key());
    if (parent_item == nullptr) {
      continue;
    }
    for (int child_uid : it.value().children) {
      BtNodeItem* child_item = items.value(child_uid);
      if (child_item == nullptr) {
        continue;
      }
      auto* edge = new BtEdgeItem(parent_item, child_item, child_uid, horizontal_layout_);
      if (tree_.nodes.contains(child_uid)) {
        const BtAbsNode& child = tree_.nodes.value(child_uid);
        edge->setChildStatus(child.status, child.prev_status);
      }
      scene_->addItem(edge);
    }
  }

  scene_->blockSignals(false);
}

void BtGraphView::syncPositionsFromScene() {
  for (QGraphicsItem* item : scene_->items()) {
    if (auto* node_item = dynamic_cast<BtNodeItem*>(item)) {
      if (tree_.nodes.contains(node_item->uid())) {
        tree_.nodes[node_item->uid()].pos = node_item->pos();
      }
    }
  }
}

void BtGraphView::pushSnapshot() {
  if (restoring_snapshot_) {
    return;
  }
  syncPositionsFromScene();
  if (undo_stack_.size() >= kMaxUndoSnapshots) {
    undo_stack_.removeFirst();
  }
  undo_stack_.push_back(tree_);
  redo_stack_.clear();
}

void BtGraphView::restoreSnapshot(const BtAbsTree& snapshot) {
  restoring_snapshot_ = true;
  tree_ = snapshot;
  rebuildScene();
  restoring_snapshot_ = false;
  emit selectionChanged(selectedUid());
}

void BtGraphView::emitSelectionFromScene() {
  emit selectionChanged(selectedUid());
}

void BtGraphView::removeSubtree(int uid) {
  if (!tree_.nodes.contains(uid)) {
    return;
  }

  const QVector<int> to_remove = CollectSubtree(tree_, uid);
  const QSet<int> remove_set(to_remove.begin(), to_remove.end());

  if (remove_set.contains(tree_.root_uid)) {
    tree_.root_uid = -1;
  }

  for (auto it = tree_.nodes.begin(); it != tree_.nodes.end(); ++it) {
    QVector<int>& children = it.value().children;
    children.erase(std::remove_if(children.begin(), children.end(),
                                  [&remove_set](int child_uid) {
                                    return remove_set.contains(child_uid);
                                  }),
                   children.end());
  }

  for (int remove_uid : to_remove) {
    tree_.nodes.remove(remove_uid);
  }
}

void BtGraphView::removeNodeKeepChildren(int uid) {
  if (!tree_.nodes.contains(uid) || uid == tree_.root_uid) {
    return;
  }

  const int parent_uid = FindParentUid(tree_, uid);
  if (parent_uid < 0 || !tree_.nodes.contains(parent_uid)) {
    return;
  }

  BtAbsNode& parent = tree_.nodes[parent_uid];
  const BtAbsNode node = tree_.nodes.value(uid);
  const int index = parent.children.indexOf(uid);
  if (index < 0) {
    return;
  }

  parent.children.removeAt(index);
  for (int i = 0; i < node.children.size(); ++i) {
    parent.children.insert(index + i, node.children.at(i));
  }
  tree_.nodes.remove(uid);
}

int BtGraphView::resolveDropParentUid() const {
  const int selected = selectedUid();
  if (selected >= 0) {
    return selected;
  }
  if (tree_.root_uid < 0) {
    return -1;
  }
  return tree_.root_uid;
}

QStringList BtGraphView::morphCandidates(const BtAbsNode& node) const {
  QStringList candidates;
  for (auto it = models_.constBegin(); it != models_.constEnd(); ++it) {
    if (it.value().kind == node.kind && it.key() != node.registration_id) {
      candidates.push_back(it.key());
    }
  }
  candidates.sort(Qt::CaseInsensitive);
  return candidates;
}

}  // namespace behavior_tree
}  // namespace autoviz
