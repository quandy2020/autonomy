/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/tf_tree_graph_view.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <unordered_map>

#include <QBrush>
#include <QFont>
#include <QFontMetricsF>
#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QPalette>
#include <QPen>
#include <QPolygonF>
#include <QScrollBar>
#include <QMouseEvent>
#include <QResizeEvent>
#include <QShowEvent>
#include <QStringList>
#include <QWheelEvent>

namespace autoviz {
namespace {

constexpr int kFrameIdRole = 0;

constexpr double kMinZoom = 0.05;
constexpr double kMaxZoom = 4.0;
constexpr double kMinNodeWidth = 46.0;
constexpr double kMinNodeHeight = 28.0;
constexpr double kNodePaddingX = 5.0;
constexpr double kNodePaddingY = 3.0;
constexpr double kSiblingGap = 16.0;
constexpr double kRootGap = 48.0;
/** Horizontal gap between an edge and the label sitting beside it. */
constexpr double kLabelOffsetX = 20.0;
/** Cap on how far a label follows a leaning approach, and the room every label
 *  column reserves for that shift so a sibling subtree cannot be crowded. */
constexpr double kMaxLabelShiftX = 20.0;
constexpr double kLabelClearanceY = 30.0;
constexpr double kFanOutClearanceY = 46.0;
constexpr double kLabelLineSpacing = 1.15;
/** How far the approach leans back towards the parent, per unit of sideways
 *  reach, and the cap on that lean. Together they keep a distant child's edge
 *  curving into the node instead of arriving dead vertical, while leaving the
 *  label beside it untouched. */
constexpr double kApproachLeanRatio = 0.05;
constexpr double kMaxApproachLean = 8.0;
/** Vertical run the lean is measured over. */
constexpr double kApproachRun = 46.0;
/** Corridor above the label row where edges do all of their sideways travel. */
constexpr double kBandTopGap = 58.0;
constexpr double kBandBottomGap = 10.0;
constexpr double kArrowLength = 9.0;
constexpr double kArrowHalfWidth = 3.5;
constexpr double kScenePadding = 22.0;
constexpr double kLegendGap = 22.0;
constexpr double kLegendPaddingX = 10.0;
constexpr double kLegendPaddingY = 6.0;
constexpr double kStrokeWidth = 1.4;
/** Beyond this the clock feeding the panel is not on the transforms' timeline
 *  (wall clock against simulated time), so the age annotation is meaningless
 *  and only makes every label wider. `_allFramesAsDot` omits it too when it has
 *  no usable clock. */
constexpr double kMaxPlausibleAgeSeconds = 86400.0;
constexpr double kStaleAgeSeconds = 1.0;

/** Light chrome shared with TfTreePanel — no dark canvas. */
struct GraphPalette {
  QColor canvas;
  QColor node_fill;
  QColor node_fill_bottom;
  QColor node_static_fill;
  QColor node_static_fill_bottom;
  QColor node_current_fill;
  QColor edge;
  QColor edge_stale;
  QColor ink;
  QColor secondary_text;
  QColor accent;
  QColor accent_soft;
  QColor static_accent;
  QColor placeholder_fill;
  QColor label_bg;
  QColor legend_fill;
  QColor legend_border;
};

GraphPalette MakeGraphPalette(const QPalette& palette) {
  Q_UNUSED(palette);
  GraphPalette graph;
  graph.canvas = QColor(248, 249, 251);
  graph.node_fill = QColor(255, 255, 255);
  graph.node_fill_bottom = QColor(241, 245, 249);
  graph.node_static_fill = QColor(255, 251, 245);
  graph.node_static_fill_bottom = QColor(255, 243, 224);
  graph.node_current_fill = QColor(224, 247, 250);
  graph.edge = QColor(100, 116, 139);
  graph.edge_stale = QColor(148, 163, 184);
  graph.ink = QColor(30, 41, 59);
  graph.secondary_text = QColor(71, 85, 105);
  graph.accent = QColor(8, 145, 178);
  graph.accent_soft = QColor(8, 145, 178, 45);
  graph.static_accent = QColor(217, 119, 6);
  graph.placeholder_fill = QColor(255, 255, 255);
  graph.label_bg = QColor(255, 255, 255, 245);
  graph.legend_fill = QColor(255, 255, 255);
  graph.legend_border = QColor(203, 213, 225);
  return graph;
}

QFont GraphFont(int point_size) {
  QFont font;
  font.setStyleHint(QFont::SansSerif);
  font.setPointSize(point_size);
  return font;
}

/** Frame names carry the structure, so they stay at reading size. */
QFont NodeFont() {
  QFont font = GraphFont(10);
  font.setBold(true);
  return font;
}

/** Every column reserves room for its label, so the five metric lines drive the
 *  whole graph width; a smaller size here is what keeps the tree compact. */
QFont LabelFont() { return GraphFont(8); }

/** `line_spacing` <= 0 falls back to the font's own leading. */
QSizeF MeasureBlock(const QFontMetricsF& metrics, const QStringList& lines,
                    double line_spacing = -1.0) {
  double width = 0.0;
  for (const QString& line : lines) {
    width = std::max(width, metrics.horizontalAdvance(line));
  }
  const double spacing =
      line_spacing > 0.0 ? line_spacing : metrics.lineSpacing();
  return QSizeF(width, spacing * lines.size());
}

/** graphviz centres every line of a label within the label box. */
void AddCenteredBlock(QGraphicsScene* scene, const QStringList& lines,
                      const QFont& font, const QColor& color,
                      const QPointF& top_left, double block_width,
                      double line_spacing, double z_value) {
  double y = top_left.y();
  for (const QString& line : lines) {
    auto* item = scene->addSimpleText(line, font);
    item->setBrush(color);
    const QRectF bounds = item->boundingRect();
    // Split the extra leading above and below, so the ink stays centred in the
    // height MeasureBlock reserved for the block.
    item->setPos(top_left.x() + (block_width - bounds.width()) * 0.5,
                 y + (line_spacing - bounds.height()) * 0.5);
    item->setZValue(z_value);
    y += line_spacing;
  }
}

QString FormatFixed(double value, int precision = 3) {
  if (!std::isfinite(value)) {
    return QStringLiteral("unknown");
  }
  return QString::number(value, 'f', precision);
}

QString NormalizeParent(const QString& parent) {
  if (parent.isEmpty() || parent == QLatin1String("NO_PARENT")) {
    return {};
  }
  return parent;
}

/** Same fields, order, and units as tf2's `_allFramesAsDot` edge label. */
QStringList BuildEdgeLabel(const transform::TfFrameStats& stats,
                           double current_time_seconds) {
  const double latest = static_cast<double>(stats.last_stamp_ns) / 1e9;
  const double oldest = static_cast<double>(stats.oldest_stamp_ns) / 1e9;
  QString recent = FormatFixed(latest);
  const double age = current_time_seconds - latest;
  if (current_time_seconds > 0.0 && age >= 0.0 &&
      age < kMaxPlausibleAgeSeconds) {
    recent += QStringLiteral(" ( %1 sec old)").arg(FormatFixed(age));
  }
  QString authority = QString::fromStdString(stats.authority);
  if (authority.isEmpty()) {
    authority = QStringLiteral("no recorded authority");
  }
  QStringList lines;
  lines << QStringLiteral("Broadcaster: %1").arg(authority)
        << QStringLiteral("Average rate: %1")
               .arg(FormatFixed(stats.average_rate_hertz))
        << QStringLiteral("Buffer length: %1")
               .arg(FormatFixed(stats.buffer_length_seconds))
        << QStringLiteral("Most recent transform: %1").arg(recent)
        << QStringLiteral("Oldest transform: %1").arg(FormatFixed(oldest));
  return lines;
}

struct LayoutNode {
  QString frame_id;
  QString parent_id;
  QStringList edge_label;
  std::vector<int> children;
  QSizeF ellipse;
  QSizeF label;
  double center_x = 0.0;
  double center_y = 0.0;
  double age_seconds = -1.0;
  double average_rate_hertz = 0.0;
  int depth = 0;
  int parent_index = -1;
  /** Position among the parent's children, left to right, and their count. */
  int sibling_index = 0;
  int sibling_count = 1;
  bool is_static = false;
  bool placed = false;

  /** Space owned left of the centre; the label always sits on the right. */
  double leftExtent() const { return ellipse.width() * 0.5; }
  double rightExtent() const {
    return std::max(ellipse.width() * 0.5,
                    label.isEmpty() ? 0.0
                                    : kMaxLabelShiftX + kLabelOffsetX +
                                          label.width());
  }
  bool isStale() const {
    return age_seconds >= 0.0 && age_seconds > kStaleAgeSeconds;
  }
};

/** Top of the label block for the edge that ends at `child`, centred in the gap
 *  between the two outlines so the text sits the same distance from the parent
 *  and from the child. Rows reserve `kLabelClearanceY` on both sides, so the
 *  clamp only matters for a label taller than its own row. */
double EdgeLabelTop(const LayoutNode& parent, const LayoutNode& child) {
  const double parent_bottom = parent.center_y + parent.ellipse.height() * 0.5;
  const double child_top = child.center_y - child.ellipse.height() * 0.5;
  const double centered =
      (parent_bottom + child_top - child.label.height()) * 0.5;
  const double highest = parent_bottom + kLabelClearanceY;
  const double lowest = child_top - kLabelClearanceY - child.label.height();
  return highest < lowest ? std::clamp(centered, highest, lowest) : centered;
}

/** Ellipse that circumscribes a text box, the way dot sizes its default node. */
QSizeF EllipseForText(const QSizeF& text) {
  constexpr double kRootTwo = 1.41421356237;
  return QSizeF(
      std::max(kMinNodeWidth, text.width() * kRootTwo + kNodePaddingX * 2.0),
      std::max(kMinNodeHeight, text.height() * kRootTwo + kNodePaddingY * 2.0));
}

/** Point where the ray leaving `center` towards `direction` meets the outline. */
QPointF EllipseBoundary(const QPointF& center, const QSizeF& ellipse,
                        double angle) {
  return QPointF(center.x() + ellipse.width() * 0.5 * std::cos(angle),
                 center.y() + ellipse.height() * 0.5 * std::sin(angle));
}

/** Point on the parent outline where the edge to `child` leaves it.
 *  Departures fan around the underside the way graphviz spaces them: aiming
 *  every edge straight at its child would pinch them all into one point and
 *  smear the distant ones along the outline, so each edge also gets an angular
 *  slot from its position among the siblings, and the steeper of the two wins.
 */
QPointF EllipseExit(const LayoutNode& parent, const LayoutNode& child,
                    const QPointF& target) {
  /** Widest departure from straight down, so no edge skims the outline. */
  constexpr double kExitArcLimit = 1.05;  // ~60 degrees off vertical.
  const double semi_x = std::max(1.0, parent.ellipse.width() * 0.5);
  const double semi_y = std::max(1.0, parent.ellipse.height() * 0.5);
  const double aimed = std::atan2((target.y() - parent.center_y) / semi_y,
                                  (target.x() - parent.center_x) / semi_x);
  double angle = aimed;
  if (child.sibling_count > 1) {
    const double fraction = static_cast<double>(child.sibling_index) /
                            (child.sibling_count - 1);
    const double slot = M_PI_2 + kExitArcLimit - fraction * 2.0 * kExitArcLimit;
    if (std::abs(slot - M_PI_2) < std::abs(aimed - M_PI_2)) {
      angle = slot;
    }
  }
  return EllipseBoundary(QPointF(parent.center_x, parent.center_y),
                         parent.ellipse, angle);
}

/** Last stretch of an edge: where it lands on the child, and the lean it
 *  arrives with. Shared by the edge and the label pass so the text can keep a
 *  constant gap from the line rather than from the child's column. */
struct EdgeApproach {
  QPointF heading;
  QPointF tip;
  QPointF end;

  /** x of the approach line at `y`, extended above the arrow head. */
  double xAt(double y) const {
    return end.x() - heading.x() / heading.y() * (end.y() - y);
  }
};

EdgeApproach MakeEdgeApproach(const LayoutNode& parent,
                              const LayoutNode& child) {
  // Leaning in from the parent's side keeps the edge turning all the way down,
  // so it lands on the shoulder the way a graphviz spline does.
  const double reach = child.center_x - parent.center_x;
  const double lean = std::clamp(-reach * kApproachLeanRatio,
                                 -kMaxApproachLean, kMaxApproachLean);
  const double angle = M_PI_2 + std::atan2(lean, kApproachRun);
  EdgeApproach approach;
  approach.heading = QPointF(std::cos(angle), std::sin(angle));
  approach.tip = EllipseBoundary(QPointF(child.center_x, child.center_y),
                                 child.ellipse, angle - M_PI);
  approach.end =
      QPointF(approach.tip.x() - approach.heading.x() * kArrowLength,
              approach.tip.y() - approach.heading.y() * kArrowLength);
  return approach;
}

/** Left edge of the label block: a fixed gap from the approach line wherever
 *  that line runs past the text, so every label is spaced the same way from the
 *  edge it annotates instead of drifting closer as the lean grows. */
double EdgeLabelLeft(const EdgeApproach& approach, const LayoutNode& child,
                     double label_top) {
  // x is monotonic in y along the approach, so the two ends bound its travel.
  const double line_x =
      std::max(approach.xAt(label_top),
               approach.xAt(label_top + child.label.height()));
  const double shift =
      std::clamp(line_x - child.center_x, 0.0, kMaxLabelShiftX);
  return child.center_x + shift + kLabelOffsetX;
}

/** Arrow head pointing along the unit vector `heading`. */
void AppendArrowHead(QPainterPath* path, const QPointF& tip,
                     const QPointF& heading) {
  const QPointF base(tip.x() - heading.x() * kArrowLength,
                     tip.y() - heading.y() * kArrowLength);
  const QPointF flank(-heading.y() * kArrowHalfWidth,
                      heading.x() * kArrowHalfWidth);
  QPolygonF head;
  head << tip << base + flank << base - flank;
  path->addPolygon(head);
  path->closeSubpath();
}

}  // namespace

TfTreeGraphView::TfTreeGraphView(QWidget* parent) : QGraphicsView(parent) {
  scene_ = new QGraphicsScene(this);
  setScene(scene_);
  setRenderHint(QPainter::Antialiasing, true);
  setRenderHint(QPainter::TextAntialiasing, true);
  setDragMode(QGraphicsView::ScrollHandDrag);
  setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
  setResizeAnchor(QGraphicsView::AnchorViewCenter);
  setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
  setBackgroundBrush(MakeGraphPalette(palette()).canvas);
  setFrameShape(QFrame::NoFrame);
}

void TfTreeGraphView::requestFit() { fit_pending_ = true; }

void TfTreeGraphView::setCurrentFrame(const QString& frame_id) {
  if (current_frame_id_ == frame_id) {
    return;
  }
  current_frame_id_ = frame_id;
  if (!last_frames_.empty()) {
    setFrames(last_frames_, last_current_time_seconds_, last_filter_);
  }
}

void TfTreeGraphView::showMessage(const QString& message) {
  scene_->clear();
  const GraphPalette graph_palette = MakeGraphPalette(palette());
  setBackgroundBrush(graph_palette.canvas);
  auto* text = scene_->addSimpleText(message, NodeFont());
  text->setBrush(graph_palette.secondary_text);
  const QRectF bounds = text->boundingRect();
  const QRectF box(bounds.adjusted(-22, -14, 22, 14));
  QPainterPath message_path;
  message_path.addRoundedRect(box, 10.0, 10.0);
  auto* rect = scene_->addPath(message_path, QPen(graph_palette.legend_border, 1.0),
                               QBrush(graph_palette.placeholder_fill));
  rect->setZValue(-1.0);
  text->setPos(-bounds.width() * 0.5, -bounds.height() * 0.5);
  text->setZValue(1.0);
  scene_->setSceneRect(box.adjusted(-220, -140, 220, 140));
  resetTransform();
  zoom_factor_ = 1.0;
  fit_pending_ = false;
  fitted_ = false;
  centerOn(rect);
  emit graphRendered(0, 0);
}

void TfTreeGraphView::setFrames(
    const std::vector<transform::TfFrameStats>& frames,
    double current_time_seconds, const QString& filter) {
  last_frames_ = frames;
  last_current_time_seconds_ = current_time_seconds;
  last_filter_ = filter;
  const QFont node_font = NodeFont();
  const QFont label_font = LabelFont();
  const QFontMetricsF node_metrics(node_font);
  const QFontMetricsF label_metrics(label_font);
  const double label_line_spacing =
      label_metrics.lineSpacing() * kLabelLineSpacing;

  std::vector<LayoutNode> nodes;
  nodes.reserve(frames.size());
  std::unordered_map<std::string, int> index_by_frame;

  for (const transform::TfFrameStats& stats : frames) {
    const QString frame_id = QString::fromStdString(stats.frame_id);
    if (frame_id.isEmpty()) {
      continue;
    }
    const QString parent_id =
        NormalizeParent(QString::fromStdString(stats.parent_id));
    if (!filter.isEmpty() &&
        !frame_id.contains(filter, Qt::CaseInsensitive) &&
        !parent_id.contains(filter, Qt::CaseInsensitive)) {
      continue;
    }
    LayoutNode node;
    node.frame_id = frame_id;
    node.parent_id = parent_id;
    node.is_static = stats.is_static;
    node.average_rate_hertz = stats.average_rate_hertz;
    node.ellipse = EllipseForText(MeasureBlock(node_metrics, {frame_id}));
    if (!parent_id.isEmpty()) {
      node.edge_label = BuildEdgeLabel(stats, current_time_seconds);
      node.label =
          MeasureBlock(label_metrics, node.edge_label, label_line_spacing);
      const double latest = static_cast<double>(stats.last_stamp_ns) / 1e9;
      if (current_time_seconds > 0.0 && latest > 0.0) {
        const double age = current_time_seconds - latest;
        if (age >= 0.0 && age < kMaxPlausibleAgeSeconds) {
          node.age_seconds = age;
        }
      }
    }
    index_by_frame.emplace(stats.frame_id, static_cast<int>(nodes.size()));
    nodes.push_back(std::move(node));
  }

  // Stats only exist for frames that received a transform, so the frame at the
  // top of the tree is never reported. `view_frames` still draws it, and
  // without it every child would be treated as a root and lose its edge label.
  const int measured_count = static_cast<int>(nodes.size());
  for (int index = 0; index < measured_count; ++index) {
    const std::string parent_id = nodes[index].parent_id.toStdString();
    if (parent_id.empty() || index_by_frame.count(parent_id) != 0) {
      continue;
    }
    LayoutNode parent;
    parent.frame_id = nodes[index].parent_id;
    parent.ellipse =
        EllipseForText(MeasureBlock(node_metrics, {parent.frame_id}));
    index_by_frame.emplace(parent_id, static_cast<int>(nodes.size()));
    nodes.push_back(std::move(parent));
  }

  if (nodes.empty()) {
    showMessage(filter.isEmpty() ? tr("No transforms received yet.")
                                 : tr("No frames match the filter."));
    return;
  }

  const int node_count = static_cast<int>(nodes.size());
  std::vector<int> roots;
  for (int index = 0; index < node_count; ++index) {
    const auto parent = index_by_frame.find(nodes[index].parent_id.toStdString());
    if (parent != index_by_frame.end() && parent->second != index) {
      nodes[index].parent_index = parent->second;
      nodes[parent->second].children.push_back(index);
    } else {
      // Root, or a frame whose parent was filtered out: draw no incoming edge.
      nodes[index].label = QSizeF();
      nodes[index].edge_label.clear();
      roots.push_back(index);
    }
  }

  for (int index = 0; index < node_count; ++index) {
    std::vector<int>& children = nodes[index].children;
    std::sort(children.begin(), children.end(), [&nodes](int lhs, int rhs) {
      return nodes[lhs].frame_id < nodes[rhs].frame_id;
    });
    // Placement follows this order, so it doubles as the left-to-right order.
    for (int slot = 0; slot < static_cast<int>(children.size()); ++slot) {
      nodes[children[slot]].sibling_index = slot;
      nodes[children[slot]].sibling_count = static_cast<int>(children.size());
    }
  }
  std::sort(roots.begin(), roots.end(), [&nodes](int lhs, int rhs) {
    return nodes[lhs].frame_id < nodes[rhs].frame_id;
  });

  // Depth first, so a malformed cycle cannot reach unbounded recursion.
  std::function<void(int, int)> assign_depth = [&](int index, int depth) {
    nodes[index].depth = depth;
    for (int child : nodes[index].children) {
      if (nodes[child].depth == 0 && child != index) {
        assign_depth(child, depth + 1);
      }
    }
  };
  for (int root : roots) {
    assign_depth(root, 0);
  }

  int max_depth = 0;
  for (const LayoutNode& node : nodes) {
    max_depth = std::max(max_depth, node.depth);
  }
  std::vector<double> row_node_height(max_depth + 1, 0.0);
  std::vector<double> row_label_height(max_depth + 1, 0.0);
  for (const LayoutNode& node : nodes) {
    row_node_height[node.depth] =
        std::max(row_node_height[node.depth], node.ellipse.height());
    row_label_height[node.depth] =
        std::max(row_label_height[node.depth], node.label.height());
  }
  std::vector<double> row_clearance(max_depth + 1, kLabelClearanceY);
  for (const LayoutNode& node : nodes) {
    if (node.children.size() > 1 && node.depth + 1 <= max_depth) {
      row_clearance[node.depth + 1] = kFanOutClearanceY;
    }
  }
  std::vector<double> row_top(max_depth + 1, 0.0);
  for (int depth = 1; depth <= max_depth; ++depth) {
    row_top[depth] = row_top[depth - 1] + row_node_height[depth - 1] +
                     row_clearance[depth] * 2.0 + row_label_height[depth];
  }
  for (LayoutNode& node : nodes) {
    node.center_y = row_top[node.depth] + row_node_height[node.depth] * 0.5;
  }

  // Tidy top-down placement: children first, then centre the parent over them.
  std::function<double(int, double)> place = [&](int index,
                                                double left) -> double {
    LayoutNode& node = nodes[index];
    if (node.placed) {
      return left;
    }
    node.placed = true;
    const double own_width = node.leftExtent() + node.rightExtent();
    if (node.children.empty()) {
      node.center_x = left + node.leftExtent();
      return left + own_width;
    }
    double cursor = left;
    double first_center = left;
    double last_center = left;
    bool has_child = false;
    for (int child : node.children) {
      if (nodes[child].placed) {
        continue;
      }
      const double right = place(child, cursor);
      if (!has_child) {
        first_center = nodes[child].center_x;
        has_child = true;
      }
      last_center = nodes[child].center_x;
      cursor = right + kSiblingGap;
    }
    if (has_child) {
      cursor -= kSiblingGap;
    }
    const double right_edge = std::max(cursor, left + own_width);
    const double children_center =
        has_child ? (first_center + last_center) * 0.5
                  : left + node.leftExtent();
    node.center_x =
        std::clamp(children_center, left + node.leftExtent(),
                   std::max(left + node.leftExtent(),
                            right_edge - node.rightExtent()));
    return right_edge;
  };

  double cursor = 0.0;
  for (int root : roots) {
    cursor = place(root, cursor) + kRootGap;
  }
  // Frames stranded by a parent cycle still deserve a column.
  for (int index = 0; index < node_count; ++index) {
    if (!nodes[index].placed) {
      cursor = place(index, cursor) + kRootGap;
    }
  }

  const int horizontal_scroll = horizontalScrollBar()->value();
  const int vertical_scroll = verticalScrollBar()->value();

  const GraphPalette graph_palette = MakeGraphPalette(palette());
  setBackgroundBrush(graph_palette.canvas);
  scene_->clear();

  for (const LayoutNode& node : nodes) {
    if (node.parent_index < 0) {
      continue;
    }
    const LayoutNode& parent = nodes[node.parent_index];
    const EdgeApproach approach_line = MakeEdgeApproach(parent, node);
    const QPointF heading = approach_line.heading;
    const QPointF tip = approach_line.tip;
    const QPointF end = approach_line.end;
    const double corridor_y =
        node.label.isEmpty()
            ? (parent.center_y + parent.ellipse.height() * 0.5 + end.y()) * 0.5
            : EdgeLabelTop(parent, node) - kArrowLength;
    const double approach = std::max(1.0, end.y() - corridor_y);
    const QPointF corridor_exit(end.x() - heading.x() / heading.y() * approach,
                                corridor_y);
    const QPointF start = EllipseExit(parent, node, corridor_exit);
    const double drop = std::max(1.0, corridor_y - start.y());
    const double spread = corridor_exit.x() - start.x();

    QPainterPath edge_path;
    edge_path.moveTo(start);
    edge_path.cubicTo(
        QPointF(start.x() + spread * 0.40, start.y() + drop * 0.30),
        QPointF(corridor_exit.x() - heading.x() * drop * 0.70,
                corridor_y - heading.y() * drop * 0.70),
        corridor_exit);
    edge_path.lineTo(end);

    QPainterPath arrow_path;
    AppendArrowHead(&arrow_path, tip, heading);

    const QColor edge_color =
        node.isStale() ? graph_palette.edge_stale : graph_palette.edge;
    const QPen stroke(edge_color, node.isStale() ? 1.0 : kStrokeWidth,
                      node.isStale() ? Qt::DashLine : Qt::SolidLine, Qt::RoundCap,
                      Qt::RoundJoin);
    auto* edge_item = scene_->addPath(edge_path, stroke, QBrush(Qt::NoBrush));
    edge_item->setZValue(-2.0);
    auto* arrow_item =
        scene_->addPath(arrow_path, QPen(Qt::NoPen), QBrush(edge_color));
    arrow_item->setZValue(-2.0);
  }

  for (const LayoutNode& node : nodes) {
    if (node.parent_index < 0 || node.edge_label.isEmpty()) {
      continue;
    }
    const LayoutNode& parent = nodes[node.parent_index];
    const double label_top = EdgeLabelTop(parent, node);
    const double label_left =
        EdgeLabelLeft(MakeEdgeApproach(parent, node), node, label_top);
    const QRectF label_box(
        label_left - 6.0, label_top - 4.0, node.label.width() + 12.0,
        node.label.height() + 8.0);
    QPainterPath label_path;
    label_path.addRoundedRect(label_box, 6.0, 6.0);
    auto* label_bg = scene_->addPath(label_path, QPen(Qt::NoPen),
                                     QBrush(graph_palette.label_bg));
    label_bg->setZValue(-1.5);
    AddCenteredBlock(scene_, node.edge_label, label_font,
                     node.isStale() ? graph_palette.edge_stale
                                    : graph_palette.secondary_text,
                     QPointF(label_left, label_top), node.label.width(),
                     label_line_spacing, -1.0);
  }

  for (const LayoutNode& node : nodes) {
    const QRectF bounds(node.center_x - node.ellipse.width() * 0.5,
                        node.center_y - node.ellipse.height() * 0.5,
                        node.ellipse.width(), node.ellipse.height());
    const bool current = node.frame_id == current_frame_id_;
    QColor fill_top = node.is_static ? graph_palette.node_static_fill
                                     : graph_palette.node_fill;
    QColor fill_bottom = node.is_static ? graph_palette.node_static_fill_bottom
                                        : graph_palette.node_fill_bottom;
    if (current) {
      fill_top = graph_palette.node_current_fill;
      fill_bottom = graph_palette.node_fill_bottom;
    }
    QLinearGradient gradient(bounds.topLeft(), bounds.bottomLeft());
    gradient.setColorAt(0.0, fill_top);
    gradient.setColorAt(1.0, fill_bottom);

    QPen node_pen(graph_palette.edge, 1.2);
    if (node.is_static) {
      node_pen.setColor(graph_palette.static_accent);
      node_pen.setStyle(Qt::DashLine);
    }
    if (current) {
      node_pen.setColor(graph_palette.accent);
      node_pen.setWidthF(2.4);
      node_pen.setStyle(Qt::SolidLine);
      auto* glow = scene_->addEllipse(
          bounds.adjusted(-4, -4, 4, 4), QPen(Qt::NoPen),
          QBrush(graph_palette.accent_soft));
      glow->setZValue(0.5);
    }

    auto* ellipse =
        scene_->addEllipse(bounds, node_pen, QBrush(gradient));
    ellipse->setZValue(1.0);
    ellipse->setData(kFrameIdRole, node.frame_id);
    QString tip = node.frame_id;
    if (node.is_static) {
      tip += tr(" (static)");
    }
    if (node.average_rate_hertz > 0.0) {
      tip += tr("\n%1 Hz").arg(FormatFixed(node.average_rate_hertz, 2));
    }
    if (node.age_seconds >= 0.0) {
      tip += tr("\n%1 s old").arg(FormatFixed(node.age_seconds, 2));
    }
    ellipse->setToolTip(tip);

    auto* text = scene_->addSimpleText(node.frame_id, node_font);
    text->setBrush(graph_palette.ink);
    const QRectF text_bounds = text->boundingRect();
    text->setPos(bounds.center().x() - text_bounds.width() * 0.5,
                 bounds.center().y() - text_bounds.height() * 0.5);
    text->setZValue(2.0);
    text->setData(kFrameIdRole, node.frame_id);
  }

  // Caption card above the tree.
  const QRectF graph_bounds = scene_->itemsBoundingRect();
  QStringList caption;
  caption << tr("Transform Tree");
  QStringList detail;
  detail << tr("%1 frames · %2 roots")
                .arg(node_count)
                .arg(static_cast<int>(roots.size()));
  if (current_time_seconds > 0.0) {
    detail << tr("Recorded at %1").arg(FormatFixed(current_time_seconds, 3));
  }
  const QSizeF caption_size = MeasureBlock(node_metrics, caption);
  const QSizeF detail_size =
      MeasureBlock(label_metrics, detail, label_line_spacing);
  const double legend_width =
      std::max(caption_size.width(), detail_size.width()) +
      kLegendPaddingX * 2.0 + 8.0;
  const double legend_height = caption_size.height() + detail_size.height() +
                               kLegendPaddingY * 4.0;
  const QRectF legend_rect(graph_bounds.center().x() - legend_width * 0.5,
                           graph_bounds.top() - kLegendGap - legend_height,
                           legend_width, legend_height);
  QPainterPath legend_path;
  legend_path.addRoundedRect(legend_rect, 10.0, 10.0);
  auto* legend_box = scene_->addPath(
      legend_path, QPen(graph_palette.legend_border, 1.0),
      QBrush(graph_palette.legend_fill));
  legend_box->setZValue(1.0);
  // Accent bar on the left of the legend.
  QPainterPath accent_path;
  accent_path.addRoundedRect(
      QRectF(legend_rect.left() + 3.0, legend_rect.top() + 8.0, 3.0,
             legend_rect.height() - 16.0),
      1.5, 1.5);
  auto* accent_bar =
      scene_->addPath(accent_path, QPen(Qt::NoPen), QBrush(graph_palette.accent));
  accent_bar->setZValue(2.0);
  AddCenteredBlock(scene_, caption, node_font, graph_palette.ink,
                   QPointF(legend_rect.left() + kLegendPaddingX + 4.0,
                           legend_rect.top() + kLegendPaddingY),
                   legend_width - kLegendPaddingX * 2.0 - 4.0,
                   node_metrics.lineSpacing(), 2.0);
  AddCenteredBlock(scene_, detail, label_font, graph_palette.secondary_text,
                   QPointF(legend_rect.left() + kLegendPaddingX + 4.0,
                           legend_rect.top() + kLegendPaddingY * 2.0 +
                               caption_size.height()),
                   legend_width - kLegendPaddingX * 2.0 - 4.0,
                   label_line_spacing, 2.0);

  scene_->setSceneRect(scene_->itemsBoundingRect().adjusted(
      -kScenePadding, -kScenePadding, kScenePadding, kScenePadding));

  if (fit_pending_) {
    fit_pending_ = false;
    zoomToFit();
  } else {
    horizontalScrollBar()->setValue(horizontal_scroll);
    verticalScrollBar()->setValue(vertical_scroll);
  }

  emit graphRendered(node_count, static_cast<int>(roots.size()));
}

void TfTreeGraphView::mousePressEvent(QMouseEvent* event) {
  if (event != nullptr) {
    if (QGraphicsItem* item = itemAt(event->position().toPoint())) {
      const QVariant frame_id = item->data(kFrameIdRole);
      if (frame_id.isValid()) {
        const QString frame = frame_id.toString();
        if (!frame.isEmpty()) {
          current_frame_id_ = frame;
          emit frameActivated(frame);
          setFrames(last_frames_, last_current_time_seconds_, last_filter_);
        }
      }
    }
  }
  QGraphicsView::mousePressEvent(event);
}

void TfTreeGraphView::zoomToFit() {
  const QRectF bounds = scene_->itemsBoundingRect().adjusted(
      -kScenePadding, -kScenePadding, kScenePadding, kScenePadding);
  if (bounds.isEmpty()) {
    return;
  }
  fitInView(bounds, Qt::KeepAspectRatio);
  zoom_factor_ = transform().m11();
  fitted_ = true;
}

void TfTreeGraphView::wheelEvent(QWheelEvent* event) {
  const double factor = event->angleDelta().y() > 0 ? 1.12 : 1.0 / 1.12;
  const double next_zoom = std::clamp(zoom_factor_ * factor, kMinZoom, kMaxZoom);
  const double applied = next_zoom / std::max(zoom_factor_, 1e-9);
  zoom_factor_ = next_zoom;
  scale(applied, applied);
  fitted_ = false;
  event->accept();
}

void TfTreeGraphView::showEvent(QShowEvent* event) {
  QGraphicsView::showEvent(event);
  if (fit_pending_) {
    fit_pending_ = false;
    zoomToFit();
  }
}

void TfTreeGraphView::resizeEvent(QResizeEvent* event) {
  QGraphicsView::resizeEvent(event);
  // Undocking or expanding the panel changes the viewport by a lot; keep the
  // whole tree in view unless the user has chosen their own zoom.
  if (fitted_) {
    zoomToFit();
  }
}

}  // namespace autoviz
