/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_host.hpp"

#include <algorithm>
#include <functional>
#include <limits>

#include <QAbstractAnimation>
#include <QColor>
#include <QCoreApplication>
#include <QFont>
#include <QLabel>
#include <QDockWidget>
#include <QEasingCurve>
#include <QEnterEvent>
#include <QEvent>
#include <QLinearGradient>
#include <QPalette>
#include <QPainterPath>
#include <QRegion>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QPaintEvent>
#include <QPointer>
#include <QPropertyAnimation>
#include <QResizeEvent>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSplitter>
#include <QSplitterHandle>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <glog/logging.h>

#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/theme/tokens.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

/** Hit target width — the seam is a canvas gutter, not a drawn border. */
int SplitterHitWidth() { return theme::Shell().gutter; }

/** One-pixel hairline used by card frames. */
constexpr qreal kCardHairline = 1.0;
constexpr int kMinPaneSpan = 80;

void ApplyExpandingPolicy(QWidget* widget) {
  if (widget == nullptr) {
    return;
  }
  const QSizePolicy policy = widget->sizePolicy();
  if (policy.horizontalPolicy() != QSizePolicy::Expanding ||
      policy.verticalPolicy() != QSizePolicy::Expanding) {
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }
  if (widget->maximumWidth() < QWIDGETSIZE_MAX ||
      widget->maximumHeight() < QWIDGETSIZE_MAX) {
    widget->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
  }
}

/**
 * @brief Make @p splitter children fill its current geometry.
 *
 * Preserves relative size ratios when possible. Without stretch factors, a
 * deferred @c setSizes based on a stale/narrow host width leaves large empty
 * gaps between 3D View / Map (and other) panes.
 *
 * No-ops when the splitter is already filled — repeated setSizes on resize
 * thrash Ogre/GL viewports (constant flicker, no interaction).
 */
void FillSplitter(QSplitter* splitter) {
  if (splitter == nullptr || splitter->count() <= 0) {
    return;
  }
  const int count = splitter->count();
  for (int i = 0; i < count; ++i) {
    QWidget* child = splitter->widget(i);
    ApplyExpandingPolicy(child);
    splitter->setStretchFactor(i, 1);
    if (auto* nested = qobject_cast<QSplitter*>(child)) {
      FillSplitter(nested);
    }
  }

  const int handle_space = splitter->handleWidth() * qMax(0, count - 1);
  const int available =
      (splitter->orientation() == Qt::Horizontal ? splitter->width()
                                                 : splitter->height()) -
      handle_space;
  if (available < count) {
    return;
  }

  QList<int> current = splitter->sizes();
  qint64 sum = 0;
  for (int size : current) {
    sum += qMax(size, 0);
  }
  // Already filling the host — avoid setSizes (triggers GL viewport resize).
  if (current.size() == count && sum > 0 &&
      qAbs(static_cast<int>(sum) - available) <= 2) {
    return;
  }

  QList<int> sizes;
  sizes.reserve(count);
  if (current.size() != count || sum <= 0) {
    const int each = qMax(available / count, kMinPaneSpan);
    for (int i = 0; i < count; ++i) {
      sizes.push_back(each);
    }
  } else {
    int allocated = 0;
    for (int i = 0; i < count; ++i) {
      int next = 0;
      if (i + 1 == count) {
        next = qMax(available - allocated, kMinPaneSpan);
      } else {
        next = static_cast<int>((static_cast<qint64>(qMax(current.at(i), 1)) *
                                 available) /
                                sum);
        next = qMax(next, kMinPaneSpan);
      }
      sizes.push_back(next);
      allocated += next;
    }
  }
  splitter->setSizes(sizes);
}

void EqualizeSplitter(QSplitter* splitter) {
  if (splitter == nullptr || splitter->count() <= 0) {
    return;
  }
  const int count = splitter->count();
  for (int i = 0; i < count; ++i) {
    ApplyExpandingPolicy(splitter->widget(i));
    splitter->setStretchFactor(i, 1);
  }
  const int handle_space = splitter->handleWidth() * qMax(0, count - 1);
  const int available =
      (splitter->orientation() == Qt::Horizontal ? qMax(splitter->width(), 1)
                                                 : qMax(splitter->height(), 1)) -
      handle_space;
  if (available >= count) {
    const int each = qMax(available / count, kMinPaneSpan);
    QList<int> sizes;
    sizes.reserve(count);
    for (int i = 0; i < count; ++i) {
      sizes.push_back(each);
    }
    splitter->setSizes(sizes);
  }
  // Equalize nested after parent span is committed so children see real sizes.
  for (int i = 0; i < count; ++i) {
    if (auto* nested = qobject_cast<QSplitter*>(splitter->widget(i))) {
      EqualizeSplitter(nested);
    }
  }
}

class PanelSplitterHandle : public QSplitterHandle {
 public:
  explicit PanelSplitterHandle(Qt::Orientation orientation, QSplitter* parent)
      : QSplitterHandle(orientation, parent) {}

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);
    const theme::ShellTokens shell = theme::Shell();
    // Canvas gutter: panes read as separate cards with a ~7px gap between
    // them. Painting panel_surface here fused the mosaic into one slab.
    painter.fillRect(rect(), shell.window);

    // Idle: no seam (the gutter itself separates cards). Hover: accent drag
    // affordance so the splitter stays discoverable.
    if (!underMouse()) {
      return;
    }
    painter.setPen(QPen(shell.accent, 1));
    if (orientation() == Qt::Horizontal) {
      const int x = width() / 2;
      painter.drawLine(x, 0, x, height());
    } else {
      const int y = height() / 2;
      painter.drawLine(0, y, width(), y);
    }
  }

  void enterEvent(QEnterEvent* event) override {
    QSplitterHandle::enterEvent(event);
    update();
  }

  void leaveEvent(QEvent* event) override {
    QSplitterHandle::leaveEvent(event);
    update();
  }
};

class PanelSplitter : public QSplitter {
 public:
  explicit PanelSplitter(Qt::Orientation orientation, QWidget* parent = nullptr)
      : QSplitter(orientation, parent) {
    setChildrenCollapsible(false);
    setHandleWidth(SplitterHitWidth());
    setOpaqueResize(true);
    // Clear theme QSS borders so only the 1px painted seam shows.
    setStyleSheet(style::sheet(QStringLiteral("chrome/layout")));
  }

 protected:
  QSplitterHandle* createHandle() override {
    return new PanelSplitterHandle(orientation(), this);
  }
};

void RaiseSplitterHandles(QSplitter* splitter) {
  if (splitter == nullptr) {
    return;
  }
  splitter->setHandleWidth(SplitterHitWidth());
  for (int i = 1; i < splitter->count(); ++i) {
    if (QSplitterHandle* handle = splitter->handle(i)) {
      handle->setAttribute(Qt::WA_TransparentForMouseEvents, false);
      handle->raise();
      handle->show();
      handle->update();
    }
  }
  for (int i = 0; i < splitter->count(); ++i) {
    if (auto* nested = qobject_cast<QSplitter*>(splitter->widget(i))) {
      RaiseSplitterHandles(nested);
    }
  }
}

/**
 * @brief Mosaic pane chrome — card fill plus a 1px border.
 *
 * The border needs a 1px layout inset: the dock itself paints an opaque
 * surface and would otherwise cover the frame.
 *
 * Rounded corners are opt-in: a 3D View hosts a native GLX child, which is
 * never clipped by a child widget mask. Those panes stay square; every other
 * center panel gets the shell card radius applied through a widget mask so the
 * dock content is clipped alongside the painted frame.
 */
class MainPanelCard : public QWidget {
 public:
  explicit MainPanelCard(bool rounded, QWidget* parent = nullptr)
      : QWidget(parent), rounded_(rounded) {
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setAttribute(Qt::WA_StyledBackground, false);
    setAutoFillBackground(false);
    if (rounded_) {
      // Corners are painted with the canvas colour; clipping the pane itself
      // would punch a hole through the window and reveal the desktop / the
      // backing store (the dark triangle seen at card corners).
      setAttribute(Qt::WA_NoSystemBackground, false);
    }
  }

  ~MainPanelCard() override {
    if (QWidget* child = clipped_child_.data()) {
      if (child->parentWidget() == this) {
        child->clearMask();
      }
    }
  }

  qreal cardRadius() const {
    return rounded_ ? static_cast<qreal>(theme::Shell().panel_radius) : 0.0;
  }

  /**
   * @brief Clips @p child to the rounded card outline.
   *
   * Masking the child (not this pane) keeps the pane's corner pixels painted,
   * so the cut corners show the canvas colour instead of an unpainted hole.
   */
  void setClippedChild(QWidget* child) {
    clipped_child_ = child;
    if (child != nullptr) {
      child->installEventFilter(this);
      updateChildMask();
    }
  }

 protected:
  bool eventFilter(QObject* watched, QEvent* event) override {
    if (watched == clipped_child_.data() && event != nullptr &&
        (event->type() == QEvent::Resize ||
         event->type() == QEvent::ParentChange ||
         event->type() == QEvent::Show)) {
      updateChildMask();
    }
    return QWidget::eventFilter(watched, event);
  }

  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    const theme::ShellTokens shell = theme::Shell();
    if (!rounded_) {
      painter.fillRect(rect(), shell.panel_surface);
      painter.setPen(QPen(shell.panel_border, kCardHairline));
      painter.drawRect(rect().adjusted(0, 0, -1, -1));
      return;
    }
    // Canvas base: the child mask cuts the corners down to this colour.
    painter.fillRect(rect(), shell.window);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const qreal inset = kCardHairline / 2.0;
    const QRectF frame =
        QRectF(rect()).adjusted(inset, inset, -inset, -inset);
    painter.setPen(QPen(shell.panel_border, kCardHairline));
    painter.setBrush(shell.panel_surface);
    painter.drawRoundedRect(frame, cardRadius(), cardRadius());
  }

  void resizeEvent(QResizeEvent* event) override {
    QWidget::resizeEvent(event);
    updateChildMask();
  }

 private:
  /** Clip the dock content to the painted card outline. */
  void updateChildMask() {
    QWidget* child = clipped_child_.data();
    if (child == nullptr) {
      return;
    }
    if (!rounded_ || child->parentWidget() != this || child->width() <= 0 ||
        child->height() <= 0) {
      child->clearMask();
      return;
    }
    QPainterPath path;
    const qreal radius = cardRadius();
    path.addRoundedRect(QRectF(child->rect()), radius, radius);
    child->setMask(QRegion(path.toFillPolygon().toPolygon()));
  }

  bool rounded_ = false;
  QPointer<QWidget> clipped_child_;
};

QWidget* WrapDockInPane(QDockWidget* dock) {
  if (dock == nullptr) {
    return nullptr;
  }
  // Always build a fresh pane. Never reuse — clearSplitterTree may have already
  // destroyed the previous wrapper.
  if (QWidget* parent = dock->parentWidget()) {
    if (parent->property("mainPanelPane").toBool()) {
      if (QLayout* layout = parent->layout()) {
        layout->removeWidget(dock);
      }
      // Do not setParent(nullptr): QDockWidget becomes a top-level window.
    }
  }
  // Native GLX viewports cannot be clipped by a widget mask, so only docks
  // without a 3D View get the rounded card treatment.
  const bool is_viewport =
      dock->property("panelTypeId").toString() == QLatin1String("ViewportDock") ||
      dock->objectName().startsWith(QLatin1String("ViewportDock"));
  auto* pane = new MainPanelCard(!is_viewport);
  pane->setProperty("mainPanelPane", true);
  // Wide enough for dock title tools (settings/split/change/expand/close).
  pane->setMinimumSize(220, 80);
  ApplyExpandingPolicy(pane);
  pane->setMinimumSize(220, 80);
  // Opaque fill — Clear/transparent panes show MainPanelHost black through
  // gaps as 黑边 after splitter 吸附.
  pane->setAttribute(Qt::WA_OpaquePaintEvent, true);
  pane->setAttribute(Qt::WA_StyledBackground, true);
  pane->setAutoFillBackground(true);
  {
    const theme::ShellTokens shell = theme::Shell();
    QPalette pal = pane->palette();
    pal.setColor(QPalette::Window, shell.panel_surface);
    pal.setColor(QPalette::Base, shell.panel_surface);
    pane->setPalette(pal);
  }
  pane->setStyleSheet(style::mark(style::Mark::Clear));
  auto* layout = new QVBoxLayout(pane);
  // 1px inset keeps the card border visible under the opaque dock.
  layout->setContentsMargins(1, 1, 1, 1);
  layout->setSpacing(0);
  ApplyExpandingPolicy(dock);
  if (QWidget* content = dock->widget()) {
    ApplyExpandingPolicy(content);
  }
  layout->addWidget(dock, /*stretch=*/1);
  if (auto* card = dynamic_cast<MainPanelCard*>(pane)) {
    card->setClippedChild(dock);
  }
  return pane;
}

QDockWidget* DockFromPane(QWidget* widget) {
  if (widget == nullptr) {
    return nullptr;
  }
  if (auto* dock = qobject_cast<QDockWidget*>(widget)) {
    return dock;
  }
  if (widget->property("mainPanelPane").toBool()) {
    return widget->findChild<QDockWidget*>(QString(), Qt::FindDirectChildrenOnly);
  }
  return nullptr;
}

void CollectDocks(QWidget* root, QList<QDockWidget*>* out) {
  if (root == nullptr || out == nullptr) {
    return;
  }
  if (auto* dock = qobject_cast<QDockWidget*>(root)) {
    out->push_back(dock);
    return;
  }
  if (root->property("mainPanelPane").toBool()) {
    if (QDockWidget* dock = DockFromPane(root)) {
      out->push_back(dock);
    }
    return;
  }
  if (auto* splitter = qobject_cast<QSplitter*>(root)) {
    for (int i = 0; i < splitter->count(); ++i) {
      CollectDocks(splitter->widget(i), out);
    }
  }
}

QWidget* FindPaneForDock(QSplitter* root, QDockWidget* dock) {
  if (root == nullptr || dock == nullptr) {
    return nullptr;
  }
  if (QWidget* parent = dock->parentWidget()) {
    if (parent->property("mainPanelPane").toBool()) {
      // Pane must live under the splitter tree (not a floating/orphan parent).
      for (QWidget* walk = parent->parentWidget(); walk != nullptr;
           walk = walk->parentWidget()) {
        if (walk == root) {
          return parent;
        }
      }
    }
  }
  return nullptr;
}

/**
 * Foxglove / react-mosaic drop: whole pane is a target. Nearest edge in
 * normalized [0,1] space — raw |dx| vs |dy| locked tall remaining leaves into
 * Top/Bottom whenever the cursor stayed near the title bar (左右 never won).
 */
bool EdgeForLocalPoint(const QRect& pane_rect, const QPoint& local,
                       Qt::Edge* edge_out) {
  if (edge_out == nullptr || !pane_rect.contains(local)) {
    return false;
  }
  const double w = static_cast<double>(qMax(1, pane_rect.width()));
  const double h = static_cast<double>(qMax(1, pane_rect.height()));
  const double x_rate =
      static_cast<double>(local.x() - pane_rect.left()) / w;
  const double y_rate =
      static_cast<double>(local.y() - pane_rect.top()) / h;
  const double dist_l = x_rate;
  const double dist_r = 1.0 - x_rate;
  const double dist_t = y_rate * 0.92;
  const double dist_b = (1.0 - y_rate) * 0.92;
  if (dist_b <= dist_t && dist_b <= dist_l && dist_b <= dist_r) {
    *edge_out = Qt::BottomEdge;
  } else if (dist_t <= dist_l && dist_t <= dist_r) {
    *edge_out = Qt::TopEdge;
  } else if (dist_l <= dist_r) {
    *edge_out = Qt::LeftEdge;
  } else {
    *edge_out = Qt::RightEdge;
  }
  return true;
}

/** Half-pane highlight matching Foxglove mosaic drop preview. */
QRect HighlightRectForEdge(const QRect& pane_rect, Qt::Edge edge) {
  const int hw = qMax(1, pane_rect.width() / 2);
  const int hh = qMax(1, pane_rect.height() / 2);
  switch (edge) {
    case Qt::LeftEdge:
      return QRect(pane_rect.left(), pane_rect.top(), hw, pane_rect.height());
    case Qt::RightEdge:
      return QRect(pane_rect.right() - hw + 1, pane_rect.top(), hw,
                   pane_rect.height());
    case Qt::TopEdge:
      return QRect(pane_rect.left(), pane_rect.top(), pane_rect.width(), hh);
    case Qt::BottomEdge:
      return QRect(pane_rect.left(), pane_rect.bottom() - hh + 1,
                   pane_rect.width(), hh);
    default:
      return QRect();
  }
}

bool FindDropAt(QSplitter* root, QWidget* host, const QPoint& global_pos,
                const QDockWidget* exclude, QDockWidget** target_out,
                Qt::Edge* edge_out, QRect* highlight_in_host) {
  if (root == nullptr || target_out == nullptr || edge_out == nullptr) {
    return false;
  }
  QList<QDockWidget*> docks;
  CollectDocks(root, &docks);
  // Prefer the smallest pane under the cursor (deepest mosaic leaf).
  QDockWidget* best_dock = nullptr;
  QWidget* best_pane = nullptr;
  int best_area = std::numeric_limits<int>::max();
  QPoint best_local;
  for (QDockWidget* dock : docks) {
    if (dock == nullptr || dock == exclude) {
      continue;
    }
    QWidget* pane = FindPaneForDock(root, dock);
    if (pane == nullptr || !pane->isVisible()) {
      continue;
    }
    const QPoint local = pane->mapFromGlobal(global_pos);
    if (!pane->rect().contains(local)) {
      continue;
    }
    const int area = qMax(1, pane->width()) * qMax(1, pane->height());
    if (area < best_area) {
      best_area = area;
      best_dock = dock;
      best_pane = pane;
      best_local = local;
    }
  }
  // Layout churn / native-hole miss: fall back to the sole (or largest) leaf
  // and map the cursor into that pane so 左右吸附 still resolves.
  if (best_dock == nullptr || best_pane == nullptr) {
    int largest = -1;
    for (QDockWidget* dock : docks) {
      if (dock == nullptr || dock == exclude) {
        continue;
      }
      QWidget* pane = FindPaneForDock(root, dock);
      if (pane == nullptr || !pane->isVisible()) {
        continue;
      }
      const int area = qMax(1, pane->width()) * qMax(1, pane->height());
      if (area > largest) {
        largest = area;
        best_dock = dock;
        best_pane = pane;
      }
    }
    if (best_dock == nullptr || best_pane == nullptr) {
      return false;
    }
    best_local = best_pane->mapFromGlobal(global_pos);
    if (!best_pane->rect().contains(best_local)) {
      // Clamp into pane so EdgeForLocalPoint can still pick a half.
      best_local.setX(qBound(0, best_local.x(), best_pane->width() - 1));
      best_local.setY(qBound(0, best_local.y(), best_pane->height() - 1));
    }
  }
  Qt::Edge edge = Qt::LeftEdge;
  if (!EdgeForLocalPoint(best_pane->rect(), best_local, &edge)) {
    return false;
  }
  *target_out = best_dock;
  *edge_out = edge;
  if (highlight_in_host != nullptr && host != nullptr) {
    const QRect pane_hl = HighlightRectForEdge(best_pane->rect(), edge);
    *highlight_in_host =
        QRect(best_pane->mapTo(host, pane_hl.topLeft()), pane_hl.size());
  }
  return true;
}

/** Foxglove/react-mosaic remove: collapse a splitter that has a single child. */
void CollapseDegenerateSplitter(QSplitter* splitter, QSplitter* root) {
  while (splitter != nullptr && splitter != root && splitter->count() == 1) {
    QPointer<QSplitter> alive(splitter);
    QWidget* only = splitter->widget(0);
    auto* parent = qobject_cast<QSplitter*>(splitter->parentWidget());
    if (parent == nullptr || only == nullptr) {
      break;
    }
    const int index = parent->indexOf(splitter);
    if (index < 0) {
      break;
    }
    QList<int> sizes = parent->sizes();
    // replaceWidget hides/shows and can emit QDockWidget::visibilityChanged.
    // Callers must hold mutating_mosaic_ so handlers do not re-enter scrub.
    parent->replaceWidget(index, only);
    if (!alive) {
      splitter = parent;
      continue;
    }
    splitter->setParent(nullptr);
    splitter->deleteLater();
    if (sizes.size() == parent->count()) {
      parent->setSizes(sizes);
    }
    splitter = parent;
  }
}

/** Drop panes with no dock (left behind when setFloating reparents out). */
void ScrubEmptyPanesRecursive(QSplitter* splitter, QSplitter* root) {
  if (splitter == nullptr) {
    return;
  }
  // Snapshot children first — CollapseDegenerateSplitter / delete mutates the
  // splitter mid-iteration and left dangling QWidget* for qobject_cast (SIGSEGV).
  QList<QPointer<QWidget>> children;
  children.reserve(splitter->count());
  for (int i = 0; i < splitter->count(); ++i) {
    children.push_back(splitter->widget(i));
  }

  QList<QPointer<QSplitter>> nested_to_collapse;
  for (const QPointer<QWidget>& child_guard : children) {
    QWidget* child = child_guard.data();
    if (child == nullptr) {
      continue;
    }
    if (auto* nested = qobject_cast<QSplitter*>(child)) {
      ScrubEmptyPanesRecursive(nested, root);
      if (!child_guard) {
        continue;
      }
      if (nested->count() == 0) {
        nested->setParent(nullptr);
        delete nested;
      } else {
        nested_to_collapse.push_back(nested);
      }
      continue;
    }
    if (!child->property("mainPanelPane").toBool()) {
      // Orphan filler widgets (not pane / splitter / dock) become the black
      // band above a left/right snap — delete them.
      if (qobject_cast<QDockWidget*>(child) == nullptr) {
        child->setParent(nullptr);
        delete child;
      }
      continue;
    }
    QDockWidget* dock = DockFromPane(child);
    // Empty leaf, or dock that floated away / is no longer a child of this pane.
    if (dock == nullptr || dock->parentWidget() != child) {
      child->setParent(nullptr);
      delete child;
    }
  }
  for (const QPointer<QSplitter>& nested : nested_to_collapse) {
    if (nested != nullptr) {
      CollapseDegenerateSplitter(nested.data(), root);
    }
  }
  CollapseDegenerateSplitter(splitter, root);
}

void PrepareDockForSplitter(QDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  dock->setAllowedAreas(Qt::NoDockWidgetArea);
  dock->setFeatures(QDockWidget::DockWidgetClosable);
  ApplyExpandingPolicy(dock);
  if (QWidget* content = dock->widget()) {
    ApplyExpandingPolicy(content);
  }
  // Avoid re-entrant visibilityChanged when already docked in the splitter.
  if (dock->isFloating()) {
    const QSignalBlocker blocker(dock);
    dock->setFloating(false);
  }
}

/**
 * Collapse the mosaic to a single root leaf for @p dock.
 * Prevents Left/Right 吸附 from nesting under a Vertical remnant (black band
 * on top, grey|purple strip on the bottom).
 *
 * @p park must park any other docks found under the cleared tree.
 */
void FlattenToSoleRootLeaf(
    QSplitter* root, QDockWidget* dock,
    const std::function<void(QDockWidget*)>& park) {
  if (root == nullptr || dock == nullptr) {
    return;
  }
  PrepareDockForSplitter(dock);
  if (QWidget* parent = dock->parentWidget()) {
    if (parent->property("mainPanelPane").toBool()) {
      if (QLayout* layout = parent->layout()) {
        layout->removeWidget(dock);
      }
    }
  }

  QList<QWidget*> orphans;
  while (root->count() > 0) {
    QWidget* child = root->widget(0);
    child->setParent(nullptr);
    orphans.push_back(child);
  }
  for (QWidget* orphan : orphans) {
    if (orphan == nullptr) {
      continue;
    }
    const QList<QDockWidget*> nested_docks =
        orphan->findChildren<QDockWidget*>();
    for (QDockWidget* nested : nested_docks) {
      if (nested == nullptr || nested == dock) {
        continue;
      }
      if (park) {
        park(nested);
      } else {
        nested->hide();
      }
    }
    if (QWidget* p = dock->parentWidget()) {
      if (p == orphan || orphan->isAncestorOf(dock)) {
        if (QLayout* layout = p->layout()) {
          layout->removeWidget(dock);
        }
      }
    }
    delete orphan;
  }

  root->setOrientation(Qt::Horizontal);
  root->addWidget(WrapDockInPane(dock));
  dock->show();
  dock->raise();
}

constexpr char kMosaicPrefix[] = "mosaic:v1:";

QJsonObject SerializeMosaicNode(QWidget* widget) {
  QJsonObject node;
  if (widget == nullptr) {
    return node;
  }
  if (QDockWidget* dock = DockFromPane(widget)) {
    node.insert(QStringLiteral("type"), QStringLiteral("leaf"));
    QString name = dock->objectName();
    if (dock->property("mosaicPlaceholder").toBool()) {
      // Persist the original leaf so a later session can retry the real panel
      // once the plugin/type is available again.
      const QString missing = dock->property("mosaicMissingName").toString();
      if (!missing.isEmpty()) {
        name = missing;
      }
    }
    node.insert(QStringLiteral("name"), name);
    return node;
  }
  if (auto* splitter = qobject_cast<QSplitter*>(widget)) {
    node.insert(QStringLiteral("type"), QStringLiteral("split"));
    node.insert(QStringLiteral("orientation"),
                splitter->orientation() == Qt::Horizontal
                    ? QStringLiteral("horizontal")
                    : QStringLiteral("vertical"));
    QJsonArray sizes;
    for (int size : splitter->sizes()) {
      sizes.push_back(size);
    }
    node.insert(QStringLiteral("sizes"), sizes);
    QJsonArray children;
    for (int i = 0; i < splitter->count(); ++i) {
      QJsonObject child = SerializeMosaicNode(splitter->widget(i));
      if (!child.isEmpty()) {
        children.push_back(child);
      }
    }
    node.insert(QStringLiteral("children"), children);
    return node;
  }
  return node;
}

/**
 * @brief Placeholder pane for a mosaic leaf whose panel type is unavailable.
 *
 * Keeps the saved split geometry and tells the user which type is missing;
 * @ref SerializeMosaicNode writes the original leaf name back so a future
 * session can restore the real panel.
 */
QDockWidget* CreateMosaicPlaceholder(const QString& missing_name) {
  const QString title = QCoreApplication::translate(
      "autoviz", "Missing: %1").arg(missing_name);
  auto* dock = new PanelDockWidget(title);
  dock->setObjectName(QStringLiteral("MosaicPlaceholder_") + missing_name);
  dock->setProperty("mosaicPlaceholder", true);
  dock->setProperty("mosaicMissingName", missing_name);

  auto* body = new QWidget(dock);
  auto* layout = new QVBoxLayout(body);
  layout->setContentsMargins(16, 16, 16, 16);
  layout->setSpacing(6);
  const theme::ShellTokens shell = theme::Shell();
  auto* heading = new QLabel(
      QCoreApplication::translate("autoviz", "Panel unavailable"), body);
  heading->setAlignment(Qt::AlignHCenter);
  QFont heading_font = heading->font();
  heading_font.setWeight(QFont::DemiBold);
  heading->setFont(heading_font);
  heading->setStyleSheet(
      QStringLiteral("color: %1;").arg(theme::Hex(shell.text)));
  auto* detail = new QLabel(missing_name, body);
  detail->setAlignment(Qt::AlignHCenter);
  detail->setWordWrap(true);
  detail->setStyleSheet(
      QStringLiteral("color: %1;").arg(theme::Hex(shell.secondary_text)));
  layout->addStretch(1);
  layout->addWidget(heading);
  layout->addWidget(detail);
  layout->addStretch(1);
  body->setStyleSheet(
      QStringLiteral("background: %1;").arg(theme::Hex(shell.panel_surface)));
  dock->setWidget(body);
  return dock;
}

QWidget* BuildMosaicNode(
    const QJsonObject& node,
    const std::function<QDockWidget*(const QString&)>& resolve,
    QList<QDockWidget*>* placed) {
  if (node.isEmpty() || placed == nullptr) {
    return nullptr;
  }
  const QString type = node.value(QStringLiteral("type")).toString();
  if (type == QLatin1String("leaf")) {
    const QString name = node.value(QStringLiteral("name")).toString();
    QDockWidget* dock = resolve ? resolve(name) : nullptr;
    if (dock == nullptr) {
      dock = CreateMosaicPlaceholder(name);
    }
    PrepareDockForSplitter(dock);
    placed->push_back(dock);
    return WrapDockInPane(dock);
  }
  if (type != QLatin1String("split")) {
    return nullptr;
  }
  const Qt::Orientation orientation =
      node.value(QStringLiteral("orientation")).toString() ==
              QLatin1String("vertical")
          ? Qt::Vertical
          : Qt::Horizontal;
  auto* splitter = new PanelSplitter(orientation);
  const QJsonArray children = node.value(QStringLiteral("children")).toArray();
  for (const QJsonValue& child_value : children) {
    if (!child_value.isObject()) {
      continue;
    }
    if (QWidget* child =
            BuildMosaicNode(child_value.toObject(), resolve, placed)) {
      splitter->addWidget(child);
    }
  }
  if (splitter->count() == 0) {
    splitter->deleteLater();
    return nullptr;
  }
  if (splitter->count() == 1) {
    // Degenerate split → promote the only child.
    QWidget* only = splitter->widget(0);
    only->setParent(nullptr);
    splitter->deleteLater();
    return only;
  }
  QList<int> sizes;
  const QJsonArray sizes_json = node.value(QStringLiteral("sizes")).toArray();
  for (const QJsonValue& size_value : sizes_json) {
    sizes.push_back(size_value.toInt(80));
  }
  if (sizes.size() == splitter->count()) {
    splitter->setSizes(sizes);
  }
  RaiseSplitterHandles(splitter);
  return splitter;
}

}  // namespace

class MainPanelHost::DropOverlay : public QWidget {
 public:
  /**
   * Opaque top-level plate sized to the drop half only.
   * Geometry morphs with OutCubic so edge changes feel Foxglove-smooth;
   * windowOpacity fades in/out without a translucent X11 hole.
   */
  explicit DropOverlay(QWidget* host)
      : QWidget(/*parent=*/nullptr), host_(host) {
    setObjectName(QStringLiteral("MainPanelDropOverlay"));
    setWindowFlags(Qt::Tool | Qt::FramelessWindowHint |
                   Qt::WindowStaysOnTopHint | Qt::WindowDoesNotAcceptFocus);
    setAttribute(Qt::WA_ShowWithoutActivating, true);
    setAttribute(Qt::WA_TransparentForMouseEvents, true);
    setAttribute(Qt::WA_TranslucentBackground, false);
    setAttribute(Qt::WA_NoSystemBackground, false);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setAutoFillBackground(false);
    hide();

    geo_anim_ = new QPropertyAnimation(this, "geometry", this);
    geo_anim_->setDuration(150);
    geo_anim_->setEasingCurve(QEasingCurve::OutCubic);

    opacity_anim_ = new QPropertyAnimation(this, "windowOpacity", this);
    opacity_anim_->setDuration(120);
    opacity_anim_->setEasingCurve(QEasingCurve::OutCubic);
  }

  void beginSession(const QRect& /*host_local*/, bool /*empty_mosaic*/) {
    session_active_ = true;
    edge_ = Qt::LeftEdge;
    stopAnims();
    hide();
    setWindowOpacity(1.0);
  }

  void endSession() {
    session_active_ = false;
    if (geo_anim_ != nullptr) {
      geo_anim_->stop();
    }
    if (!isVisible()) {
      hide();
      return;
    }
    fadeOutAndHide();
  }

  void clearPreview() { endSession(); }

  void setHalfPanePreview(const QRect& /*host_local*/,
                          const QRect& /*pane_in_host*/,
                          const QRect& dest_in_host, Qt::Edge edge) {
    session_active_ = true;
    if (host_ == nullptr || !dest_in_host.isValid() || dest_in_host.isEmpty()) {
      fadeOutAndHide();
      return;
    }
    const QRect local = dest_in_host.adjusted(3, 3, -3, -3);
    if (!local.isValid() || local.isEmpty()) {
      fadeOutAndHide();
      return;
    }
    const QPoint origin = host_->mapToGlobal(local.topLeft());
    const QRect target(origin, local.size());
    const bool edge_changed = edge_ != edge;
    edge_ = edge;

    // Cancel a pending hide fade if the cursor re-entered a valid half.
    if (opacity_anim_ != nullptr &&
        opacity_anim_->state() == QAbstractAnimation::Running &&
        opacity_anim_->endValue().toReal() < 0.5) {
      opacity_anim_->stop();
    }

    if (!isVisible()) {
      // Appear from a slightly inset rect so the first flash eases in.
      const QRect sprout = QRect(target.center(), QSize(1, 1))
                               .adjusted(-12, -12, 12, 12)
                               .intersected(target);
      setGeometry(sprout.isValid() ? sprout : target);
      setWindowOpacity(0.0);
      show();
      raise();
      animateGeometry(target);
      animateOpacity(0.0, 0.94);
      update();
      return;
    }

    raise();
    const QRect current =
        (geo_anim_ != nullptr &&
         geo_anim_->state() == QAbstractAnimation::Running)
            ? geo_anim_->endValue().toRect()
            : geometry();
    if (RectsNearlyEqual(current, target) && !edge_changed) {
      return;
    }
    if (RectsNearlyEqual(current, target)) {
      update();
      return;
    }
    animateGeometry(target);
    if (windowOpacity() < 0.9) {
      animateOpacity(windowOpacity(), 0.94);
    }
    update();
  }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const QRect plate = rect().adjusted(1, 1, -1, -1);

    // Soft outer ring then solid fill — reads smoother while morphing.
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(0x7C, 0x4D, 0xFF, 90));
    painter.drawRoundedRect(plate.adjusted(-2, -2, 2, 2), 10, 10);

    painter.setBrush(QColor(0x7C, 0x4D, 0xFF));
    painter.drawRoundedRect(plate, 8, 8);

    // Inner sheen.
    QLinearGradient sheen(plate.topLeft(), plate.bottomLeft());
    sheen.setColorAt(0.0, QColor(255, 255, 255, 40));
    sheen.setColorAt(1.0, QColor(255, 255, 255, 0));
    painter.setBrush(sheen);
    painter.drawRoundedRect(plate.adjusted(2, 2, -2, -2), 6, 6);

    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(QColor(255, 255, 255, 210), 2));
    painter.drawRoundedRect(plate, 8, 8);

    painter.setBrush(QColor(255, 255, 255, 235));
    painter.setPen(Qt::NoPen);
    const QPoint c = plate.center();
    const int s = qBound(14, qMin(plate.width(), plate.height()) / 7, 26);
    QPolygon tri;
    switch (edge_) {
      case Qt::TopEdge:
        tri << QPoint(c.x(), c.y() - s) << QPoint(c.x() - s, c.y() + s / 2)
            << QPoint(c.x() + s, c.y() + s / 2);
        break;
      case Qt::BottomEdge:
        tri << QPoint(c.x(), c.y() + s) << QPoint(c.x() - s, c.y() - s / 2)
            << QPoint(c.x() + s, c.y() - s / 2);
        break;
      case Qt::LeftEdge:
        tri << QPoint(c.x() - s, c.y()) << QPoint(c.x() + s / 2, c.y() - s)
            << QPoint(c.x() + s / 2, c.y() + s);
        break;
      case Qt::RightEdge:
      default:
        tri << QPoint(c.x() + s, c.y()) << QPoint(c.x() - s / 2, c.y() - s)
            << QPoint(c.x() - s / 2, c.y() + s);
        break;
    }
    painter.drawPolygon(tri);
  }

 private:
  static bool RectsNearlyEqual(const QRect& a, const QRect& b) {
    return qAbs(a.x() - b.x()) <= 2 && qAbs(a.y() - b.y()) <= 2 &&
           qAbs(a.width() - b.width()) <= 2 &&
           qAbs(a.height() - b.height()) <= 2;
  }

  void stopAnims() {
    if (geo_anim_ != nullptr) {
      geo_anim_->stop();
    }
    if (opacity_anim_ != nullptr) {
      opacity_anim_->stop();
      QObject::disconnect(opacity_anim_, &QPropertyAnimation::finished, this,
                          nullptr);
    }
  }

  void animateGeometry(const QRect& target) {
    if (geo_anim_ == nullptr) {
      setGeometry(target);
      return;
    }
    if (geo_anim_->state() == QAbstractAnimation::Running &&
        RectsNearlyEqual(geo_anim_->endValue().toRect(), target)) {
      return;
    }
    geo_anim_->stop();
    geo_anim_->setStartValue(geometry());
    geo_anim_->setEndValue(target);
    geo_anim_->setDuration(edgeTravelMs(geometry(), target));
    geo_anim_->start();
  }

  static int edgeTravelMs(const QRect& from, const QRect& to) {
    // Short hop for tiny nudges; longer morph when flipping L↔R / U↔D.
    const int dx = qAbs(from.center().x() - to.center().x());
    const int dy = qAbs(from.center().y() - to.center().y());
    const int dist = qMax(dx, dy);
    return qBound(90, 90 + dist / 8, 180);
  }

  void animateOpacity(qreal from, qreal to) {
    if (opacity_anim_ == nullptr) {
      setWindowOpacity(to);
      return;
    }
    QObject::disconnect(opacity_anim_, &QPropertyAnimation::finished, this,
                        nullptr);
    opacity_anim_->stop();
    opacity_anim_->setDuration(120);
    opacity_anim_->setStartValue(from);
    opacity_anim_->setEndValue(to);
    opacity_anim_->start();
  }

  void fadeOutAndHide() {
    if (!isVisible()) {
      return;
    }
    if (geo_anim_ != nullptr) {
      geo_anim_->stop();
    }
    if (opacity_anim_ == nullptr) {
      hide();
      setWindowOpacity(1.0);
      return;
    }
    QObject::disconnect(opacity_anim_, &QPropertyAnimation::finished, this,
                        nullptr);
    opacity_anim_->stop();
    opacity_anim_->setDuration(100);
    opacity_anim_->setStartValue(windowOpacity());
    opacity_anim_->setEndValue(0.0);
    QObject::connect(opacity_anim_, &QPropertyAnimation::finished, this,
                     [this]() {
                       hide();
                       setWindowOpacity(1.0);
                     });
    opacity_anim_->start();
  }

  QWidget* host_ = nullptr;
  Qt::Edge edge_ = Qt::LeftEdge;
  bool session_active_ = false;
  QPropertyAnimation* geo_anim_ = nullptr;
  QPropertyAnimation* opacity_anim_ = nullptr;
};

MainPanelHost::MainPanelHost(QWidget* parent) : QMainWindow(parent) {
  setObjectName(QStringLiteral("MainPanelHost"));
  setWindowFlags(Qt::Widget);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMinimumSize(0, 0);
  setDockOptions(QMainWindow::AllowNestedDocks | QMainWindow::AllowTabbedDocks);
  setDockNestingEnabled(false);
  setContentsMargins(0, 0, 0, 0);
  setAttribute(Qt::WA_OpaquePaintEvent, true);
  setAttribute(Qt::WA_TranslucentBackground, false);
  setAutoFillBackground(true);
  {
    const theme::ShellTokens shell = theme::Shell();
    QPalette pal = palette();
    pal.setColor(QPalette::Window, shell.panel_surface);
    pal.setColor(QPalette::Base, shell.panel_surface);
    setPalette(pal);
  }

  parking_ = new QWidget(this);
  parking_->setObjectName(QStringLiteral("MainPanelHostParking"));
  parking_->hide();

  root_splitter_ = new PanelSplitter(Qt::Horizontal, this);
  setCentralWidget(root_splitter_);

  // Opaque half-pane Tool window (global dest rect only).
  drop_overlay_ = new DropOverlay(this);
}

void MainPanelHost::parkDock(QDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  if (parking_ == nullptr) {
    parking_ = new QWidget(this);
    parking_->setObjectName(QStringLiteral("MainPanelHostParking"));
    parking_->hide();
  }
  const QSignalBlocker blocker(dock);
  if (dock->isFloating()) {
    dock->setFloating(false);
  }
  // Detach from splitter pane layout first.
  if (QWidget* parent = dock->parentWidget()) {
    if (QLayout* layout = parent->layout()) {
      layout->removeWidget(dock);
    }
  }
  if (dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
    removeDockWidget(dock);
  }
  // Park under the plain parking_ widget with Qt::Widget flags. Bare
  // setParent(non-QMainWindow) would promote QDockWidget to a top-level
  // window (transparent 3D). setParent(QMainWindow) engages dock machinery
  // and corrupts NoDockWidgetArea mosaic docks — avoid both.
  dock->setFeatures(QDockWidget::DockWidgetClosable);
  dock->setAllowedAreas(Qt::NoDockWidgetArea);
  dock->setParent(parking_, Qt::Widget);
  dock->hide();
}

QList<QDockWidget*> MainPanelHost::parkedPanels() const {
  QList<QDockWidget*> docks;
  if (parking_ == nullptr) {
    return docks;
  }
  for (QObject* child : parking_->children()) {
    if (auto* dock = qobject_cast<QDockWidget*>(child)) {
      docks.push_back(dock);
    }
  }
  return docks;
}

QSize MainPanelHost::minimumSizeHint() const { return QSize(0, 0); }

QSize MainPanelHost::sizeHint() const { return QSize(640, 480); }

void MainPanelHost::PreferredGridSize(int count, int* rows, int* cols) {
  if (rows == nullptr || cols == nullptr) {
    return;
  }
  if (count <= 0) {
    *rows = 0;
    *cols = 0;
    return;
  }
  if (count == 1) {
    *rows = 1;
    *cols = 1;
  } else if (count == 2) {
    *rows = 1;
    *cols = 2;
  } else if (count <= 4) {
    *rows = 2;
    *cols = 2;
  } else if (count <= 6) {
    *rows = 2;
    *cols = 3;
  } else if (count <= 9) {
    *rows = 3;
    *cols = 3;
  } else if (count <= 12) {
    *rows = 3;
    *cols = 4;
  } else {
    *rows = 4;
    *cols = 4;
  }
}

QList<QDockWidget*> MainPanelHost::hostedPanels() const {
  QList<QDockWidget*> docks;
  CollectDocks(root_splitter_, &docks);
  return docks;
}

bool MainPanelHost::hostsPanel(const QDockWidget* dock) const {
  if (dock == nullptr) {
    return false;
  }
  return hostedPanels().contains(const_cast<QDockWidget*>(dock));
}

void MainPanelHost::clearSplitterTree() {
  if (root_splitter_ == nullptr) {
    return;
  }
  const QList<QDockWidget*> docks = hostedPanels();
  for (QDockWidget* dock : docks) {
    parkDock(dock);
  }
  // Fully detach then destroy empty panes/splitters synchronously. deleteLater
  // races with the first activate/paint after loadConfig and corrupts the heap.
  QList<QWidget*> orphans;
  while (root_splitter_->count() > 0) {
    QWidget* child = root_splitter_->widget(0);
    child->setParent(nullptr);
    if (qobject_cast<QDockWidget*>(child) == nullptr) {
      orphans.push_back(child);
    }
  }
  root_splitter_->setOrientation(Qt::Horizontal);
  for (QWidget* orphan : orphans) {
    delete orphan;
  }
}

void MainPanelHost::removePanel(QDockWidget* dock) {
  if (dock == nullptr || !hostsPanel(dock)) {
    return;
  }
  // Collapse/scrub emit visibilityChanged; skipping re-entry prevents UAF in
  // ScrubEmptyPanesRecursive when the Displays/Panels menu hides a leaf.
  if (mutating_mosaic_) {
    parkDock(dock);
    return;
  }
  mutating_mosaic_ = true;
  QWidget* pane = dock->parentWidget();
  QPointer<QSplitter> parent_splitter;
  if (pane != nullptr && pane->property("mainPanelPane").toBool()) {
    parent_splitter = qobject_cast<QSplitter*>(pane->parentWidget());
  }
  parkDock(dock);
  if (pane != nullptr && pane->property("mainPanelPane").toBool()) {
    pane->setParent(nullptr);
    delete pane;
  }
  if (parent_splitter != nullptr) {
    CollapseDegenerateSplitter(parent_splitter.data(), root_splitter_);
  }
  // Inline scrub while mutating_mosaic_ is held (scrubEmptyPanes early-outs).
  if (root_splitter_ != nullptr) {
    ScrubEmptyPanesRecursive(root_splitter_, root_splitter_);
    if (root_splitter_->count() > 0) {
      EqualizeSplitter(root_splitter_);
      FillSplitter(root_splitter_);
    }
  }
  mutating_mosaic_ = false;
}

void MainPanelHost::scrubEmptyPanes() {
  if (root_splitter_ == nullptr || mutating_mosaic_) {
    return;
  }
  mutating_mosaic_ = true;
  ScrubEmptyPanesRecursive(root_splitter_, root_splitter_);
  // Remaining panes must expand into the freed space (closing Map / Plot /
  // duplicate 3D View / title-bar undock used to leave a blank gap).
  if (root_splitter_->count() > 0) {
    EqualizeSplitter(root_splitter_);
    FillSplitter(root_splitter_);
    const QPointer<MainPanelHost> self(this);
    QTimer::singleShot(0, this, [self]() {
      if (self == nullptr || self->root_splitter_ == nullptr ||
          self->mutating_mosaic_) {
        return;
      }
      EqualizeSplitter(self->root_splitter_);
      FillSplitter(self->root_splitter_);
      RaiseSplitterHandles(self->root_splitter_);
    });
  }
  RaiseSplitterHandles(root_splitter_);
  mutating_mosaic_ = false;
}

bool MainPanelHost::replacePanel(QDockWidget* old_dock, QDockWidget* new_dock) {
  if (old_dock == nullptr || new_dock == nullptr || old_dock == new_dock ||
      root_splitter_ == nullptr) {
    return false;
  }
  if (!hostsPanel(old_dock)) {
    return false;
  }

  // New dock must not remain in another mosaic leaf / QMainWindow area.
  if (hostsPanel(new_dock)) {
    removePanel(new_dock);
  }
  if (dockWidgetArea(new_dock) != Qt::NoDockWidgetArea) {
    removeDockWidget(new_dock);
  }

  QWidget* pane = FindPaneForDock(root_splitter_, old_dock);
  auto* layout =
      pane != nullptr ? qobject_cast<QVBoxLayout*>(pane->layout()) : nullptr;
  if (pane == nullptr || layout == nullptr ||
      !pane->property("mainPanelPane").toBool()) {
    return false;
  }

  PrepareDockForSplitter(new_dock);
  layout->removeWidget(old_dock);
  parkDock(old_dock);

  ApplyExpandingPolicy(new_dock);
  if (QWidget* content = new_dock->widget()) {
    ApplyExpandingPolicy(content);
  }
  layout->addWidget(new_dock, /*stretch=*/1);
  new_dock->show();
  new_dock->raise();

  FillSplitter(root_splitter_);
  RaiseSplitterHandles(root_splitter_);
  const QPointer<MainPanelHost> self(this);
  QTimer::singleShot(0, this, [self]() {
    if (self == nullptr || self->root_splitter_ == nullptr) {
      return;
    }
    FillSplitter(self->root_splitter_);
    RaiseSplitterHandles(self->root_splitter_);
  });
  return true;
}

void MainPanelHost::addPanel(QDockWidget* dock) {
  if (dock == nullptr || root_splitter_ == nullptr) {
    return;
  }
  if (hostsPanel(dock)) {
    dock->show();
    FillSplitter(root_splitter_);
    RaiseSplitterHandles(root_splitter_);
    return;
  }
  if (dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
    removeDockWidget(dock);
  }
  PrepareDockForSplitter(dock);
  root_splitter_->addWidget(WrapDockInPane(dock));
  dock->show();
  equalizeTopLevel();
  RaiseSplitterHandles(root_splitter_);
  // Layout may still report a stale host width; refill after geometry commits.
  const QPointer<MainPanelHost> self(this);
  QTimer::singleShot(0, this, [self]() {
    if (self == nullptr || self->root_splitter_ == nullptr) {
      return;
    }
    EqualizeSplitter(self->root_splitter_);
    FillSplitter(self->root_splitter_);
    RaiseSplitterHandles(self->root_splitter_);
  });
}

bool MainPanelHost::hitTestDrop(const QPoint& global_pos,
                                const QDockWidget* exclude,
                                QDockWidget** target_out,
                                Qt::Edge* edge_out) const {
  // Foxglove: drop against the mosaic leaf under the cursor (not host-global).
  return FindDropAt(root_splitter_, const_cast<MainPanelHost*>(this),
                    global_pos, exclude, target_out, edge_out, nullptr);
}

void MainPanelHost::beginMosaicDropSession() {
  if (drop_overlay_ == nullptr) {
    return;
  }
  // Keep host palette unchanged — siblings must look like idle mosaic panes.
  drop_overlay_->beginSession(rect(), hostedPanels().isEmpty());
}

void MainPanelHost::endMosaicDropSession() {
  if (drop_overlay_ != nullptr) {
    drop_overlay_->endSession();
  }
  const theme::ShellTokens shell = theme::Shell();
  QPalette pal = palette();
  pal.setColor(QPalette::Window, shell.panel_surface);
  pal.setColor(QPalette::Base, shell.panel_surface);
  setPalette(pal);
}

void MainPanelHost::setDropPreview(const QPoint& global_pos,
                                   const QDockWidget* exclude, bool visible) {
  if (drop_overlay_ == nullptr) {
    return;
  }
  if (!visible) {
    endMosaicDropSession();
    return;
  }

  auto dest_for_edge = [](const QRect& pane_rect, Qt::Edge edge) -> QRect {
    const int hw = qMax(1, pane_rect.width() / 2);
    const int hh = qMax(1, pane_rect.height() / 2);
    switch (edge) {
      case Qt::LeftEdge:
        return QRect(pane_rect.left(), pane_rect.top(), hw, pane_rect.height());
      case Qt::RightEdge:
        return QRect(pane_rect.right() - hw + 1, pane_rect.top(), hw,
                     pane_rect.height());
      case Qt::TopEdge:
        return QRect(pane_rect.left(), pane_rect.top(), pane_rect.width(), hh);
      case Qt::BottomEdge:
        return QRect(pane_rect.left(), pane_rect.bottom() - hh + 1,
                     pane_rect.width(), hh);
      default:
        return QRect();
    }
  };

  auto edge_for_point = [](const QRect& pane_rect, const QPoint& local,
                           Qt::Edge* edge_out) -> bool {
    if (edge_out == nullptr || !pane_rect.contains(local)) {
      return false;
    }
    const double w = static_cast<double>(qMax(1, pane_rect.width()));
    const double h = static_cast<double>(qMax(1, pane_rect.height()));
    const double x_rate =
        static_cast<double>(local.x() - pane_rect.left()) / w;
    const double y_rate =
        static_cast<double>(local.y() - pane_rect.top()) / h;
    const double dist_l = x_rate;
    const double dist_r = 1.0 - x_rate;
    const double dist_t = y_rate;
    const double dist_b = 1.0 - y_rate;
      // Slight bias toward top/bottom so 下放 half is easy to aim on wide panes.
    const double d_l = dist_l;
    const double d_r = dist_r;
    const double d_t = dist_t * 0.92;
    const double d_b = dist_b * 0.92;
    if (d_b <= d_t && d_b <= d_l && d_b <= d_r) {
      *edge_out = Qt::BottomEdge;
    } else if (d_t <= d_l && d_t <= d_r) {
      *edge_out = Qt::TopEdge;
    } else if (d_l <= d_r) {
      *edge_out = Qt::LeftEdge;
    } else {
      *edge_out = Qt::RightEdge;
    }
    return true;
  };

  const QRect host_rect = rect();
  QDockWidget* target = nullptr;
  Qt::Edge edge = Qt::LeftEdge;
  QRect pane_in_host;
  QRect dest_in_host;

  // Prefer the leaf under the cursor; fall back to full-host half.
  if (FindDropAt(root_splitter_, this, global_pos, exclude, &target, &edge,
                 &dest_in_host) &&
      target != nullptr) {
    if (QWidget* pane = FindPaneForDock(root_splitter_, target)) {
      pane_in_host = QRect(pane->mapTo(this, QPoint(0, 0)), pane->size());
      const QPoint local = pane->mapFromGlobal(global_pos);
      edge_for_point(pane->rect(), local, &edge);
      dest_in_host = dest_for_edge(pane_in_host, edge);
    }
  } else {
    const QPoint local = mapFromGlobal(global_pos);
    if (!edge_for_point(host_rect, local, &edge)) {
      drop_overlay_->setHalfPanePreview(host_rect, QRect(), QRect(), edge);
      drop_overlay_->raise();
      return;
    }
    pane_in_host = host_rect;
    dest_in_host = dest_for_edge(host_rect, edge);
  }

  // Only the target leaf half is painted; siblings stay visible.
  drop_overlay_->setHalfPanePreview(host_rect, pane_in_host, dest_in_host,
                                    edge);
  drop_overlay_->raise();
}

bool MainPanelHost::tryAcceptDrop(QDockWidget* dragged,
                                  const QPoint& global_pos) {
  setDropPreview(global_pos, dragged, false);
  QDockWidget* target = nullptr;
  Qt::Edge edge = Qt::LeftEdge;
  if (!hitTestDrop(global_pos, dragged, &target, &edge) || target == nullptr) {
    return false;
  }
  return dockPanelAt(dragged, target, edge);
}

bool MainPanelHost::dockPanelAt(QDockWidget* dragged, QDockWidget* target,
                                Qt::Edge edge) {
  if (dragged == nullptr || target == nullptr || root_splitter_ == nullptr ||
      dragged == target) {
    return false;
  }

  // Extract dragged first — removePanel manages its own mutating_mosaic_ flag.
  // Holding the flag across removePanel would early-out to park-only and leave
  // a stale leaf in the splitter tree.
  if (hostsPanel(dragged)) {
    removePanel(dragged);
  }
  if (dockWidgetArea(dragged) != Qt::NoDockWidgetArea) {
    removeDockWidget(dragged);
  }
  if (dockWidgetArea(target) != Qt::NoDockWidgetArea) {
    removeDockWidget(target);
  }

  // Hold through replaceWidget / show so visibilityChanged cannot re-enter
  // scrub/removePanel (layout scramble / UAF during drag-snap).
  const bool was_mutating = mutating_mosaic_;
  mutating_mosaic_ = true;

  ScrubEmptyPanesRecursive(root_splitter_, root_splitter_);

  const auto park = [this](QDockWidget* dock) { parkDock(dock); };

  // Target may have been collapsed away if it was the only sibling of dragged
  // in a degenerate tree — re-seed when needed.
  if (!hostsPanel(target)) {
    PrepareDockForSplitter(target);
    FlattenToSoleRootLeaf(root_splitter_, target, park);
  }

  // After title-drag park, usually only @p target remains. Flatten so Left/Right
  // reuses the root as Horizontal — nesting under a Vertical remnant leaves the
  // black top band + grey|purple strip the user screenshots.
  {
    QList<QDockWidget*> remaining;
    CollectDocks(root_splitter_, &remaining);
    remaining.removeAll(dragged);
    if (remaining.size() == 1 && remaining[0] == target) {
      QWidget* pane = FindPaneForDock(root_splitter_, target);
      const bool already_sole =
          pane != nullptr && pane->parentWidget() == root_splitter_ &&
          root_splitter_->count() == 1;
      if (!already_sole) {
        FlattenToSoleRootLeaf(root_splitter_, target, park);
      }
    }
  }

  PrepareDockForSplitter(dragged);
  PrepareDockForSplitter(target);

  QWidget* target_pane = FindPaneForDock(root_splitter_, target);
  if (target_pane == nullptr) {
    FlattenToSoleRootLeaf(root_splitter_, target, park);
    target_pane = FindPaneForDock(root_splitter_, target);
  }
  if (target_pane == nullptr) {
    mutating_mosaic_ = was_mutating;
    return false;
  }

  auto* parent = qobject_cast<QSplitter*>(target_pane->parentWidget());
  if (parent == nullptr) {
    parent = root_splitter_;
    if (parent->indexOf(target_pane) < 0) {
      parent->addWidget(target_pane);
    }
  }

  const int index = parent->indexOf(target_pane);
  if (index < 0) {
    mutating_mosaic_ = was_mutating;
    return false;
  }

  const Qt::Orientation orientation =
      (edge == Qt::LeftEdge || edge == Qt::RightEdge) ? Qt::Horizontal
                                                      : Qt::Vertical;
  const bool dragged_first =
      (edge == Qt::LeftEdge || edge == Qt::TopEdge);

  QList<int> parent_sizes = parent->sizes();
  QWidget* dragged_pane = WrapDockInPane(dragged);

  // Sole root leaf → reuse root orientation instead of nesting.
  if (parent == root_splitter_ && root_splitter_->count() == 1) {
    root_splitter_->setOrientation(orientation);
    if (dragged_first) {
      root_splitter_->insertWidget(0, dragged_pane);
    } else {
      root_splitter_->addWidget(dragged_pane);
    }
    target->show();
    dragged->show();
    ApplyExpandingPolicy(root_splitter_);
    ScrubEmptyPanesRecursive(root_splitter_, root_splitter_);
    if (root_splitter_->count() > 0) {
      EqualizeSplitter(root_splitter_);
      FillSplitter(root_splitter_);
      RaiseSplitterHandles(root_splitter_);
    }
    mutating_mosaic_ = was_mutating;
    return true;
  }

  // Prefer promoting a 2-child root to the drop orientation instead of nesting
  // a Horizontal under Vertical (black top + side strip).
  if (parent == root_splitter_ && root_splitter_->count() == 2 &&
      orientation != root_splitter_->orientation()) {
    QWidget* sibling =
        root_splitter_->widget(index == 0 ? 1 : 0);
    // Only safe when sibling is the other leaf of a pure 2-dock mosaic and the
    // dragged is new — rebuild as flat root in @p orientation.
    QList<QDockWidget*> docks;
    CollectDocks(root_splitter_, &docks);
    if (docks.size() == 1 && docks[0] == target) {
      FlattenToSoleRootLeaf(root_splitter_, target, park);
      root_splitter_->setOrientation(orientation);
      if (dragged_first) {
        root_splitter_->insertWidget(0, dragged_pane);
      } else {
        root_splitter_->addWidget(dragged_pane);
      }
      target->show();
      dragged->show();
      ApplyExpandingPolicy(root_splitter_);
      EqualizeSplitter(root_splitter_);
      FillSplitter(root_splitter_);
      RaiseSplitterHandles(root_splitter_);
      mutating_mosaic_ = was_mutating;
      return true;
    }
    (void)sibling;
  }

  const int parent_span = parent->orientation() == Qt::Horizontal
                              ? qMax(parent->width(), width())
                              : qMax(parent->height(), height());
  int span = parent_sizes.value(index, 0);
  if (span < kMinPaneSpan * 2) {
    span = orientation == Qt::Horizontal
               ? qMax(qMax(target_pane->width(), parent_span),
                      qMax(width(), 160))
               : qMax(qMax(target_pane->height(), parent_span),
                      qMax(height(), 120));
  }

  // When inserting a horizontal split into a vertical parent (or vice versa),
  // span must be the TARGET leaf span along the NEW orientation axis — using
  // the parent's vertical size as width leaves the nested strip too short and
  // a black void above/beside it.
  if (orientation != parent->orientation()) {
    span = orientation == Qt::Horizontal
               ? qMax(qMax(target_pane->width(), width()), 160)
               : qMax(qMax(target_pane->height(), height()), 120);
  }

  auto* nested = new PanelSplitter(orientation);
  QWidget* replaced = parent->replaceWidget(index, nested);
  if (replaced == nullptr) {
    replaced = target_pane;
  }
  if (dragged_first) {
    nested->addWidget(dragged_pane);
    nested->addWidget(replaced);
  } else {
    nested->addWidget(replaced);
    nested->addWidget(dragged_pane);
  }

  target->show();
  dragged->show();

  const int half = qMax(span / 2, kMinPaneSpan);
  nested->setSizes({half, qMax(span - half, kMinPaneSpan)});
  nested->setStretchFactor(0, 1);
  nested->setStretchFactor(1, 1);
  if (parent_sizes.size() == parent->count()) {
    parent->setSizes(parent_sizes);
  }
  ApplyExpandingPolicy(root_splitter_);
  ScrubEmptyPanesRecursive(root_splitter_, root_splitter_);
  if (root_splitter_->count() > 0) {
    EqualizeSplitter(root_splitter_);
    FillSplitter(root_splitter_);
    RaiseSplitterHandles(root_splitter_);
  }
  mutating_mosaic_ = was_mutating;
  return true;
}

void MainPanelHost::splitPanel(QDockWidget* first, QDockWidget* second,
                               Qt::Orientation orientation) {
  // Foxglove / react-mosaic: replace the leaf with
  //   { first: original, second: newPanel, direction }
  // where direction "row"  == Qt::Horizontal (Split right)
  //       direction "column" == Qt::Vertical   (Split down).
  // Only that leaf is affected; sibling panes keep size and position.
  if (first == nullptr || second == nullptr || root_splitter_ == nullptr ||
      first == second) {
    return;
  }
  if (dockWidgetArea(first) != Qt::NoDockWidgetArea) {
    removeDockWidget(first);
  }
  if (dockWidgetArea(second) != Qt::NoDockWidgetArea) {
    removeDockWidget(second);
  }

  // Hold through replaceWidget / show — same re-entry hazard as dockPanelAt.
  const bool was_mutating = mutating_mosaic_;
  mutating_mosaic_ = true;

  PrepareDockForSplitter(first);
  PrepareDockForSplitter(second);

  QWidget* first_pane = FindPaneForDock(root_splitter_, first);
  if (first_pane == nullptr) {
    // Original is not in the mosaic yet — seed it as the sole root leaf.
    first_pane = WrapDockInPane(first);
    root_splitter_->addWidget(first_pane);
  }

  auto* parent = qobject_cast<QSplitter*>(first_pane->parentWidget());
  if (parent == nullptr) {
    parent = root_splitter_;
    root_splitter_->addWidget(first_pane);
  }

  const int index = parent->indexOf(first_pane);
  if (index < 0) {
    mutating_mosaic_ = was_mutating;
    return;
  }

  QList<int> parent_sizes = parent->sizes();

  // Path [] leaf → become the root split node (no extra nesting wrapper).
  if (parent == root_splitter_ && root_splitter_->count() == 1) {
    root_splitter_->setOrientation(orientation);
    QWidget* second_pane = WrapDockInPane(second);
    root_splitter_->addWidget(second_pane);
    first->show();
    second->show();
    EqualizeSplitter(root_splitter_);
    FillSplitter(root_splitter_);
    RaiseSplitterHandles(root_splitter_);
    mutating_mosaic_ = was_mutating;
    const QPointer<MainPanelHost> self(this);
    QTimer::singleShot(0, this, [self]() {
      if (self == nullptr || self->root_splitter_ == nullptr ||
          self->mutating_mosaic_) {
        return;
      }
      EqualizeSplitter(self->root_splitter_);
      FillSplitter(self->root_splitter_);
      RaiseSplitterHandles(self->root_splitter_);
    });
    return;
  }

  // Prefer the parent's allocated span for this leaf; fall back to host geometry
  // so a not-yet-laid-out pane does not lock in a tiny split (gap in the middle).
  const int parent_span = parent->orientation() == Qt::Horizontal
                              ? qMax(parent->width(), width())
                              : qMax(parent->height(), height());
  int span = parent_sizes.value(index, 0);
  if (span < kMinPaneSpan * 2) {
    span = orientation == Qt::Horizontal
               ? qMax(qMax(first_pane->width(), parent_span),
                      qMax(width(), 160))
               : qMax(qMax(first_pane->height(), parent_span),
                      qMax(height(), 120));
  }

  auto* nested = new PanelSplitter(orientation);
  QWidget* replaced = parent->replaceWidget(index, nested);
  if (replaced == nullptr) {
    replaced = first_pane;
  }
  nested->addWidget(replaced);
  QWidget* second_pane = WrapDockInPane(second);
  nested->addWidget(second_pane);

  first->show();
  second->show();

  const int half = qMax(span / 2, kMinPaneSpan);
  nested->setSizes({half, qMax(span - half, kMinPaneSpan)});
  nested->setStretchFactor(0, 1);
  nested->setStretchFactor(1, 1);
  if (parent_sizes.size() == parent->count()) {
    parent->setSizes(parent_sizes);
  }
  FillSplitter(root_splitter_);
  RaiseSplitterHandles(root_splitter_);
  mutating_mosaic_ = was_mutating;
  const QPointer<MainPanelHost> self(this);
  QTimer::singleShot(0, this, [self]() {
    if (self == nullptr || self->root_splitter_ == nullptr ||
        self->mutating_mosaic_) {
      return;
    }
    FillSplitter(self->root_splitter_);
    RaiseSplitterHandles(self->root_splitter_);
  });
}

void MainPanelHost::equalizeTopLevel() {
  if (root_splitter_ == nullptr || root_splitter_->count() <= 0) {
    return;
  }
  // Use the splitter's own geometry (not the host) and assign stretch so a
  // later window resize continues to fill without leaving a centre gap.
  if (root_splitter_->width() < 2 || root_splitter_->height() < 2) {
    root_splitter_->resize(qMax(width(), 1), qMax(height(), 1));
  }
  EqualizeSplitter(root_splitter_);
  FillSplitter(root_splitter_);
}

void MainPanelHost::tilePanels(const QList<QDockWidget*>& visible,
                               const QList<QDockWidget*>& hidden) {
  if (tiling_ || root_splitter_ == nullptr) {
    return;
  }
  tiling_ = true;

  for (QDockWidget* dock : hidden) {
    if (dock == nullptr) {
      continue;
    }
    if (dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
      removeDockWidget(dock);
    }
    PrepareDockForSplitter(dock);
    parkDock(dock);
  }

  for (QDockWidget* dock : visible) {
    if (dock == nullptr) {
      continue;
    }
    if (dockWidgetArea(dock) != Qt::NoDockWidgetArea) {
      removeDockWidget(dock);
    }
    PrepareDockForSplitter(dock);
  }

  clearSplitterTree();

  if (visible.isEmpty()) {
    tiling_ = false;
    return;
  }

  int rows = 0;
  int cols = 0;
  PreferredGridSize(visible.size(), &rows, &cols);

  if (rows <= 1) {
    root_splitter_->setOrientation(Qt::Horizontal);
    for (QDockWidget* dock : visible) {
      root_splitter_->addWidget(WrapDockInPane(dock));
      dock->show();
      dock->raise();
    }
  } else {
    root_splitter_->setOrientation(Qt::Vertical);
    for (int r = 0; r < rows; ++r) {
      auto* row = new PanelSplitter(Qt::Horizontal, root_splitter_);
      bool row_has_panel = false;
      for (int c = 0; c < cols; ++c) {
        const int index = r * cols + c;
        if (index >= visible.size()) {
          break;
        }
        QDockWidget* dock = visible.at(index);
        row->addWidget(WrapDockInPane(dock));
        dock->show();
        dock->raise();
        row_has_panel = true;
      }
      if (row_has_panel) {
        root_splitter_->addWidget(row);
        EqualizeSplitter(row);
        RaiseSplitterHandles(row);
      } else {
        delete row;
      }
    }
  }

  equalizeTopLevel();
  RaiseSplitterHandles(root_splitter_);
  tiling_ = false;
}

void MainPanelHost::syncHorizontalDockLayout() {
  if (root_splitter_ == nullptr) {
    return;
  }
  // Scale existing ratios to the current host size (do not destroy mosaic).
  FillSplitter(root_splitter_);
  RaiseSplitterHandles(root_splitter_);
}

QByteArray MainPanelHost::saveMosaicLayout() const {
  if (root_splitter_ == nullptr) {
    return {};
  }
  QJsonObject document;
  document.insert(QStringLiteral("root"), SerializeMosaicNode(root_splitter_));
  const QByteArray json =
      QJsonDocument(document).toJson(QJsonDocument::Compact);
  if (json.isEmpty() || json == QByteArrayLiteral("{}")) {
    return {};
  }
  return QByteArray(kMosaicPrefix) + json;
}

bool MainPanelHost::restoreMosaicLayout(
    const QByteArray& encoded,
    const std::function<QDockWidget*(const QString& object_name)>& resolve) {
  if (root_splitter_ == nullptr || !encoded.startsWith(kMosaicPrefix)) {
    return false;
  }
  const QByteArray json = encoded.mid(static_cast<int>(sizeof(kMosaicPrefix) - 1));
  const QJsonDocument document = QJsonDocument::fromJson(json);
  if (!document.isObject()) {
    return false;
  }
  const QJsonObject root_object =
      document.object().value(QStringLiteral("root")).toObject();
  if (root_object.isEmpty()) {
    return false;
  }

  // Hold mutating_mosaic_ so visibilityChanged from park/show cannot re-enter
  // removePanel / addPanel and leave an empty title-less pane (blank column).
  const bool was_mutating = mutating_mosaic_;
  mutating_mosaic_ = true;

  // Clear BEFORE BuildMosaicNode. WrapDockInPane reuses an existing
  // mainPanelPane parent; if we build first then clearSplitterTree(), those
  // panes are deleteLater()'d while still referenced by the new tree — UAF
  // (often QPalette/QBrush crash on the next activate/paint).
  QList<QDockWidget*> previously_hosted = hostedPanels();
  clearSplitterTree();
  for (QDockWidget* dock : previously_hosted) {
    if (dock == nullptr) {
      continue;
    }
    if (dock->property("mosaicPlaceholder").toBool()) {
      // Placeholders are derived state: drop them so repeated restores cannot
      // stack duplicates, and let the rebuild recreate the ones still needed.
      dock->setProperty("panelDisposed", true);
      dock->deleteLater();
      continue;
    }
    PrepareDockForSplitter(dock);
    parkDock(dock);
  }

  QList<QDockWidget*> placed;
  const auto logging_resolve =
      [&resolve](const QString& name) -> QDockWidget* {
    QDockWidget* dock = resolve ? resolve(name) : nullptr;
    if (dock == nullptr) {
      LOG(WARNING) << "mosaic: unresolved dock '" << name.toStdString()
                   << "'";
    }
    return dock;
  };
  QWidget* built = BuildMosaicNode(root_object, logging_resolve, &placed);
  LOG(INFO) << "mosaic: placed " << placed.size() << " pane(s), built="
            << (built != nullptr);
  if (built == nullptr || placed.isEmpty()) {
    mutating_mosaic_ = was_mutating;
    return false;
  }

  for (QDockWidget* dock : previously_hosted) {
    if (dock != nullptr && !placed.contains(dock)) {
      parkDock(dock);
    }
  }

  if (auto* as_splitter = qobject_cast<QSplitter*>(built)) {
    // Replace root contents with the restored splitter's children / orientation.
    root_splitter_->setOrientation(as_splitter->orientation());
    const QList<int> sizes = as_splitter->sizes();
    while (as_splitter->count() > 0) {
      // addWidget reparents; avoid setParent(nullptr) (top-level flash / UAF).
      root_splitter_->addWidget(as_splitter->widget(0));
    }
    if (sizes.size() == root_splitter_->count()) {
      root_splitter_->setSizes(sizes);
    }
    delete as_splitter;
  } else {
    root_splitter_->setOrientation(Qt::Horizontal);
    root_splitter_->addWidget(built);
  }

  for (QDockWidget* dock : placed) {
    if (dock != nullptr) {
      dock->show();
      dock->raise();
    }
  }
  // Drop any title-less empty panes left by park/reparent churn, then refill.
  ScrubEmptyPanesRecursive(root_splitter_, root_splitter_);
  if (root_splitter_->count() > 0) {
    EqualizeSplitter(root_splitter_);
    FillSplitter(root_splitter_);
  }
  RaiseSplitterHandles(root_splitter_);
  mutating_mosaic_ = was_mutating;
  const QPointer<MainPanelHost> self(this);
  QTimer::singleShot(0, this, [self]() {
    if (self == nullptr || self->root_splitter_ == nullptr ||
        self->mutating_mosaic_) {
      return;
    }
    self->scrubEmptyPanes();
  });
  return true;
}

void MainPanelHost::resizeEvent(QResizeEvent* event) {
  QMainWindow::resizeEvent(event);
  // Drop overlay geometry is the half-pane rect only — do not expand to host.
  // Only top up when there is a real gap. Unconditional FillSplitter here
  // fights Ogre resize and produces perpetual 3D View flicker.
  if (root_splitter_ != nullptr && !tiling_ && root_splitter_->count() > 0) {
    const int handle_space =
        root_splitter_->handleWidth() * qMax(0, root_splitter_->count() - 1);
    const int available =
        (root_splitter_->orientation() == Qt::Horizontal
             ? root_splitter_->width()
             : root_splitter_->height()) -
        handle_space;
    int sum = 0;
    for (int size : root_splitter_->sizes()) {
      sum += size;
    }
    if (available - sum > 8) {
      FillSplitter(root_splitter_);
    }
  }
  RaiseSplitterHandles(root_splitter_);
}

}  // namespace autoviz
