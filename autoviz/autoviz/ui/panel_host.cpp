/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_host.hpp"

#include <QDockWidget>
#include <QEnterEvent>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPainter>
#include <QPaintEvent>
#include <QPointer>
#include <QResizeEvent>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSplitter>
#include <QSplitterHandle>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

/** Hit target width — visual line stays 1px so the seam reads as a border. */
constexpr int kSplitterHit = 7;
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
  const int each = qMax(available / count, kMinPaneSpan);
  QList<int> sizes;
  sizes.reserve(count);
  for (int i = 0; i < count; ++i) {
    sizes.push_back(each);
  }
  splitter->setSizes(sizes);
}

class PanelSplitterHandle : public QSplitterHandle {
 public:
  explicit PanelSplitterHandle(Qt::Orientation orientation, QSplitter* parent)
      : QSplitterHandle(orientation, parent) {}

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.fillRect(rect(), Qt::transparent);

    // Idle: barely-there cool seam; hover: soft cyan drag affordance.
    const bool hover = underMouse();
    const glass::ShellTokens shell = glass::Shell();
    QColor line = hover ? shell.accent : shell.border;
    line.setAlpha(hover ? 140 : 90);
    painter.setPen(QPen(line, 1));
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
    setHandleWidth(kSplitterHit);
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
  splitter->setHandleWidth(kSplitterHit);
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
  auto* pane = new QWidget();
  pane->setProperty("mainPanelPane", true);
  // Wide enough for dock title tools (settings/split/change/expand/close).
  pane->setMinimumSize(220, 80);
  ApplyExpandingPolicy(pane);
  pane->setMinimumSize(220, 80);
  pane->setAttribute(Qt::WA_StyledBackground, true);
  pane->setStyleSheet(style::mark(style::Mark::Clear));
  auto* layout = new QVBoxLayout(pane);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  ApplyExpandingPolicy(dock);
  if (QWidget* content = dock->widget()) {
    ApplyExpandingPolicy(content);
  }
  layout->addWidget(dock, /*stretch=*/1);
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

/** Foxglove/react-mosaic remove: collapse a splitter that has a single child. */
void CollapseDegenerateSplitter(QSplitter* splitter, QSplitter* root) {
  while (splitter != nullptr && splitter != root && splitter->count() == 1) {
    QWidget* only = splitter->widget(0);
    auto* parent = qobject_cast<QSplitter*>(splitter->parentWidget());
    if (parent == nullptr || only == nullptr) {
      break;
    }
    const int index = parent->indexOf(splitter);
    QList<int> sizes = parent->sizes();
    parent->replaceWidget(index, only);
    splitter->setParent(nullptr);
    splitter->deleteLater();
    if (sizes.size() == parent->count()) {
      parent->setSizes(sizes);
    }
    splitter = parent;
  }
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

constexpr char kMosaicPrefix[] = "mosaic:v1:";

QJsonObject SerializeMosaicNode(QWidget* widget) {
  QJsonObject node;
  if (widget == nullptr) {
    return node;
  }
  if (QDockWidget* dock = DockFromPane(widget)) {
    node.insert(QStringLiteral("type"), QStringLiteral("leaf"));
    node.insert(QStringLiteral("name"), dock->objectName());
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
      return nullptr;
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

MainPanelHost::MainPanelHost(QWidget* parent) : QMainWindow(parent) {
  setObjectName(QStringLiteral("MainPanelHost"));
  setWindowFlags(Qt::Widget);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  setMinimumSize(0, 0);
  setDockOptions(QMainWindow::AllowNestedDocks | QMainWindow::AllowTabbedDocks);
  setDockNestingEnabled(false);
  setContentsMargins(0, 0, 0, 0);

  parking_ = new QWidget(this);
  parking_->setObjectName(QStringLiteral("MainPanelHostParking"));
  parking_->hide();

  root_splitter_ = new PanelSplitter(Qt::Horizontal, this);
  setCentralWidget(root_splitter_);
}

void MainPanelHost::parkDock(QDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  if (parking_ == nullptr) {
    parking_ = new QWidget(this);
    parking_->hide();
  }
  const QSignalBlocker blocker(dock);
  dock->setParent(parking_);
  dock->hide();
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
  QWidget* pane = dock->parentWidget();
  QSplitter* parent_splitter = nullptr;
  if (pane != nullptr && pane->property("mainPanelPane").toBool()) {
    parent_splitter = qobject_cast<QSplitter*>(pane->parentWidget());
  }
  parkDock(dock);
  if (pane != nullptr && pane->property("mainPanelPane").toBool()) {
    pane->setParent(nullptr);
    delete pane;
  }
  CollapseDegenerateSplitter(parent_splitter, root_splitter_);
  // Remaining panes must expand into the freed space (closing Map / Plot /
  // duplicate 3D View used to leave a half-width gap beside the survivor).
  if (root_splitter_ != nullptr && root_splitter_->count() > 0) {
    FillSplitter(root_splitter_);
    const QPointer<MainPanelHost> self(this);
    QTimer::singleShot(0, this, [self]() {
      if (self == nullptr || self->root_splitter_ == nullptr) {
        return;
      }
      FillSplitter(self->root_splitter_);
      RaiseSplitterHandles(self->root_splitter_);
    });
  }
  RaiseSplitterHandles(root_splitter_);
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
    RaiseSplitterHandles(root_splitter_);
    const QPointer<MainPanelHost> self(this);
    QTimer::singleShot(0, this, [self]() {
      if (self == nullptr || self->root_splitter_ == nullptr) {
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
  const QPointer<MainPanelHost> self(this);
  QTimer::singleShot(0, this, [self]() {
    if (self == nullptr || self->root_splitter_ == nullptr) {
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
    PrepareDockForSplitter(dock);
    parkDock(dock);
  }

  QList<QDockWidget*> placed;
  QWidget* built = BuildMosaicNode(root_object, resolve, &placed);
  if (built == nullptr || placed.isEmpty()) {
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

void MainPanelHost::resizeEvent(QResizeEvent* event) {
  QMainWindow::resizeEvent(event);
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
