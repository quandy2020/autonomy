/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel_host.hpp"

#include <QDockWidget>
#include <QEnterEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSplitter>
#include <QSplitterHandle>
#include <QVBoxLayout>
#include <QWidget>

#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

/** Hit target width — visual line stays 1px so the seam reads as a border. */
constexpr int kSplitterHit = 7;

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
  if (QWidget* parent = dock->parentWidget()) {
    if (parent->property("mainPanelPane").toBool()) {
      return parent;
    }
  }
  auto* pane = new QWidget();
  pane->setProperty("mainPanelPane", true);
  // Wide enough for dock title tools (settings/split/change/expand/close).
  pane->setMinimumSize(220, 80);
  pane->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  pane->setAttribute(Qt::WA_StyledBackground, true);
  pane->setStyleSheet(style::mark(style::Mark::Clear));
  auto* layout = new QVBoxLayout(pane);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);
  layout->addWidget(dock);
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
  // Avoid re-entrant visibilityChanged when already docked in the splitter.
  if (dock->isFloating()) {
    const QSignalBlocker blocker(dock);
    dock->setFloating(false);
  }
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

  root_splitter_ = new PanelSplitter(Qt::Horizontal, this);
  setCentralWidget(root_splitter_);
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
    if (dock == nullptr) {
      continue;
    }
    dock->setParent(this);
    dock->hide();
  }
  while (root_splitter_->count() > 0) {
    QWidget* child = root_splitter_->widget(0);
    child->setParent(nullptr);
    if (qobject_cast<QDockWidget*>(child) == nullptr) {
      child->deleteLater();
    }
  }
  root_splitter_->setOrientation(Qt::Horizontal);
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
  dock->setParent(this);
  dock->hide();
  if (pane != nullptr && pane->property("mainPanelPane").toBool()) {
    pane->setParent(nullptr);
    pane->deleteLater();
  }
  CollapseDegenerateSplitter(parent_splitter, root_splitter_);
  RaiseSplitterHandles(root_splitter_);
}

void MainPanelHost::addPanel(QDockWidget* dock) {
  if (dock == nullptr || root_splitter_ == nullptr) {
    return;
  }
  if (hostsPanel(dock)) {
    dock->show();
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
  const int span =
      parent_sizes.value(index, orientation == Qt::Horizontal
                                    ? qMax(first_pane->width(), 160)
                                    : qMax(first_pane->height(), 120));

  // Path [] leaf → become the root split node (no extra nesting wrapper).
  if (parent == root_splitter_ && root_splitter_->count() == 1) {
    root_splitter_->setOrientation(orientation);
    QWidget* second_pane = WrapDockInPane(second);
    root_splitter_->addWidget(second_pane);
    first->show();
    second->show();
    const int half = qMax(span / 2, 80);
    root_splitter_->setSizes({half, qMax(span - half, 80)});
    RaiseSplitterHandles(root_splitter_);
    return;
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

  const int half = qMax(span / 2, 80);
  nested->setSizes({half, qMax(span - half, 80)});
  if (parent_sizes.size() == parent->count()) {
    parent->setSizes(parent_sizes);
  }
  RaiseSplitterHandles(root_splitter_);
}

void MainPanelHost::equalizeTopLevel() {
  if (root_splitter_ == nullptr || root_splitter_->count() <= 0) {
    return;
  }
  QList<int> sizes;
  sizes.reserve(root_splitter_->count());
  const int total = root_splitter_->orientation() == Qt::Horizontal
                        ? qMax(width(), 1)
                        : qMax(height(), 1);
  const int each = qMax(total / root_splitter_->count(), 80);
  for (int i = 0; i < root_splitter_->count(); ++i) {
    sizes.push_back(each);
  }
  root_splitter_->setSizes(sizes);
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
    dock->setParent(this);
    dock->hide();
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
        QList<int> widths;
        const int cell = qMax(width() / qMax(row->count(), 1), 80);
        for (int i = 0; i < row->count(); ++i) {
          widths.push_back(cell);
        }
        row->setSizes(widths);
        RaiseSplitterHandles(row);
      } else {
        row->deleteLater();
      }
    }
  }

  equalizeTopLevel();
  RaiseSplitterHandles(root_splitter_);
  tiling_ = false;
}

void MainPanelHost::syncHorizontalDockLayout() {
  RaiseSplitterHandles(root_splitter_);
}

void MainPanelHost::resizeEvent(QResizeEvent* event) {
  QMainWindow::resizeEvent(event);
  RaiseSplitterHandles(root_splitter_);
}

}  // namespace autoviz
