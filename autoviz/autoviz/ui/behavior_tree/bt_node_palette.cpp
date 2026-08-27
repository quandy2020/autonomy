/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/behavior_tree/bt_node_palette.hpp"

#include "autoviz/ui/behavior_tree/bt_graph_view.hpp"
#include "autoviz/ui/behavior_tree/bt_icon_loader.hpp"
#include "autoviz/ui/behavior_tree/bt_node_models.hpp"

#include <QColor>
#include <QDrag>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QMimeData>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QToolButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

namespace autoviz {
namespace behavior_tree {
namespace {

constexpr char kBg[] = "#f8f9fb";
constexpr char kText[] = "#0f172a";
constexpr char kTextMuted[] = "#64748b";
constexpr char kBorder[] = "#e2e8f0";
constexpr char kAccent[] = "#0891b2";
constexpr char kGrootCustomBlue[] = "#466e9a";

QString PaletteToolButtonStyle() {
  return QStringLiteral(
      "QToolButton {"
      "  background: transparent; border: 1px solid transparent;"
      "  border-radius: 6px; padding: 3px; min-width: 26px; max-width: 26px;"
      "  min-height: 26px; max-height: 26px;"
      "}"
      "QToolButton:hover { background: rgba(8,145,178,0.10); border-color: %1; }"
      "QToolButton:checked { background: rgba(8,145,178,0.16); border-color: %2; }")
      .arg(QLatin1String(kBorder), QLatin1String(kAccent));
}

QToolButton* MakePaletteButton(QWidget* parent, const QIcon& icon, const QString& tip,
                               bool checkable = false) {
  auto* button = new QToolButton(parent);
  button->setIcon(icon);
  button->setIconSize(QSize(18, 18));
  button->setToolButtonStyle(Qt::ToolButtonIconOnly);
  button->setToolTip(tip);
  button->setCheckable(checkable);
  button->setStyleSheet(PaletteToolButtonStyle());
  button->setAutoRaise(true);
  return button;
}

QIcon GrootPaletteIcon(const QString& relative_path) {
  return BtIconLoader::toolbarIcon(relative_path, 18);
}

constexpr int kRegistrationRole = Qt::UserRole + 1;
constexpr int kKindRole = Qt::UserRole + 2;

QString PortDirectionLabel(BtPortDirection direction) {
  switch (direction) {
    case BtPortDirection::kInput:
      return QStringLiteral("Input");
    case BtPortDirection::kOutput:
      return QStringLiteral("Output");
    case BtPortDirection::kInOut:
      return QStringLiteral("InOut");
  }
  return QStringLiteral("Input");
}

class BtPaletteTreeWidget : public QTreeWidget {
 public:
  explicit BtPaletteTreeWidget(bool* locked_flag, QWidget* parent = nullptr)
      : QTreeWidget(parent), locked_flag_(locked_flag) {}

 protected:
  void startDrag(Qt::DropActions supported_actions) override {
    if (locked_flag_ != nullptr && *locked_flag_) {
      return;
    }
    QTreeWidgetItem* item = currentItem();
    if (item == nullptr || !item->data(0, kRegistrationRole).isValid()) {
      return;
    }
    const QString registration_id = item->data(0, kRegistrationRole).toString();
    const BtNodeKind kind =
        static_cast<BtNodeKind>(item->data(0, kKindRole).toInt());
    if (registration_id.isEmpty()) {
      return;
    }
    auto* drag = new QDrag(this);
    drag->setMimeData(MakeBtNodeDragPayload(registration_id, kind));
    drag->exec(supported_actions, Qt::CopyAction);
  }

 private:
  bool* locked_flag_ = nullptr;
};

}  // namespace

BtNodePalette::BtNodePalette(QWidget* parent) : QWidget(parent) {
  setAttribute(Qt::WA_StyledBackground, true);
  setStyleSheet(QStringLiteral("background: #f3f4f6;"));

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 6, 8, 8);
  layout->setSpacing(6);

  auto* header = new QHBoxLayout();
  header->setContentsMargins(0, 0, 0, 0);
  header->setSpacing(4);
  auto* title = new QLabel(tr("Models"), this);
  title->setStyleSheet(QStringLiteral("color: %1; font-size: 12px; font-weight: 700;")
                           .arg(QLatin1String(kText)));
  header->addWidget(title);
  header->addStretch(1);
  add_button_ = MakePaletteButton(this, GrootPaletteIcon(QStringLiteral("svg/list_add.svg")),
                                  tr("Add custom node"));
  import_button_ = MakePaletteButton(this, GrootPaletteIcon(QStringLiteral("svg/download.svg")),
                                     tr("Import TreeNodesModel"));
  export_button_ = MakePaletteButton(this, GrootPaletteIcon(QStringLiteral("svg/upload.svg")),
                                      tr("Export TreeNodesModel"));
  lock_button_ = MakePaletteButton(this, GrootPaletteIcon(QStringLiteral("svg/lock_open.svg")),
                                     tr("Lock palette"), true);
  header->addWidget(add_button_);
  header->addWidget(import_button_);
  header->addWidget(export_button_);
  header->addWidget(lock_button_);
  layout->addLayout(header);

  filter_edit_ = new QLineEdit(this);
  filter_edit_->setPlaceholderText(tr("Filter"));
  filter_edit_->setClearButtonEnabled(true);
  filter_edit_->setStyleSheet(QStringLiteral(
      "QLineEdit {"
      "  background: #ffffff; color: %1;"
      "  border: 1px solid %2; border-radius: 8px;"
      "  padding: 6px 10px; min-height: 26px;"
      "}"
      "QLineEdit:focus { border-color: %3; }")
                                  .arg(QLatin1String(kText), QLatin1String(kBorder),
                                       QLatin1String(kAccent)));
  layout->addWidget(filter_edit_);

  auto* tree_frame = new QFrame(this);
  tree_frame->setStyleSheet(QStringLiteral(
      "QFrame { background: #ffffff; border: 1px solid %1; border-radius: 8px; }")
                                .arg(QLatin1String(kBorder)));
  auto* tree_layout = new QVBoxLayout(tree_frame);
  tree_layout->setContentsMargins(0, 0, 0, 0);

  tree_ = new BtPaletteTreeWidget(&locked_, tree_frame);
  tree_->setHeaderHidden(true);
  tree_->setRootIsDecorated(true);
  tree_->setDragEnabled(true);
  tree_->setDragDropMode(QAbstractItemView::DragOnly);
  tree_->setSelectionMode(QAbstractItemView::SingleSelection);
  tree_->setContextMenuPolicy(Qt::CustomContextMenu);
  tree_->setStyleSheet(QStringLiteral(
      "QTreeWidget {"
      "  background: transparent; color: %1; border: none; outline: none;"
      "  font-size: 12px;"
      "}"
      "QTreeWidget::item { padding: 4px 6px; min-height: 24px; }"
      "QTreeWidget::item:selected {"
      "  background: #d1fae5; color: %1;"
      "}"
      "QTreeWidget::item:hover:!selected {"
      "  background: rgba(15,23,42,0.04);"
      "}")
                           .arg(QLatin1String(kText)));
  tree_layout->addWidget(tree_);
  layout->addWidget(tree_frame, 1);

  auto* ports_label = new QLabel(tr("Ports"), this);
  ports_label->setStyleSheet(
      QStringLiteral("color: %1; font-size: 11px; font-weight: 700;")
          .arg(QLatin1String(kTextMuted)));
  layout->addWidget(ports_label);

  ports_table_ = new QTableWidget(this);
  ports_table_->setColumnCount(3);
  ports_table_->setHorizontalHeaderLabels({tr("Direction"), tr("Port"), tr("Description")});
  ports_table_->horizontalHeader()->setStretchLastSection(true);
  ports_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  ports_table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  ports_table_->verticalHeader()->setVisible(false);
  ports_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  ports_table_->setSelectionMode(QAbstractItemView::NoSelection);
  ports_table_->setMaximumHeight(140);
  ports_table_->setStyleSheet(QStringLiteral(
      "QTableWidget {"
      "  background: #ffffff; color: %1; border: 1px solid %2;"
      "  border-radius: 8px; gridline-color: %2; font-size: 11px;"
      "}"
      "QTableWidget::item { padding: 3px 6px; }"
      "QHeaderView::section {"
      "  background: %3; color: %4; border: none;"
      "  border-bottom: 1px solid %2; padding: 4px 6px;"
      "  font-size: 10px; font-weight: 700;"
      "}")
                                  .arg(QLatin1String(kText), QLatin1String(kBorder),
                                       QLatin1String(kBg), QLatin1String(kTextMuted)));
  layout->addWidget(ports_table_);

  connect(filter_edit_, &QLineEdit::textChanged, this, &BtNodePalette::applyFilter);
  connect(tree_, &QTreeWidget::itemSelectionChanged, this,
          &BtNodePalette::updatePortsForSelection);
  connect(add_button_, &QToolButton::clicked, this, &BtNodePalette::onAddNodeClicked);
  connect(import_button_, &QToolButton::clicked, this, &BtNodePalette::onImportClicked);
  connect(export_button_, &QToolButton::clicked, this, &BtNodePalette::onExportClicked);
  connect(lock_button_, &QToolButton::toggled, this, &BtNodePalette::onLockToggled);
  connect(tree_, &QTreeWidget::customContextMenuRequested, this, &BtNodePalette::onContextMenu);
  connect(tree_, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem* item, int) {
    if (item == nullptr || !item->data(0, kRegistrationRole).isValid()) {
      return;
    }
    const QString registration_id = item->data(0, kRegistrationRole).toString();
    const BtNodeKind kind =
        static_cast<BtNodeKind>(item->data(0, kKindRole).toInt());
    if (kind == BtNodeKind::kSubTree) {
      emit subtreeOpenRequested(registration_id);
      return;
    }
    emit nodeTypeSelected(registration_id, kind);
  });
}

void BtNodePalette::setModels(const QHash<QString, BtNodeModel>& models) {
  models_ = models;
  rebuildTree();
}

void BtNodePalette::setLocked(bool locked) {
  locked_ = locked;
  if (lock_button_ != nullptr) {
    lock_button_->blockSignals(true);
    lock_button_->setChecked(locked);
    lock_button_->setIcon(locked ? GrootPaletteIcon(QStringLiteral("svg/lock.svg"))
                                 : GrootPaletteIcon(QStringLiteral("svg/lock_open.svg")));
    lock_button_->blockSignals(false);
  }
  tree_->setDragEnabled(!locked);
}

void BtNodePalette::onAddNodeClicked() {
  emit addNodeRequested();
}

void BtNodePalette::onImportClicked() { emit importModelsRequested(); }

void BtNodePalette::onExportClicked() { emit exportModelsRequested(); }

void BtNodePalette::onLockToggled(bool locked) {
  setLocked(locked);
  emit lockChanged(locked);
}

void BtNodePalette::onContextMenu(const QPoint& pos) {
  QTreeWidgetItem* item = tree_->itemAt(pos);
  if (item == nullptr || !item->data(0, kRegistrationRole).isValid()) {
    return;
  }
  const QString registration_id = item->data(0, kRegistrationRole).toString();
  if (locked_ || IsBuiltinModel(registration_id)) {
    return;
  }

  QMenu menu(this);
  QAction* edit_action = menu.addAction(tr("Edit"));
  QAction* remove_action = menu.addAction(tr("Remove"));
  connect(edit_action, &QAction::triggered, this,
          [this, registration_id]() { emit editNodeRequested(registration_id); });
  connect(remove_action, &QAction::triggered, this,
          [this, registration_id]() { emit removeNodeRequested(registration_id); });
  menu.exec(tree_->mapToGlobal(pos));
}

QString BtNodePalette::KindGroupLabel(BtNodeKind kind) {
  switch (kind) {
    case BtNodeKind::kAction:
      return QStringLiteral("Action");
    case BtNodeKind::kCondition:
      return QStringLiteral("Condition");
    case BtNodeKind::kControl:
      return QStringLiteral("Control");
    case BtNodeKind::kDecorator:
      return QStringLiteral("Decorator");
    case BtNodeKind::kSubTree:
      return QStringLiteral("SubTree");
    case BtNodeKind::kRoot:
      return QStringLiteral("Root");
    case BtNodeKind::kUndefined:
      break;
  }
  return QStringLiteral("Other");
}

void BtNodePalette::rebuildTree() {
  tree_->clear();
  const BtNodeKind groups[] = {
      BtNodeKind::kAction,    BtNodeKind::kCondition, BtNodeKind::kControl,
      BtNodeKind::kDecorator, BtNodeKind::kSubTree,
  };

  QHash<BtNodeKind, QTreeWidgetItem*> group_items;
  for (BtNodeKind kind : groups) {
    auto* group = new QTreeWidgetItem(tree_, {KindGroupLabel(kind)});
    group->setFlags(group->flags() & ~Qt::ItemIsDragEnabled);
    group->setExpanded(true);
    group->setIcon(0, BtIconLoader::nodeIcon(QString(), kind, 16));
    group_items.insert(kind, group);
  }

  QStringList ids = models_.keys();
  ids.sort(Qt::CaseInsensitive);
  for (const QString& id : ids) {
    const BtNodeModel& model = models_.value(id);
    QTreeWidgetItem* group = group_items.value(model.kind);
    if (group == nullptr) {
      continue;
    }
    auto* leaf = new QTreeWidgetItem(group, {model.registration_id});
    leaf->setData(0, kRegistrationRole, model.registration_id);
    leaf->setData(0, kKindRole, static_cast<int>(model.kind));
    leaf->setIcon(0, BtIconLoader::nodeIcon(model.registration_id, model.kind, 16));

    QFont font = leaf->font(0);
    font.setPointSize(11);
    const bool is_builtin = IsBuiltinModel(model.registration_id);
    font.setItalic(is_builtin);
    leaf->setFont(0, font);

    if (is_builtin) {
      if (const auto caption =
              BtIconLoader::nodeCaptionColor(model.registration_id, model.kind)) {
        leaf->setForeground(0, *caption);
      } else {
        leaf->setForeground(0, ColorForKind(model.kind).darker(115));
      }
    } else {
      leaf->setForeground(0, QColor(QLatin1String(kGrootCustomBlue)));
    }
  }

  for (int i = tree_->topLevelItemCount() - 1; i >= 0; --i) {
    if (tree_->topLevelItem(i)->childCount() == 0) {
      delete tree_->takeTopLevelItem(i);
    }
  }

  applyFilter(filter_edit_->text());
  updatePortsForSelection();
}

void BtNodePalette::applyFilter(const QString& text) {
  const QString needle = text.trimmed();
  for (int g = 0; g < tree_->topLevelItemCount(); ++g) {
    QTreeWidgetItem* group = tree_->topLevelItem(g);
    bool group_visible = false;
    for (int c = 0; c < group->childCount(); ++c) {
      QTreeWidgetItem* leaf = group->child(c);
      const bool match =
          needle.isEmpty() || leaf->text(0).contains(needle, Qt::CaseInsensitive) ||
          group->text(0).contains(needle, Qt::CaseInsensitive);
      leaf->setHidden(!match);
      if (match) {
        group_visible = true;
      }
    }
    group->setHidden(!group_visible);
    if (group_visible) {
      group->setExpanded(true);
    }
  }
}

void BtNodePalette::updatePortsForSelection() {
  ports_table_->setRowCount(0);
  QTreeWidgetItem* item = tree_->currentItem();
  if (item == nullptr || !item->data(0, kRegistrationRole).isValid()) {
    return;
  }
  const QString registration_id = item->data(0, kRegistrationRole).toString();
  if (!models_.contains(registration_id)) {
    return;
  }
  const QVector<BtPortModel>& ports = models_.value(registration_id).ports;
  ports_table_->setRowCount(ports.size());
  for (int row = 0; row < ports.size(); ++row) {
    const BtPortModel& port = ports.at(row);
    ports_table_->setItem(row, 0, new QTableWidgetItem(PortDirectionLabel(port.direction)));
    ports_table_->setItem(row, 1, new QTableWidgetItem(port.name));
    ports_table_->setItem(row, 2,
                          new QTableWidgetItem(port.description.isEmpty() ? port.type_name
                                                                            : port.description));
  }
}

}  // namespace behavior_tree
}  // namespace autoviz
