/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/common/display_property.hpp"
#include "autoviz/ui/display_tree_delegate.hpp"

#include <algorithm>

#include <QApplication>
#include <QCheckBox>
#include <QColorDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QStyle>
#include <QTreeWidget>

namespace autoviz {
namespace {

constexpr int kColorSwatchWidth = 28;
constexpr int kColorSwatchHeight = 16;

QColor ColorFromPropertyText(const QString& text) {
  return common::ParseColorProperty(text.toStdString(), QColor(128, 128, 128));
}

QString ColorToPropertyText(const QColor& color) {
  return QString::fromStdString(common::FormatColorProperty(color));
}

QString FormatColorDisplayText(const QColor& color) {
  return QStringLiteral("%1; %2; %3")
      .arg(color.red())
      .arg(color.green())
      .arg(color.blue());
}

QRect ColorSwatchRect(const QRect& cell_rect) {
  const int height = std::min(kColorSwatchHeight, cell_rect.height() - 6);
  return QRect(cell_rect.left() + 4,
               cell_rect.top() + (cell_rect.height() - height) / 2,
               kColorSwatchWidth, height);
}

}  // namespace

QModelIndex DisplayTreeNameColumnIndex(const QModelIndex& index) {
  if (!index.isValid()) {
    return index;
  }
  if (index.column() == kDisplayTreeColName) {
    return index;
  }
  return index.sibling(index.row(), kDisplayTreeColName);
}

DisplayTreeItemKind ItemKindFromIndex(const QModelIndex& index) {
  return static_cast<DisplayTreeItemKind>(
      DisplayTreeNameColumnIndex(index).data(kDisplayTreeRoleKind).toInt());
}

QString PropertyKeyFromIndex(const QModelIndex& index) {
  return DisplayTreeNameColumnIndex(index)
      .data(kDisplayTreeRolePropertyKey)
      .toString();
}

int PropertyKindFromIndex(const QModelIndex& index) {
  const QModelIndex meta = DisplayTreeNameColumnIndex(index);
  return meta.data(kDisplayTreeRolePropertyKind).toInt();
}

bool IsColorTreeItem(const DisplayTreeItemKind kind, const QString& property_key,
                     const int property_kind) {
  if (property_kind ==
      static_cast<int>(common::DisplayPropertyKind::kColor)) {
    return true;
  }
  if (kind == DisplayTreeItemKind::kGlobalBackgroundColor) {
    return true;
  }
  if (kind != DisplayTreeItemKind::kDisplayProperty) {
    return false;
  }
  if (property_key == QLatin1String("color_transform") ||
      property_key == QLatin1String("color_scheme") ||
      property_key == QLatin1String("color_channel")) {
    return false;
  }
  return property_key == QLatin1String("color") ||
         property_key.endsWith(QLatin1String("_color"), Qt::CaseInsensitive);
}

DisplayNameTreeDelegate::DisplayNameTreeDelegate(QObject* parent)
    : QStyledItemDelegate(parent) {}

void DisplayNameTreeDelegate::paint(QPainter* painter,
                                    const QStyleOptionViewItem& option,
                                    const QModelIndex& index) const {
  QStyleOptionViewItem opt = option;
  initStyleOption(&opt, index);

  const auto kind = static_cast<DisplayTreeItemKind>(
      index.data(kDisplayTreeRoleKind).toInt());
  const bool draggable = kind == DisplayTreeItemKind::kDisplay ||
                         kind == DisplayTreeItemKind::kDisplayChild;

  if (draggable) {
    painter->save();
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setPen(Qt::NoPen);
    painter->setBrush(QColor(120, 120, 120));
    const int cx = opt.rect.left() + 5;
    const int cy = opt.rect.center().y();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 2; ++col) {
        painter->drawEllipse(QPointF(cx + col * 4.0, cy - 4.0 + row * 4.0), 1.6,
                             1.6);
      }
    }
    painter->restore();
    opt.rect = opt.rect.adjusted(12, 0, 0, 0);
  }

  QStyledItemDelegate::paint(painter, opt, index);
}

QSize DisplayNameTreeDelegate::sizeHint(const QStyleOptionViewItem& option,
                                        const QModelIndex& index) const {
  QSize size = QStyledItemDelegate::sizeHint(option, index);
  const auto kind = static_cast<DisplayTreeItemKind>(
      index.data(kDisplayTreeRoleKind).toInt());
  if (kind == DisplayTreeItemKind::kDisplay ||
      kind == DisplayTreeItemKind::kDisplayChild) {
    size.rwidth() += 12;
  }
  return size;
}

DisplayTreeDelegate::DisplayTreeDelegate(QObject* parent)
    : QStyledItemDelegate(parent) {}

void DisplayTreeDelegate::paint(QPainter* painter,
                                const QStyleOptionViewItem& option,
                                const QModelIndex& index) const {
  if (index.column() != kDisplayTreeColValue) {
    QStyledItemDelegate::paint(painter, option, index);
    return;
  }

  const auto kind = ItemKindFromIndex(index);
  const QString property_key = PropertyKeyFromIndex(index);
  const int property_kind = PropertyKindFromIndex(index);
  if (!IsColorTreeItem(kind, property_key, property_kind)) {
    QStyledItemDelegate::paint(painter, option, index);
    return;
  }

  QStyleOptionViewItem opt = option;
  initStyleOption(&opt, index);
  const QString color_text = opt.text;
  opt.text.clear();
  opt.icon = QIcon();
  opt.features.setFlag(QStyleOptionViewItem::HasDisplay, false);

  if (QStyle* style = opt.widget ? opt.widget->style() : QApplication::style()) {
    style->drawControl(QStyle::CE_ItemViewItem, &opt, painter, opt.widget);
  }

  const QColor color = ColorFromPropertyText(color_text);
  const QRect swatch_rect = ColorSwatchRect(opt.rect);
  painter->save();
  painter->setPen(QPen(QColor(120, 120, 120)));
  painter->setBrush(color);
  painter->drawRect(swatch_rect);

  const QString rgb_text = FormatColorDisplayText(color);
  QRect text_rect = opt.rect;
  text_rect.setLeft(swatch_rect.right() + 6);
  painter->setPen(opt.state.testFlag(QStyle::State_Selected)
                      ? opt.palette.color(QPalette::HighlightedText)
                      : opt.palette.color(QPalette::Text));
  painter->drawText(text_rect, Qt::AlignVCenter | Qt::AlignLeft, rgb_text);
  painter->restore();
}

void DisplayTreeDelegate::setChannels(const QStringList& channels) {
  channels_ = channels;
}

QWidget* DisplayTreeDelegate::createEditor(QWidget* parent,
                                           const QStyleOptionViewItem& option,
                                           const QModelIndex& index) const {
  if (index.column() != kDisplayTreeColValue) {
    return QStyledItemDelegate::createEditor(parent, option, index);
  }

  const auto kind = ItemKindFromIndex(index);
  const QString property_key = PropertyKeyFromIndex(index);
  const int property_kind = PropertyKindFromIndex(index);
  const QString current = index.data(Qt::EditRole).toString();

  if (kind == DisplayTreeItemKind::kGlobalFixedFrame ||
      kind == DisplayTreeItemKind::kDisplayChannel) {
    auto* combo = new QComboBox(parent);
    combo->setEditable(true);
    combo->addItems(channels_);
    combo->setCurrentText(current);
    return combo;
  }

  if (IsColorTreeItem(kind, property_key, property_kind)) {
    return nullptr;
  }

  if (kind == DisplayTreeItemKind::kGlobalFrameRate) {
    auto* spin = new QDoubleSpinBox(parent);
    spin->setDecimals(0);
    spin->setRange(1, 1000);
    spin->setValue(current.toDouble());
    return spin;
  }

  if (kind == DisplayTreeItemKind::kDisplayProperty) {
    const QString default_lower = index.data(Qt::UserRole + 10).toString();
    if (default_lower == QLatin1String("true") ||
        default_lower == QLatin1String("false")) {
      auto* check = new QCheckBox(parent);
      check->setChecked(current == QLatin1String("true"));
      return check;
    }

    const QString options = index.data(Qt::UserRole + 11).toString();
    if (!options.isEmpty()) {
      auto* combo = new QComboBox(parent);
      for (const QString& option_name : options.split(QLatin1Char('\n'))) {
        combo->addItem(option_name);
      }
      const int found = combo->findText(current);
      if (found >= 0) {
        combo->setCurrentIndex(found);
      }
      return combo;
    }

    if (property_key.contains(QLatin1String("path"), Qt::CaseInsensitive) ||
        property_key.contains(QLatin1String("urdf"), Qt::CaseInsensitive)) {
      auto* row = new QWidget(parent);
      auto* layout = new QHBoxLayout(row);
      layout->setContentsMargins(0, 0, 0, 0);
      auto* edit = new QLineEdit(row);
      edit->setText(current);
      auto* browse = new QPushButton(QObject::tr("..."), row);
      browse->setFixedWidth(28);
      layout->addWidget(edit, 1);
      layout->addWidget(browse);
      row->setProperty("path_edit", QVariant::fromValue<QObject*>(edit));
      QObject::connect(browse, &QPushButton::clicked, [edit]() {
        const QString picked = QFileDialog::getOpenFileName(
            edit->window(), QObject::tr("Select File"), edit->text());
        if (!picked.isEmpty()) {
          edit->setText(picked);
          emit edit->editingFinished();
        }
      });
      return row;
    }

    bool ok = false;
    current.toFloat(&ok);
    if (ok) {
      auto* spin = new QDoubleSpinBox(parent);
      spin->setDecimals(4);
      spin->setRange(-1e6, 1e6);
      spin->setSingleStep(0.1);
      spin->setValue(current.toFloat());
      return spin;
    }
  }

  auto* edit = new QLineEdit(parent);
  edit->setText(current);
  return edit;
}

void DisplayTreeDelegate::setEditorData(QWidget* editor,
                                        const QModelIndex& index) const {
  if (index.column() != kDisplayTreeColValue) {
    QStyledItemDelegate::setEditorData(editor, index);
    return;
  }

  const QString current = index.data(Qt::EditRole).toString();
  if (auto* combo = qobject_cast<QComboBox*>(editor)) {
    combo->setCurrentText(current);
    return;
  }
  if (auto* spin = qobject_cast<QDoubleSpinBox*>(editor)) {
    spin->setValue(current.toDouble());
    return;
  }
  if (auto* check = qobject_cast<QCheckBox*>(editor)) {
    check->setChecked(current == QLatin1String("true"));
    return;
  }
  if (auto* edit = editor->findChild<QLineEdit*>()) {
    edit->setText(current);
    return;
  }
  if (auto* line = qobject_cast<QLineEdit*>(editor)) {
    line->setText(current);
  }
}

void DisplayTreeDelegate::setModelData(QWidget* editor, QAbstractItemModel* model,
                                       const QModelIndex& index) const {
  if (index.column() != kDisplayTreeColValue) {
    QStyledItemDelegate::setModelData(editor, model, index);
    return;
  }

  QString value;
  if (auto* combo = qobject_cast<QComboBox*>(editor)) {
    value = combo->currentText();
  } else if (auto* spin = qobject_cast<QDoubleSpinBox*>(editor)) {
    const auto kind = static_cast<DisplayTreeItemKind>(
        index.data(kDisplayTreeRoleKind).toInt());
    if (kind == DisplayTreeItemKind::kGlobalFrameRate) {
      value = QString::number(static_cast<int>(spin->value()));
    } else {
      value = QString::number(spin->value(), 'g', 8);
    }
  } else if (auto* check = qobject_cast<QCheckBox*>(editor)) {
    value = check->isChecked() ? QStringLiteral("true") : QStringLiteral("false");
  } else if (auto* edit = editor->findChild<QLineEdit*>()) {
    value = edit->text();
  } else if (auto* line = qobject_cast<QLineEdit*>(editor)) {
    value = line->text();
  } else {
    QStyledItemDelegate::setModelData(editor, model, index);
    return;
  }
  model->setData(index, value, Qt::EditRole);
}

void DisplayTreeDelegate::updateEditorGeometry(QWidget* editor,
                                               const QStyleOptionViewItem& option,
                                               const QModelIndex& index) const {
  if (index.column() == kDisplayTreeColValue) {
    editor->setGeometry(option.rect);
    return;
  }
  QStyledItemDelegate::updateEditorGeometry(editor, option, index);
}

bool DisplayTreeDelegate::editorEvent(QEvent* event, QAbstractItemModel* model,
                                      const QStyleOptionViewItem& option,
                                      const QModelIndex& index) {
  if (!index.isValid() || index.column() != kDisplayTreeColValue) {
    return QStyledItemDelegate::editorEvent(event, model, option, index);
  }

  const auto kind = ItemKindFromIndex(index);
  const QString property_key = PropertyKeyFromIndex(index);
  const int property_kind = PropertyKindFromIndex(index);
  if (!IsColorTreeItem(kind, property_key, property_kind)) {
    return QStyledItemDelegate::editorEvent(event, model, option, index);
  }

  if (event->type() == QEvent::MouseButtonRelease) {
    const auto* mouse_event = static_cast<const QMouseEvent*>(event);
    if (mouse_event->button() != Qt::LeftButton) {
      return true;
    }

    auto* tree = qobject_cast<QTreeWidget*>(const_cast<QWidget*>(option.widget));
    QTreeWidgetItem* item =
        tree != nullptr ? tree->itemFromIndex(index) : nullptr;
    const QColor current =
        ColorFromPropertyText(index.data(Qt::EditRole).toString());
    QWidget* parent_window = tree != nullptr ? tree->window() : nullptr;
    const QColor picked =
        QColorDialog::getColor(current, parent_window, tr("Pick Color"));
    if (picked.isValid()) {
      const QString value = ColorToPropertyText(picked);
      if (item != nullptr) {
        item->setText(kDisplayTreeColValue, value);
      } else {
        model->setData(index, value, Qt::EditRole);
      }
    }
    return true;
  }

  if (event->type() == QEvent::MouseButtonDblClick) {
    return true;
  }

  return true;
}

}  // namespace autoviz
