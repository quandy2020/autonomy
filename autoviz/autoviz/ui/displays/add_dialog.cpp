/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/displays/add_dialog.hpp"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFont>
#include <QFontMetrics>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPalette>
#include <QPushButton>
#include <QStackedWidget>
#include <QStyle>
#include <QStyleOptionViewItem>
#include <QStyledItemDelegate>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QVBoxLayout>
#include <algorithm>
#include <map>
#include <vector>

#include "autoviz/common/display_catalog.hpp"
#include "autoviz/common/display_factory.hpp"
#include "autoviz/common/interaction/channel_manager.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/common/widget/input/segmented_control.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

constexpr int kTypeRole = Qt::UserRole;
constexpr int kChannelRole = Qt::UserRole + 1;
constexpr int kCardInset = 12;
constexpr int kCardRadius = 18;

/** One continuous rounded selection/hover across indentation + item text. */
class ContinuousRowDelegate : public QStyledItemDelegate {
 public:
  using QStyledItemDelegate::QStyledItemDelegate;

  QSize sizeHint(const QStyleOptionViewItem& option,
                 const QModelIndex& index) const override {
    QSize size = QStyledItemDelegate::sizeHint(option, index);
    size.setHeight(qMax(28, size.height()));
    return size;
  }

  void paint(QPainter* painter, const QStyleOptionViewItem& option,
             const QModelIndex& index) const override {
    QStyleOptionViewItem opt(option);
    initStyleOption(&opt, index);

    const bool selected = opt.state.testFlag(QStyle::State_Selected);
    const bool hovered = opt.state.testFlag(QStyle::State_MouseOver);

    QRect row = opt.rect;
    if (const auto* view = qobject_cast<const QAbstractItemView*>(opt.widget)) {
      row = QRect(2, opt.rect.y(), view->viewport()->width() - 4,
                  opt.rect.height());
    }

    if (selected || hovered) {
      painter->save();
      painter->setRenderHint(QPainter::Antialiasing, true);
      QPainterPath path;
      path.addRoundedRect(QRectF(row).adjusted(0.5, 1.0, -0.5, -1.0), 8.0, 8.0);
      if (selected) {
        QLinearGradient fill(row.topLeft(), row.topRight());
        fill.setColorAt(0.0, glass::ShellSelectionFill(78));
        QColor end = glass::Shell().accent_soft;
        end.setAlpha(62);
        fill.setColorAt(1.0, end);
        painter->fillPath(path, fill);
      } else {
        painter->fillPath(path, glass::ShellHoverFill());
      }
      painter->restore();
    }

    const QString type = index.data(kTypeRole).toString();
    const QString channel = index.data(kChannelRole).toString();
    // Channel tab flat rows: one line = path (elide) + type badge on the right.
    if (!type.isEmpty() && !channel.isEmpty() && index.column() == 0) {
      painter->save();
      painter->setRenderHint(QPainter::Antialiasing, true);
      painter->setRenderHint(QPainter::TextAntialiasing, true);

      QRect content = row.adjusted(8, 0, -10, 0);
      const int icon = opt.decorationSize.width() > 0 ? opt.decorationSize.width()
                                                      : 16;
      QRect icon_rect(content.left(), content.center().y() - icon / 2, icon,
                      icon);
      if (!opt.icon.isNull()) {
        opt.icon.paint(painter, icon_rect, Qt::AlignCenter,
                       selected ? QIcon::Selected : QIcon::Normal,
                       QIcon::On);
      }
      content.setLeft(icon_rect.right() + 8);

      QFont type_font = opt.font;
      type_font.setWeight(QFont::DemiBold);
      type_font.setPointSizeF(qMax(10.0, type_font.pointSizeF() - 0.5));
      const QFontMetrics type_metrics(type_font);
      const int type_width = type_metrics.horizontalAdvance(type) + 2;
      QRect type_rect = content;
      type_rect.setLeft(qMax(content.left() + 40, content.right() - type_width));
      type_rect.setWidth(content.right() - type_rect.left() + 1);

      QRect path_rect = content;
      path_rect.setRight(type_rect.left() - 10);

      QFont path_font = opt.font;
      if (selected) {
        path_font.setWeight(QFont::DemiBold);
      }
      const QColor ink =
          selected ? glass::ShellSelectionInk() : glass::Shell().text;
      const QColor muted = glass::Shell().secondary_text;
      painter->setFont(path_font);
      painter->setPen(ink);
      painter->drawText(
          path_rect, Qt::AlignVCenter | Qt::AlignLeft,
          QFontMetrics(path_font).elidedText(channel, Qt::ElideMiddle,
                                             path_rect.width()));
      painter->setFont(type_font);
      painter->setPen(selected ? glass::Shell().accent : muted);
      painter->drawText(type_rect, Qt::AlignVCenter | Qt::AlignRight, type);
      painter->restore();
      return;
    }

    // Suppress native/QSS selection fill; keep icon + text rendering.
    opt.state.setFlag(QStyle::State_Selected, false);
    opt.state.setFlag(QStyle::State_MouseOver, false);
    opt.backgroundBrush = Qt::NoBrush;
    if (selected) {
      const QColor ink = glass::ShellSelectionInk();
      opt.palette.setColor(QPalette::Text, ink);
      opt.palette.setColor(QPalette::WindowText, ink);
      opt.font.setWeight(QFont::DemiBold);
    }
    QStyledItemDelegate::paint(painter, opt, index);
  }
};

class GlassWell : public QWidget {
 public:
  explicit GlassWell(QWidget* parent = nullptr) : QWidget(parent) {
    setAttribute(Qt::WA_TranslucentBackground, true);
    setAutoFillBackground(false);
  }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const QRectF card = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    QPainterPath path;
    path.addRoundedRect(card, 14.0, 14.0);

    painter.fillPath(path, QColor(255, 255, 255));
    painter.setPen(QPen(QColor(198, 198, 200), 1.0));
    painter.drawPath(path);
  }
};

class GlassFooterBar : public QWidget {
 public:
  explicit GlassFooterBar(QWidget* parent = nullptr) : QWidget(parent) {
    setAttribute(Qt::WA_TranslucentBackground, true);
    setAutoFillBackground(false);
  }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const QRectF card = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    QPainterPath path;
    path.addRoundedRect(card, 16.0, 16.0);

    painter.fillPath(path, glass::Shell().glass_fill_deep);
    painter.setPen(QPen(glass::Shell().glass_rim, 1.0));
    painter.drawPath(path);
  }
};

AddDisplaySelection SelectionFromDisplayItem(QTreeWidgetItem* item) {
  AddDisplaySelection selection;
  if (item == nullptr || item->parent() == nullptr) {
    return selection;
  }
  selection.type = item->data(0, kTypeRole).toString();
  selection.display_name = item->text(0);
  const auto config =
      common::DisplayFactory::defaultForType(selection.type.toStdString());
  selection.channel = QString::fromStdString(config.channel);
  return selection;
}

AddDisplaySelection SelectionFromTopicItem(QTreeWidgetItem* item) {
  AddDisplaySelection selection;
  if (item == nullptr || !item->data(0, kTypeRole).isValid()) {
    return selection;
  }
  selection.type = item->data(0, kTypeRole).toString();
  selection.channel = item->data(0, kChannelRole).toString();
  selection.display_name = selection.type;
  return selection;
}

QString SuggestDisplayName(const QString& type, const QString& channel,
                           const QStringList& disallowed) {
  if (type.isEmpty()) {
    return {};
  }
  QString base = type;
  if (!channel.isEmpty()) {
    const QString leaf = channel.section(QLatin1Char('/'), -1);
    if (!leaf.isEmpty() && leaf.compare(type, Qt::CaseInsensitive) != 0) {
      base = leaf + QLatin1Char(' ') + type;
    }
  }
  if (!disallowed.contains(base)) {
    return base;
  }
  for (int suffix = 2; suffix < 1000; ++suffix) {
    const QString candidate = base + QLatin1Char(' ') + QString::number(suffix);
    if (!disallowed.contains(candidate)) {
      return candidate;
    }
  }
  return base + QStringLiteral(" copy");
}

void PopulateDisplayTypeTree(QTreeWidget* tree) {
  tree->clear();
  const QIcon package_icon =
      IconLoader::load(QStringLiteral(":/autoviz/icons/default/package.svg"));
  std::map<QString, QTreeWidgetItem*> package_items;
  for (const common::DisplayTypeInfo& info :
       common::DisplayCatalog::allTypes()) {
    const QString package_q = QString::fromStdString(info.package);
    QTreeWidgetItem* package_item = nullptr;
    const auto found = package_items.find(package_q);
    if (found == package_items.end()) {
      package_item = new QTreeWidgetItem(tree);
      package_item->setText(0, package_q);
      package_item->setIcon(0, package_icon);
      package_item->setExpanded(true);
      package_items.emplace(package_q, package_item);
    } else {
      package_item = found->second;
    }

    auto* class_item = new QTreeWidgetItem(package_item);
    const QString type_q = QString::fromStdString(info.type);
    class_item->setIcon(0, IconLoader::displayIcon(type_q));
    class_item->setText(0, type_q);
    class_item->setData(0, kTypeRole, type_q);
    class_item->setToolTip(0, QString::fromStdString(info.description));
  }
}

void PopulateTopicTree(QTreeWidget* tree,
                       const common::VisualizationManager& manager) {
  tree->clear();
  struct Row {
    QString channel;
    QString type;
    QString tip;
    bool visualizable = false;
  };
  std::vector<Row> rows;
  rows.reserve(manager.channels().size());
  for (const integration::ChannelInfo& channel : manager.channels()) {
    const QString channel_q = QString::fromStdString(channel.channel_name);
    const std::vector<std::string> display_types =
        common::DisplayCatalog::typesForChannel(channel.channel_name,
                                                channel.message_type);
    if (display_types.empty()) {
      rows.push_back(Row{channel_q, {}, {}, false});
      continue;
    }
    for (const std::string& type : display_types) {
      const common::DisplayTypeInfo info =
          common::DisplayCatalog::infoForType(type);
      rows.push_back(Row{channel_q, QString::fromStdString(type),
                         QString::fromStdString(info.description), true});
    }
  }
  std::sort(rows.begin(), rows.end(), [](const Row& a, const Row& b) {
    const int by_channel = QString::compare(a.channel, b.channel, Qt::CaseInsensitive);
    if (by_channel != 0) {
      return by_channel < 0;
    }
    return QString::compare(a.type, b.type, Qt::CaseInsensitive) < 0;
  });

  for (const Row& entry : rows) {
    auto* row = new QTreeWidgetItem(tree);
    row->setData(0, kChannelRole, entry.channel);
    row->setToolTip(0, entry.visualizable
                           ? (entry.channel + QStringLiteral(" — ") + entry.type +
                              (entry.tip.isEmpty()
                                   ? QString()
                                   : (QStringLiteral("\n") + entry.tip)))
                           : entry.channel);
    if (!entry.visualizable) {
      row->setText(0, entry.channel);
      row->setDisabled(true);
      continue;
    }
    // Text kept for accessibility / fallback; delegate paints one-line path+type.
    row->setText(0, entry.channel + QStringLiteral("  ·  ") + entry.type);
    row->setIcon(0, IconLoader::displayIcon(entry.type));
    row->setData(0, kTypeRole, entry.type);
  }
}

void ApplyFrostedTree(QTreeWidget* tree, bool flat_list) {
  if (tree == nullptr) {
    return;
  }
  tree->setAttribute(Qt::WA_TranslucentBackground, true);
  tree->setAutoFillBackground(false);
  if (tree->viewport() != nullptr) {
    tree->viewport()->setAttribute(Qt::WA_TranslucentBackground, true);
    tree->viewport()->setAutoFillBackground(false);
  }
  tree->setStyleSheet(style::sheet(QStringLiteral("add_display/frosted_tree")));
  tree->setFocusPolicy(Qt::StrongFocus);
  tree->setAttribute(Qt::WA_MacShowFocusRect, false);
  tree->setRootIsDecorated(!flat_list);
  tree->setItemsExpandable(!flat_list);
  tree->setIndentation(flat_list ? 4 : 12);
  tree->setUniformRowHeights(true);
  tree->setAnimated(false);
  tree->setAllColumnsShowFocus(false);
  tree->setTextElideMode(Qt::ElideMiddle);
  tree->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  tree->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  // Kill native selection strip (branch vs item) — only the delegate paints.
  QPalette pal = tree->palette();
  pal.setBrush(QPalette::Highlight, Qt::transparent);
  pal.setBrush(QPalette::HighlightedText, glass::ShellSelectionInk());
  pal.setBrush(QPalette::Inactive, QPalette::Highlight, Qt::transparent);
  pal.setBrush(QPalette::Inactive, QPalette::HighlightedText,
               glass::ShellSelectionInk());
  tree->setPalette(pal);

  tree->setItemDelegate(new ContinuousRowDelegate(tree));
}

}  // namespace

AddDisplayDialog::AddDisplayDialog(
    std::shared_ptr<common::VisualizationManager> manager,
    const QStringList& disallowed_display_names, QWidget* parent)
    : QDialog(parent),
      manager_(std::move(manager)),
      disallowed_display_names_(disallowed_display_names) {
  setObjectName(QStringLiteral("AddDisplayDialog"));
  setWindowTitle(tr("Create visualization"));
  setModal(true);
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setAttribute(Qt::WA_TranslucentBackground, true);
  setAttribute(Qt::WA_StyledBackground, false);
  setAutoFillBackground(false);
  setMinimumSize(460, 720);
  resize(500, 760);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(kCardInset + 6, kCardInset + 6, kCardInset + 6,
                           kCardInset + 6);
  root->setSpacing(10);

  segment_control_ = new widget::SegmentedControl(this);
  segment_control_->setObjectName(QStringLiteral("CreateVizSegmentControl"));
  segment_control_->setAccessibleName(tr("Display source"));
  segment_control_->SetTone(style::Tone::Frost);
  segment_control_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  segment_control_->setFixedHeight(46);
  segment_control_->AddSegment(tr("Type"));
  segment_control_->AddSegment(tr("Channel"));
  connect(segment_control_, &widget::SegmentedControl::SegmentClicked, this,
          &AddDisplayDialog::setActiveTab);

  auto* well = new GlassWell(this);
  auto* well_layout = new QVBoxLayout(well);
  well_layout->setContentsMargins(4, 6, 4, 6);
  well_layout->setSpacing(0);

  stack_ = new QStackedWidget(well);
  stack_->setAttribute(Qt::WA_TranslucentBackground, true);
  stack_->setStyleSheet(style::mark(style::Mark::Clear));

  display_tree_ = new QTreeWidget(stack_);
  display_tree_->setObjectName(
      QStringLiteral("AddDisplayDialog/DisplayTypeTree"));
  display_tree_->setHeaderHidden(true);
  display_tree_->setIconSize(QSize(16, 16));
  display_tree_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  PopulateDisplayTypeTree(display_tree_);
  ApplyFrostedTree(display_tree_, /*flat_list=*/false);
  display_tree_->setIndentation(10);
  stack_->addWidget(display_tree_);

  auto* topic_page = new QWidget(stack_);
  topic_page->setAttribute(Qt::WA_TranslucentBackground, true);
  topic_page->setStyleSheet(style::mark(style::Mark::Clear));
  topic_tree_ = new QTreeWidget(topic_page);
  topic_tree_->setObjectName(QStringLiteral("AddDisplayDialog/TopicTree"));
  topic_tree_->setHeaderHidden(true);
  topic_tree_->setIconSize(QSize(16, 16));
  topic_tree_->header()->setStretchLastSection(true);
  topic_tree_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  ApplyFrostedTree(topic_tree_, /*flat_list=*/true);
  show_unvisualizable_topics_ =
      new QCheckBox(tr("Show unvisualizable topics"), topic_page);
  show_unvisualizable_topics_->setStyleSheet(
      style::sheet(QStringLiteral("add_display/check")));
  auto* topic_layout = new QVBoxLayout(topic_page);
  topic_layout->setContentsMargins(0, 0, 0, 0);
  topic_layout->setSpacing(8);
  topic_layout->addWidget(topic_tree_, 1);
  topic_layout->addWidget(show_unvisualizable_topics_);
  stack_->addWidget(topic_page);
  well_layout->addWidget(stack_, 1);
  refreshTopicTree();

  auto* footer = new GlassFooterBar(this);
  auto* footer_layout = new QVBoxLayout(footer);
  footer_layout->setContentsMargins(14, 12, 14, 14);
  footer_layout->setSpacing(8);

  auto* name_caption = new QLabel(tr("Name"), footer);
  name_caption->setStyleSheet(
      style::sheet(QStringLiteral("add_display/name_caption")));

  name_editor_ = new QLineEdit(footer);
  name_editor_->setObjectName(
      QStringLiteral("AddDisplayDialog/DisplayNameEdit"));
  name_editor_->setPlaceholderText(tr("Name this visualization"));
  name_editor_->setStyleSheet(
      style::sheet(QStringLiteral("add_display/line_edit")));
  name_editor_->setClearButtonEnabled(true);
  // Clear button calls clear() (programmatic) — it does not emit textEdited.
  // Mark as user-edited so updateUi() won't immediately restore the name.
  connect(name_editor_, &QLineEdit::textChanged, this,
          [this](const QString& text) {
            if (text.isEmpty()) {
              user_edited_name_ = true;
            }
            if (button_box_ != nullptr) {
              if (QPushButton* ok = button_box_->button(QDialogButtonBox::Ok)) {
                ok->setEnabled(isValid());
              }
            }
          });

  auto* action_row = new QWidget(footer);
  action_row->setAttribute(Qt::WA_TranslucentBackground, true);
  auto* action_layout = new QHBoxLayout(action_row);
  action_layout->setContentsMargins(0, 6, 0, 0);
  action_layout->setSpacing(12);

  // Own the row layout: equal-width Cancel | Confirm aligned to the name field.
  // QDialogButtonBox keeps role/signals; its own layout is bypassed.
  button_box_ =
      new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                           Qt::Horizontal, action_row);
  button_box_->setObjectName(QStringLiteral("AddDisplayDialog/ButtonBox"));
  button_box_->setCenterButtons(false);
  QPushButton* ok = button_box_->button(QDialogButtonBox::Ok);
  QPushButton* cancel = button_box_->button(QDialogButtonBox::Cancel);
  if (ok != nullptr) {
    ok->setText(tr("Confirm"));
    ok->setIcon(QIcon());
    ok->setCursor(Qt::PointingHandCursor);
    ok->setAutoDefault(true);
    ok->setDefault(true);
    ok->setFlat(false);
    ok->setFocusPolicy(Qt::StrongFocus);
    ok->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    ok->setStyleSheet(
        style::sheet(QStringLiteral("add_display/primary_button")));
  }
  if (cancel != nullptr) {
    cancel->setText(tr("Cancel"));
    cancel->setIcon(QIcon());
    cancel->setCursor(Qt::PointingHandCursor);
    cancel->setAutoDefault(false);
    cancel->setDefault(false);
    cancel->setFlat(false);
    cancel->setFocusPolicy(Qt::StrongFocus);
    cancel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    cancel->setStyleSheet(
        style::sheet(QStringLiteral("add_display/ghost_button")));
  }
  button_box_->hide();
  if (cancel != nullptr) {
    action_layout->addWidget(cancel, 1);
  }
  if (ok != nullptr) {
    action_layout->addWidget(ok, 1);
  }

  footer_layout->addWidget(name_caption);
  footer_layout->addWidget(name_editor_);
  footer_layout->addWidget(action_row);

  root->addWidget(segment_control_);
  root->addWidget(well, 1);
  root->addWidget(footer);

  connect(display_tree_, &QTreeWidget::currentItemChanged, this,
          [this](QTreeWidgetItem* current, QTreeWidgetItem* previous) {
            Q_UNUSED(previous);
            display_tab_selection_ = SelectionFromDisplayItem(current);
            updateUi();
          });
  connect(topic_tree_, &QTreeWidget::currentItemChanged, this,
          [this](QTreeWidgetItem* current, QTreeWidgetItem* previous) {
            Q_UNUSED(previous);
            topic_tab_selection_ = SelectionFromTopicItem(current);
            updateUi();
          });
  connect(display_tree_, &QTreeWidget::itemActivated, this,
          &AddDisplayDialog::accept);
  connect(topic_tree_, &QTreeWidget::itemActivated, this,
          &AddDisplayDialog::accept);
  connect(show_unvisualizable_topics_, &QCheckBox::stateChanged, this,
          [this](int) { applyTopicVisibility(); });
  connect(button_box_, &QDialogButtonBox::accepted, this,
          &AddDisplayDialog::accept);
  connect(button_box_, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(name_editor_, &QLineEdit::textEdited, this, [this]() {
    user_edited_name_ = true;
    onNameChanged();
  });

  setActiveTab(display_tab_);
  button_box_->button(QDialogButtonBox::Ok)->setEnabled(false);
}

void AddDisplayDialog::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QRectF outer =
      QRectF(rect()).adjusted(kCardInset, kCardInset, -kCardInset, -kCardInset);
  QPainterPath card;
  card.addRoundedRect(outer, kCardRadius, kCardRadius);

  // Soft milky drop shadow (reads as floating glass over parent UI).
  for (int i = 8; i >= 1; --i) {
    QPainterPath glow;
    glow.addRoundedRect(outer.adjusted(-i * 0.7, -i * 0.35, i * 0.7, i * 0.9),
                        kCardRadius + i * 0.4, kCardRadius + i * 0.4);
    painter.fillPath(glow, QColor(15, 23, 42, 4 + (8 - i)));
  }

  const glass::ShellTokens shell = glass::Shell();
  QColor plate = shell.glass_fill_deep;
  plate.setAlpha(255);
  painter.fillPath(card, plate);
  painter.setPen(QPen(shell.glass_rim, 1.0));
  painter.drawPath(card);
}

void AddDisplayDialog::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    // Drag from empty chrome / segment track margins (not tab buttons).
    QWidget* hit = childAt(event->pos());
    const bool on_segment_button = segment_control_ != nullptr &&
                                   hit != nullptr && hit != segment_control_ &&
                                   segment_control_->isAncestorOf(hit);
    const bool on_segment_track = hit == segment_control_;
    if (!on_segment_button &&
        (hit == nullptr || hit == this || on_segment_track)) {
      dragging_ = true;
      drag_offset_ =
          event->globalPosition().toPoint() - frameGeometry().topLeft();
      event->accept();
      return;
    }
  }
  QDialog::mousePressEvent(event);
}

void AddDisplayDialog::mouseMoveEvent(QMouseEvent* event) {
  if (dragging_ && (event->buttons() & Qt::LeftButton)) {
    move(event->globalPosition().toPoint() - drag_offset_);
    event->accept();
    return;
  }
  QDialog::mouseMoveEvent(event);
}

void AddDisplayDialog::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    dragging_ = false;
  }
  QDialog::mouseReleaseEvent(event);
}

void AddDisplayDialog::setActiveTab(int index) {
  if (segment_control_ != nullptr) {
    segment_control_->SetCurrentIndex(index);
  }
  if (stack_ != nullptr) {
    stack_->setCurrentIndex(index);
  }
  onTabChanged(index);
}

QSize AddDisplayDialog::sizeHint() const { return {500, 760}; }

void AddDisplayDialog::applyTopicVisibility() {
  if (topic_tree_ == nullptr || show_unvisualizable_topics_ == nullptr) {
    return;
  }
  const bool hide_unvisualizable =
      show_unvisualizable_topics_->checkState() == Qt::Unchecked;
  for (QTreeWidgetItemIterator it(topic_tree_); *it; ++it) {
    QTreeWidgetItem* item = *it;
    item->setHidden(hide_unvisualizable && item->isDisabled());
  }
}

void AddDisplayDialog::refreshTopicTree() {
  if (manager_ == nullptr || topic_tree_ == nullptr ||
      show_unvisualizable_topics_ == nullptr) {
    return;
  }
  manager_->refreshChannelList();
  PopulateTopicTree(topic_tree_, *manager_);
  applyTopicVisibility();
}

void AddDisplayDialog::onTabChanged(int index) {
  if (index == topic_tab_) {
    refreshTopicTree();
  }
  updateUi();
}

void AddDisplayDialog::onNameChanged() { updateUi(); }

void AddDisplayDialog::updateUi() {
  if (stack_ != nullptr && stack_->currentIndex() == topic_tab_) {
    selection_ = topic_tab_selection_;
  } else {
    selection_ = display_tab_selection_;
  }

  const QString selection_key =
      selection_.type + QLatin1Char('|') + selection_.channel;
  if (selection_key != last_selection_key_) {
    last_selection_key_ = selection_key;
    user_edited_name_ = false;
  }

  if (!user_edited_name_ && !selection_.type.isEmpty()) {
    name_editor_->setText(SuggestDisplayName(
        selection_.type, selection_.channel, disallowed_display_names_));
  }

  button_box_->button(QDialogButtonBox::Ok)->setEnabled(isValid());
}

bool AddDisplayDialog::isValid() {
  if (selection_.type.isEmpty()) {
    setError(tr("Select a Display type."));
    return false;
  }
  const QString display_name = name_editor_->text().trimmed();
  if (display_name.isEmpty()) {
    setError(tr("Enter a name for the display."));
    return false;
  }
  if (disallowed_display_names_.contains(display_name)) {
    setError(tr("Name in use. Display names must be unique."));
    return false;
  }
  setError({});
  return true;
}

void AddDisplayDialog::setError(const QString& error_text) {
  if (button_box_ == nullptr) {
    return;
  }
  button_box_->button(QDialogButtonBox::Ok)->setToolTip(error_text);
}

void AddDisplayDialog::accept() {
  selection_.display_name = name_editor_->text().trimmed();
  if (isValid()) {
    QDialog::accept();
  }
}

}  // namespace autoviz
