/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/displays/add_dialog.hpp"

#include <map>
#include <vector>

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFont>
#include <QFrame>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPalette>
#include <QPushButton>
#include <QStackedWidget>
#include <QStyledItemDelegate>
#include <QStyleOptionViewItem>
#include <QAbstractItemView>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QVBoxLayout>

#include "autoviz/common/display_catalog.hpp"
#include "autoviz/common/display_factory.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_manager.hpp"
#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
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

  void paint(QPainter* painter, const QStyleOptionViewItem& option,
             const QModelIndex& index) const override {
    QStyleOptionViewItem opt(option);
    initStyleOption(&opt, index);

    const bool selected = opt.state.testFlag(QStyle::State_Selected);
    const bool hovered = opt.state.testFlag(QStyle::State_MouseOver);

    QRect row = opt.rect;
    if (const auto* view = qobject_cast<const QAbstractItemView*>(opt.widget)) {
      row = QRect(2, opt.rect.y(), view->viewport()->width() - 4, opt.rect.height());
    }

    if (selected || hovered) {
      painter->save();
      painter->setRenderHint(QPainter::Antialiasing, true);
      QPainterPath path;
      path.addRoundedRect(QRectF(row).adjusted(0.5, 1.0, -0.5, -1.0), 8.0, 8.0);
      if (selected) {
        QLinearGradient fill(row.topLeft(), row.topRight());
        fill.setColorAt(0.0, QColor(20, 184, 166, 78));
        fill.setColorAt(1.0, QColor(34, 211, 238, 62));
        painter->fillPath(path, fill);
      } else {
        painter->fillPath(path, QColor(165, 243, 252, 56));
      }
      painter->restore();
    }

    // Suppress native/QSS selection fill; keep icon + text rendering.
    opt.state.setFlag(QStyle::State_Selected, false);
    opt.state.setFlag(QStyle::State_MouseOver, false);
    opt.backgroundBrush = Qt::NoBrush;
    if (selected) {
      opt.palette.setColor(QPalette::Text, QColor(0x0F, 0x76, 0x6E));
      opt.palette.setColor(QPalette::WindowText, QColor(0x0F, 0x76, 0x6E));
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

    painter.fillPath(path, QColor(255, 255, 255, 72));
    QLinearGradient sheen(card.topLeft(),
                          QPointF(card.left(), card.top() + 48.0));
    sheen.setColorAt(0.0, QColor(255, 255, 255, 110));
    sheen.setColorAt(1.0, QColor(255, 255, 255, 0));
    painter.fillPath(path, sheen);

    painter.setPen(QPen(QColor(255, 255, 255, 160), 1.0));
    painter.drawPath(path);
    painter.setPen(QPen(QColor(203, 213, 225, 90), 1.0));
    painter.drawRoundedRect(card.adjusted(1.0, 1.0, -1.0, -1.0), 13.0, 13.0);
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

    painter.fillPath(path, QColor(255, 255, 255, 96));
    QLinearGradient wash(card.topLeft(), card.bottomLeft());
    wash.setColorAt(0.0, QColor(255, 255, 255, 130));
    wash.setColorAt(1.0, QColor(240, 249, 255, 70));
    painter.fillPath(path, wash);

    painter.setPen(QPen(QColor(186, 230, 253, 120), 1.0));
    painter.drawPath(path);
    painter.setPen(QPen(QColor(255, 255, 255, 180), 1.0));
    painter.drawRoundedRect(card.adjusted(1.0, 1.0, -1.0, -1.0), 15.0, 15.0);
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
  if (item == nullptr) {
    return selection;
  }
  if (!item->data(0, kTypeRole).isValid()) {
    for (int child_index = 0; child_index < item->childCount(); ++child_index) {
      QTreeWidgetItem* child = item->child(child_index);
      if (child != nullptr && child->data(0, kTypeRole).isValid()) {
        return SelectionFromTopicItem(child);
      }
    }
    return selection;
  }
  selection.type = item->data(0, kTypeRole).toString();
  selection.display_name = item->text(0);
  selection.channel = item->data(0, kChannelRole).toString();
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
    const QString candidate =
        base + QLatin1Char(' ') + QString::number(suffix);
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
  for (const common::DisplayTypeInfo& info : common::DisplayCatalog::allTypes()) {
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

QTreeWidgetItem* InsertTopicPath(QTreeWidget* tree, const QString& channel,
                                 bool disabled) {
  QTreeWidgetItem* current = tree->invisibleRootItem();
  const QStringList parts =
      channel.split(QLatin1Char('/'), Qt::SkipEmptyParts);
  for (int part_index = 0; part_index < parts.size(); ++part_index) {
    const QString part = QStringLiteral("/") + parts[part_index];
    QTreeWidgetItem* match = nullptr;
    for (int child_index = 0; child_index < current->childCount(); ++child_index) {
      QTreeWidgetItem* child = current->child(child_index);
      if (child != nullptr && child->text(0) == part &&
          !child->data(0, kTypeRole).isValid()) {
        match = child;
        break;
      }
    }
    if (match == nullptr) {
      match = new QTreeWidgetItem(current);
      match->setText(0, part);
      match->setExpanded(part_index < 2);
      match->setDisabled(disabled);
      current = match;
    } else {
      if (!disabled) {
        match->setDisabled(false);
      }
      current = match;
    }
  }
  return current;
}

void PopulateTopicTree(QTreeWidget* tree,
                       const common::VisualizationManager& manager) {
  tree->clear();
  for (const integration::ChannelInfo& channel : manager.channels()) {
    const std::vector<std::string> display_types =
        common::DisplayCatalog::typesForChannel(channel.channel_name,
                                                channel.message_type);
    const bool visualizable = !display_types.empty();
    QTreeWidgetItem* topic_item = InsertTopicPath(
        tree, QString::fromStdString(channel.channel_name), !visualizable);
    topic_item->setData(0, kChannelRole,
                        QString::fromStdString(channel.channel_name));
    if (visualizable) {
      for (QTreeWidgetItem* node = topic_item; node != nullptr;
           node = node->parent()) {
        node->setDisabled(false);
      }
    }
    if (!visualizable) {
      continue;
    }
    for (const std::string& type : display_types) {
      const common::DisplayTypeInfo info =
          common::DisplayCatalog::infoForType(type);
      auto* row = new QTreeWidgetItem(topic_item);
      const QString type_q = QString::fromStdString(type);
      row->setText(0, type_q);
      row->setIcon(0, IconLoader::displayIcon(type_q));
      row->setData(0, kTypeRole, type_q);
      row->setData(0, kChannelRole,
                   QString::fromStdString(channel.channel_name));
      row->setToolTip(0, QString::fromStdString(info.description));
    }
  }
}

void ApplyFrostedTree(QTreeWidget* tree) {
  if (tree == nullptr) {
    return;
  }
  tree->setAttribute(Qt::WA_TranslucentBackground, true);
  tree->setAutoFillBackground(false);
  if (tree->viewport() != nullptr) {
    tree->viewport()->setAttribute(Qt::WA_TranslucentBackground, true);
    tree->viewport()->setAutoFillBackground(false);
  }
  tree->setStyleSheet(
      style::sheet(QStringLiteral("add_display/frosted_tree")));
  tree->setFocusPolicy(Qt::StrongFocus);
  tree->setAttribute(Qt::WA_MacShowFocusRect, false);
  tree->setIndentation(12);
  tree->setUniformRowHeights(true);
  tree->setAnimated(false);
  tree->setAllColumnsShowFocus(false);

  // Kill native selection strip (branch vs item) — only the delegate paints.
  QPalette pal = tree->palette();
  pal.setBrush(QPalette::Highlight, Qt::transparent);
  pal.setBrush(QPalette::HighlightedText, QColor(0x0F, 0x76, 0x6E));
  pal.setBrush(QPalette::Inactive, QPalette::Highlight, Qt::transparent);
  pal.setBrush(QPalette::Inactive, QPalette::HighlightedText,
               QColor(0x0F, 0x76, 0x6E));
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

  segment_track_ = new QFrame(this);
  segment_track_->setObjectName(QStringLiteral("CreateVizSegmentTrack"));
  segment_track_->setAttribute(Qt::WA_StyledBackground, true);
  segment_track_->setStyleSheet(
      style::sheet(QStringLiteral("add_display/segment_track")));
  segment_track_->setFixedHeight(46);
  auto* segment_layout = new QHBoxLayout(segment_track_);
  segment_layout->setContentsMargins(5, 5, 5, 5);
  segment_layout->setSpacing(6);
  tab_by_type_ = new QPushButton(tr("Type"), segment_track_);
  tab_by_channel_ = new QPushButton(tr("Channel"), segment_track_);
  tab_by_type_->setCursor(Qt::PointingHandCursor);
  tab_by_channel_->setCursor(Qt::PointingHandCursor);
  tab_by_type_->setCheckable(false);
  tab_by_channel_->setCheckable(false);
  segment_layout->addWidget(tab_by_type_, 1);
  segment_layout->addWidget(tab_by_channel_, 1);
  connect(tab_by_type_, &QPushButton::clicked, this,
          [this]() { setActiveTab(display_tab_); });
  connect(tab_by_channel_, &QPushButton::clicked, this,
          [this]() { setActiveTab(topic_tab_); });

  auto* well = new GlassWell(this);
  auto* well_layout = new QVBoxLayout(well);
  well_layout->setContentsMargins(4, 6, 4, 6);
  well_layout->setSpacing(0);

  stack_ = new QStackedWidget(well);
  stack_->setAttribute(Qt::WA_TranslucentBackground, true);
  stack_->setStyleSheet(style::mark(style::Mark::Clear));

  display_tree_ = new QTreeWidget(stack_);
  display_tree_->setObjectName(QStringLiteral("AddDisplayDialog/DisplayTypeTree"));
  display_tree_->setHeaderHidden(true);
  display_tree_->setIconSize(QSize(16, 16));
  display_tree_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  PopulateDisplayTypeTree(display_tree_);
  ApplyFrostedTree(display_tree_);
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
  ApplyFrostedTree(topic_tree_);
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
  footer_layout->setContentsMargins(14, 12, 14, 12);
  footer_layout->setSpacing(10);

  auto* name_caption = new QLabel(tr("Name"), footer);
  name_caption->setStyleSheet(
      style::sheet(QStringLiteral("add_display/name_caption")));

  name_editor_ = new QLineEdit(footer);
  name_editor_->setObjectName(QStringLiteral("AddDisplayDialog/DisplayNameEdit"));
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
  action_layout->setContentsMargins(0, 2, 0, 0);
  action_layout->setSpacing(10);

  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                     Qt::Horizontal, action_row);
  button_box_->setObjectName(QStringLiteral("AddDisplayDialog/ButtonBox"));
  button_box_->setCenterButtons(false);
  if (QPushButton* ok = button_box_->button(QDialogButtonBox::Ok)) {
    ok->setText(QStringLiteral("Confirm"));
    ok->setCursor(Qt::PointingHandCursor);
    ok->setStyleSheet(
        style::sheet(QStringLiteral("add_display/primary_button")));
    ok->setDefault(true);
  }
  if (QPushButton* cancel = button_box_->button(QDialogButtonBox::Cancel)) {
    cancel->setText(QStringLiteral("Cancel"));
    cancel->setCursor(Qt::PointingHandCursor);
    cancel->setStyleSheet(
        style::sheet(QStringLiteral("add_display/ghost_button")));
  }
  // Keep Cancel left of Confirm with a trailing flush to the right edge.
  action_layout->addStretch(1);
  action_layout->addWidget(button_box_, 0, Qt::AlignRight);

  footer_layout->addWidget(name_caption);
  footer_layout->addWidget(name_editor_);
  footer_layout->addWidget(action_row);

  root->addWidget(segment_track_);
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

  const QRectF outer = QRectF(rect()).adjusted(kCardInset, kCardInset, -kCardInset,
                                               -kCardInset);
  QPainterPath card;
  card.addRoundedRect(outer, kCardRadius, kCardRadius);

  // Soft milky drop shadow (reads as floating glass over parent UI).
  for (int i = 8; i >= 1; --i) {
    QPainterPath glow;
    glow.addRoundedRect(outer.adjusted(-i * 0.7, -i * 0.35, i * 0.7, i * 0.9),
                        kCardRadius + i * 0.4, kCardRadius + i * 0.4);
    painter.fillPath(glow, QColor(15, 23, 42, 4 + (8 - i)));
  }

  // Dense milky body — opaque enough to read, still glass-like.
  painter.fillPath(card, QColor(252, 253, 253, 228));

  QLinearGradient wash(outer.topLeft(), outer.bottomRight());
  wash.setColorAt(0.0, QColor(255, 255, 255, 70));
  wash.setColorAt(0.45, QColor(245, 252, 250, 28));
  wash.setColorAt(1.0, QColor(241, 245, 249, 40));
  painter.fillPath(card, wash);

  QLinearGradient sheen(outer.topLeft(),
                        QPointF(outer.left(), outer.top() + outer.height() * 0.38));
  sheen.setColorAt(0.0, QColor(255, 255, 255, 150));
  sheen.setColorAt(1.0, QColor(255, 255, 255, 0));
  painter.fillPath(card, sheen);

  // Inner highlight rim + outer soft edge.
  painter.setPen(QPen(QColor(255, 255, 255, 200), 1.2));
  painter.drawPath(card);
  painter.setPen(QPen(QColor(203, 213, 225, 110), 1.0));
  painter.drawRoundedRect(outer.adjusted(1.0, 1.0, -1.0, -1.0), kCardRadius - 1.0,
                          kCardRadius - 1.0);
}

void AddDisplayDialog::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    // Drag from empty chrome / segment track margins (not tab buttons).
    QWidget* hit = childAt(event->pos());
    const bool on_tab = hit == tab_by_type_ || hit == tab_by_channel_;
    const bool on_track =
        segment_track_ != nullptr &&
        (hit == segment_track_ ||
         (hit != nullptr && hit->parentWidget() == segment_track_));
    if (!on_tab && (hit == nullptr || hit == this || on_track)) {
      dragging_ = true;
      drag_offset_ = event->globalPosition().toPoint() - frameGeometry().topLeft();
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
  if (stack_ != nullptr) {
    stack_->setCurrentIndex(index);
  }
  if (tab_by_type_ != nullptr) {
    tab_by_type_->setStyleSheet(style::sheet(
        index == display_tab_ ? QStringLiteral("add_display/segment_tab_active")
                              : QStringLiteral("add_display/segment_tab_inactive")));
  }
  if (tab_by_channel_ != nullptr) {
    tab_by_channel_->setStyleSheet(style::sheet(
        index == topic_tab_ ? QStringLiteral("add_display/segment_tab_active")
                            : QStringLiteral("add_display/segment_tab_inactive")));
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
