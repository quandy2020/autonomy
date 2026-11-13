/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/plot/ui/plot_field_picker_dialog.hpp"

#include <QHBoxLayout>
#include <QAbstractItemView>
#include <QKeyEvent>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPalette>
#include <QPushButton>
#include <QStyledItemDelegate>
#include <QStyleOptionViewItem>
#include <QVBoxLayout>

#include "autoviz/common/display_catalog.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/plot/field/message_field_tree.hpp"
#include "autoviz/ui/plot/field/plot_path_utils.hpp"
#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace plot {
namespace {

constexpr int kCardInset = 8;
constexpr int kCardRadius = 12;
constexpr int kRowHeight = 22;
constexpr int kChannelRole = Qt::UserRole;
constexpr int kFieldRole = Qt::UserRole + 1;
constexpr int kCountRole = Qt::UserRole + 2;
constexpr int kFieldsRole = Qt::UserRole + 3;
constexpr int kShortNameRole = Qt::UserRole + 4;
constexpr int kDisplayTypeRole = Qt::UserRole + 5;
constexpr int kIconPx = 14;

QString ChannelShortName(const QString& channel) {
  QString path = channel.trimmed();
  while (path.startsWith(QLatin1Char('/'))) {
    path.remove(0, 1);
  }
  if (path.isEmpty()) {
    return channel;
  }
  const QStringList parts =
      path.split(QLatin1Char('/'), Qt::SkipEmptyParts);
  if (parts.size() <= 2) {
    return QLatin1Char('/') + parts.join(QLatin1Char('/'));
  }
  return parts[parts.size() - 2] + QLatin1Char('/') + parts.back();
}

QString InferDisplayType(const QString& channel,
                         const std::string& message_type) {
  const std::vector<std::string> types =
      common::DisplayCatalog::typesForChannel(channel.toStdString(),
                                              message_type);
  if (!types.empty()) {
    return QString::fromStdString(types.front());
  }
  // Catalog miss (task feedback, etc.): map schema leaf → classes/*.png.
  static const struct {
    const char* needle;
    const char* type;
  } kHints[] = {
      {"OccupancyGrid", "Map"},
      {"Odometry", "Odometry"},
      {"LaserScan", "LaserScan"},
      {"PointCloud2", "PointCloud2"},
      {"PointCloud", "PointCloud"},
      {"Path", "Path"},
      {"TwistStamped", "TwistStamped"},
      {"Twist", "TwistStamped"},
      {"Imu", "Imu"},
      {"Image", "Image"},
      {"CameraInfo", "CameraInfo"},
      {"PoseWithCovariance", "PoseWithCovariance"},
      {"PoseStamped", "Pose"},
      {"PoseArray", "PoseArray"},
      {"Pose", "Pose"},
      {"Polygon", "Polygon"},
      {"MarkerArray", "MarkerArray"},
      {"Marker", "Marker"},
      {"TFMessage", "TF"},
      {"GridCells", "GridCells"},
      {"Range", "Range"},
      {"Wrench", "Wrench"},
      {"Effort", "Effort"},
      {"FluidPressure", "FluidPressure"},
      {"Illuminance", "Illuminance"},
      {"Temperature", "Temperature"},
      {"RelativeHumidity", "RelativeHumidity"},
      {"PointStamped", "PointStamped"},
      {"AccelStamped", "AccelStamped"},
  };
  const QString mt = QString::fromStdString(message_type);
  for (const auto& hint : kHints) {
    if (mt.contains(QLatin1String(hint.needle), Qt::CaseInsensitive)) {
      return QLatin1String(hint.type);
    }
  }
  return {};
}

QIcon ChannelIconForType(const QString& display_type) {
  if (!display_type.isEmpty()) {
    const QIcon icon = IconLoader::displayIcon(display_type);
    if (!icon.isNull()) {
      return icon;
    }
  }
  return IconLoader::panelIcon(QStringLiteral("PanelPlot"));
}

class GlassPane : public QWidget {
 public:
  explicit GlassPane(QWidget* parent = nullptr) : QWidget(parent) {
    setAttribute(Qt::WA_TranslucentBackground, true);
    setAutoFillBackground(false);
  }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const QRectF card = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    QPainterPath path;
    path.addRoundedRect(card, 10.0, 10.0);
    painter.fillPath(path, QColor(255, 255, 255));
    painter.setPen(QPen(glass::Shell().border_light, 1.0));
    painter.drawPath(path);
  }
};

class RailRowDelegate : public QStyledItemDelegate {
 public:
  explicit RailRowDelegate(bool show_icon, QObject* parent = nullptr)
      : QStyledItemDelegate(parent), show_icon_(show_icon) {}

  QSize sizeHint(const QStyleOptionViewItem& option,
                 const QModelIndex& index) const override {
    Q_UNUSED(index);
    return QSize(option.rect.width(), kRowHeight);
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

    painter->save();
    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setRenderHint(QPainter::SmoothPixmapTransform, true);
    QPainterPath path;
    path.addRoundedRect(QRectF(row).adjusted(0.5, 0.5, -0.5, -0.5), 4.0, 4.0);
    if (selected) {
      painter->fillPath(path, glass::ShellSelectionFill());
    } else if (hovered) {
      painter->fillPath(path, glass::ShellHoverFill());
    }

    const QString title = index.data(Qt::DisplayRole).toString();
    const int count = index.data(kCountRole).toInt();
    const QColor ink =
        selected ? glass::ShellSelectionInk() : glass::Shell().text;
    const QColor muted = glass::Shell().secondary_text;

    QFont title_font = opt.font;
    title_font.setPointSizeF(qMax(10.5, title_font.pointSizeF() - 0.5));
    if (selected) {
      title_font.setWeight(QFont::DemiBold);
    }

    QRect content = row.adjusted(6, 0, -6, 0);
    if (show_icon_) {
      const QString display_type = index.data(kDisplayTypeRole).toString();
      const QIcon icon = ChannelIconForType(display_type);
      const QRect icon_rect(content.left(),
                            content.center().y() - kIconPx / 2, kIconPx,
                            kIconPx);
      icon.paint(painter, icon_rect, Qt::AlignCenter,
                 selected ? QIcon::Selected : QIcon::Normal, QIcon::On);
      content.setLeft(icon_rect.right() + 6);
    }
    if (count > 0) {
      const QString badge = QString::number(count);
      QFont badge_font = opt.font;
      badge_font.setPointSizeF(qMax(8.5, badge_font.pointSizeF() - 1.5));
      badge_font.setWeight(QFont::DemiBold);
      const QFontMetrics badge_fm(badge_font);
      const int badge_w = badge_fm.horizontalAdvance(badge) + 10;
      const int badge_h = 14;
      QRect badge_rect(content.right() - badge_w + 1,
                       content.center().y() - badge_h / 2, badge_w, badge_h);
      QPainterPath badge_path;
      badge_path.addRoundedRect(QRectF(badge_rect), 8.0, 8.0);
      painter->fillPath(badge_path, selected ? glass::Shell().hover_bg
                                             : QColor(0, 0, 0, 14));
      painter->setFont(badge_font);
      painter->setPen(selected ? glass::ShellSelectionInk() : muted);
      painter->drawText(badge_rect, Qt::AlignCenter, badge);
      content.setRight(badge_rect.left() - 6);
    }

    const QFontMetrics title_fm(title_font);
    painter->setFont(title_font);
    painter->setPen(ink);
    // Full channel path, middle-elide so /task/.../feedback stays readable.
    painter->drawText(
        content, Qt::AlignVCenter | Qt::AlignLeft,
        title_fm.elidedText(title, Qt::ElideMiddle, content.width()));
    painter->restore();
  }

 private:
  bool show_icon_ = false;
};

void PrepList(QListWidget* list, bool show_icon) {
  list->setAttribute(Qt::WA_TranslucentBackground, true);
  list->setAutoFillBackground(false);
  if (list->viewport() != nullptr) {
    list->viewport()->setAttribute(Qt::WA_TranslucentBackground, true);
    list->viewport()->setAutoFillBackground(false);
  }
  list->setFocusPolicy(Qt::StrongFocus);
  list->setAttribute(Qt::WA_MacShowFocusRect, false);
  list->setUniformItemSizes(true);
  list->setSpacing(0);
  list->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  list->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  list->setTextElideMode(Qt::ElideRight);
  list->setMouseTracking(true);
  list->setSelectionMode(QAbstractItemView::SingleSelection);

  QPalette pal = list->palette();
  pal.setBrush(QPalette::Highlight, Qt::transparent);
  pal.setBrush(QPalette::HighlightedText, glass::ShellSelectionInk());
  pal.setBrush(QPalette::Inactive, QPalette::Highlight, Qt::transparent);
  list->setPalette(pal);
  list->setItemDelegate(new RailRowDelegate(show_icon, list));
  if (show_icon) {
    list->setIconSize(QSize(kIconPx, kIconPx));
  }
}

}  // namespace

PlotFieldPickerDialog::PlotFieldPickerDialog(
    common::VisualizationManager* manager, QWidget* parent)
    : manager_(manager), QDialog(parent) {
  setObjectName(QStringLiteral("PlotFieldPickerDialog"));
  setWindowTitle(tr("Select plot field"));
  setModal(true);
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setAttribute(Qt::WA_TranslucentBackground, true);
  setAttribute(Qt::WA_StyledBackground, false);
  setAutoFillBackground(false);
  // Dense rows + taller window so more channels fit without feeling cramped.
  setMinimumSize(520, 480);
  resize(640, 620);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(kCardInset + 10, kCardInset + 8, kCardInset + 10,
                           kCardInset + 8);
  root->setSpacing(0);

  auto* header = new QWidget(this);
  header->setAttribute(Qt::WA_TranslucentBackground, true);
  auto* header_layout = new QHBoxLayout(header);
  header_layout->setContentsMargins(0, 0, 0, 0);
  header_layout->setSpacing(8);

  auto* title = new QLabel(tr("Select plot field"), header);
  title->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  title->setStyleSheet(style::sheet(QStringLiteral("plot/field_picker/title")));

  status_label_ = new QLabel(header);
  status_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
  status_label_->setStyleSheet(
      style::sheet(QStringLiteral("plot/field_picker/caption")));
  header_layout->addWidget(title, 1);
  header_layout->addWidget(status_label_, 0, Qt::AlignVCenter);

  filter_edit_ = new QLineEdit(this);
  filter_edit_->setPlaceholderText(
      tr("Filter…  (/odom · battery)  — Enter to add"));
  filter_edit_->setClearButtonEnabled(true);
  filter_edit_->setStyleSheet(
      style::sheet(QStringLiteral("plot/field_picker/line_edit")));

  auto* body = new QHBoxLayout();
  body->setContentsMargins(0, 0, 0, 0);
  body->setSpacing(6);

  auto* channel_pane = new GlassPane(this);
  auto* channel_layout = new QVBoxLayout(channel_pane);
  channel_layout->setContentsMargins(4, 4, 2, 4);
  channel_layout->setSpacing(1);
  channel_caption_ = new QLabel(tr("CHANNELS"), channel_pane);
  channel_caption_->setStyleSheet(
      style::sheet(QStringLiteral("plot/field_picker/rail_caption")));
  channel_list_ = new QListWidget(channel_pane);
  channel_list_->setObjectName(
      QStringLiteral("PlotFieldPickerDialog/Channels"));
  channel_list_->setStyleSheet(
      style::sheet(QStringLiteral("plot/field_picker/channel_list")));
  PrepList(channel_list_, /*show_icon=*/true);
  channel_layout->addWidget(channel_caption_);
  channel_layout->addWidget(channel_list_, 1);

  auto* field_pane = new GlassPane(this);
  auto* field_layout = new QVBoxLayout(field_pane);
  field_layout->setContentsMargins(4, 4, 2, 4);
  field_layout->setSpacing(1);
  field_caption_ = new QLabel(tr("FIELDS"), field_pane);
  field_caption_->setStyleSheet(
      style::sheet(QStringLiteral("plot/field_picker/rail_caption")));
  field_list_ = new QListWidget(field_pane);
  field_list_->setObjectName(QStringLiteral("PlotFieldPickerDialog/Fields"));
  field_list_->setStyleSheet(
      style::sheet(QStringLiteral("plot/field_picker/field_list")));
  PrepList(field_list_, /*show_icon=*/false);
  field_layout->addWidget(field_caption_);
  field_layout->addWidget(field_list_, 1);

  // Channels usually outnumber fields — give the left rail more width.
  body->addWidget(channel_pane, 7);
  body->addWidget(field_pane, 4);

  path_preview_ = new QLabel(tr("No field selected"), this);
  path_preview_->setWordWrap(false);
  path_preview_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  path_preview_->setStyleSheet(
      style::sheet(QStringLiteral("plot/field_picker/path")));

  auto* action_row = new QWidget(this);
  action_row->setAttribute(Qt::WA_TranslucentBackground, true);
  auto* action_layout = new QHBoxLayout(action_row);
  action_layout->setContentsMargins(0, 0, 0, 0);
  action_layout->setSpacing(8);

  auto* cancel = new QPushButton(tr("Cancel"), action_row);
  cancel->setObjectName(QStringLiteral("PlotFieldPickerCancel"));
  cancel->setCursor(Qt::PointingHandCursor);
  cancel->setAutoDefault(false);
  cancel->setFlat(false);
  cancel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  cancel->setStyleSheet(
      style::sheet(QStringLiteral("plot/field_picker/ghost_button")));

  add_button_ = new QPushButton(tr("Add"), action_row);
  add_button_->setObjectName(QStringLiteral("PlotFieldPickerAdd"));
  add_button_->setCursor(Qt::PointingHandCursor);
  add_button_->setAutoDefault(true);
  add_button_->setDefault(true);
  add_button_->setFlat(false);
  add_button_->setEnabled(false);
  add_button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  add_button_->setStyleSheet(
      style::sheet(QStringLiteral("plot/field_picker/primary_button")));
  add_button_->setToolTip(tr("Add selected field (Enter)"));

  action_layout->addWidget(cancel, 1);
  action_layout->addWidget(add_button_, 1);

  root->addWidget(header);
  root->addSpacing(6);
  root->addWidget(filter_edit_);
  root->addSpacing(6);
  root->addLayout(body, 1);
  root->addSpacing(6);
  root->addWidget(path_preview_);
  root->addSpacing(6);
  root->addWidget(action_row);

  rebuildCatalog();
  applyFilter(QString());

  connect(filter_edit_, &QLineEdit::textChanged, this,
          &PlotFieldPickerDialog::applyFilter);
  connect(channel_list_, &QListWidget::currentItemChanged, this,
          [this](QListWidgetItem*, QListWidgetItem*) { onChannelChanged(); });
  connect(field_list_, &QListWidget::currentItemChanged, this,
          [this](QListWidgetItem*, QListWidgetItem*) { onFieldChanged(); });
  connect(field_list_, &QListWidget::itemDoubleClicked, this,
          [this](QListWidgetItem*) { acceptCurrent(); });
  connect(channel_list_, &QListWidget::itemActivated, this,
          [this](QListWidgetItem*) {
            if (field_list_->count() > 0) {
              field_list_->setFocus(Qt::TabFocusReason);
            }
          });
  connect(add_button_, &QPushButton::clicked, this,
          &PlotFieldPickerDialog::acceptCurrent);
  connect(cancel, &QPushButton::clicked, this, &QDialog::reject);

  filter_edit_->setFocus();
}

QSize PlotFieldPickerDialog::sizeHint() const { return {640, 620}; }

void PlotFieldPickerDialog::keyPressEvent(QKeyEvent* event) {
  if (event == nullptr) {
    QDialog::keyPressEvent(event);
    return;
  }
  if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
    if (add_button_ != nullptr && add_button_->isEnabled()) {
      acceptCurrent();
      event->accept();
      return;
    }
  }
  if (event->matches(QKeySequence::Find) ||
      ((event->modifiers() & Qt::ControlModifier) &&
       event->key() == Qt::Key_F)) {
    filter_edit_->setFocus(Qt::ShortcutFocusReason);
    filter_edit_->selectAll();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_Down && filter_edit_->hasFocus() &&
      channel_list_->count() > 0) {
    channel_list_->setFocus(Qt::TabFocusReason);
    if (channel_list_->currentRow() < 0) {
      channel_list_->setCurrentRow(0);
    }
    event->accept();
    return;
  }
  QDialog::keyPressEvent(event);
}

void PlotFieldPickerDialog::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QRectF outer =
      QRectF(rect()).adjusted(kCardInset, kCardInset, -kCardInset, -kCardInset);
  QPainterPath card;
  card.addRoundedRect(outer, kCardRadius, kCardRadius);

  // Soft single-layer shadow — CapCut-like, not stacked glow.
  QPainterPath shadow;
  shadow.addRoundedRect(outer.adjusted(0.0, 2.0, 0.0, 3.0), kCardRadius,
                        kCardRadius);
  painter.fillPath(shadow, QColor(0, 0, 0, 28));

  painter.fillPath(card, QColor(250, 251, 253));
  painter.setPen(QPen(glass::Shell().border_light, 1.0));
  painter.drawPath(card);
}

void PlotFieldPickerDialog::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    QWidget* hit = childAt(event->pos());
    if (hit == nullptr || hit == this) {
      dragging_ = true;
      drag_offset_ =
          event->globalPosition().toPoint() - frameGeometry().topLeft();
      event->accept();
      return;
    }
  }
  QDialog::mousePressEvent(event);
}

void PlotFieldPickerDialog::mouseMoveEvent(QMouseEvent* event) {
  if (dragging_ && (event->buttons() & Qt::LeftButton)) {
    move(event->globalPosition().toPoint() - drag_offset_);
    event->accept();
    return;
  }
  QDialog::mouseMoveEvent(event);
}

void PlotFieldPickerDialog::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    dragging_ = false;
  }
  QDialog::mouseReleaseEvent(event);
}

void PlotFieldPickerDialog::rebuildCatalog() {
  catalog_.clear();
  if (manager_ == nullptr) {
    return;
  }
  for (const QString& channel : AllKnownChannels(manager_)) {
    if (channel.trimmed().isEmpty()) {
      continue;
    }
    const std::string message_type = MessageTypeForChannel(manager_, channel);
    const QStringList fields = NumericFieldPathsForMessageType(message_type);
    if (fields.isEmpty()) {
      continue;
    }
    catalog_.push_back(
        ChannelEntry{channel, fields, InferDisplayType(channel, message_type)});
  }
}

void PlotFieldPickerDialog::applyFilter(const QString& text) {
  const QString needle = text.trimmed();
  const QString previous_channel =
      channel_list_->currentItem() != nullptr
          ? channel_list_->currentItem()->data(kChannelRole).toString()
          : QString();

  channel_list_->clear();
  field_list_->clear();

  int restore_row = -1;
  int visible_fields = 0;
  for (const ChannelEntry& entry : catalog_) {
    QStringList matched_fields;
    if (needle.isEmpty()) {
      matched_fields = entry.fields;
    } else {
      const bool channel_hit =
          entry.channel.contains(needle, Qt::CaseInsensitive) ||
          ChannelShortName(entry.channel)
              .contains(needle, Qt::CaseInsensitive);
      for (const QString& field : entry.fields) {
        if (channel_hit || field.contains(needle, Qt::CaseInsensitive) ||
            CombinedPlotValuePath(entry.channel, field)
                .contains(needle, Qt::CaseInsensitive)) {
          matched_fields.push_back(field);
        }
      }
    }
    if (matched_fields.isEmpty()) {
      continue;
    }
    visible_fields += matched_fields.size();
    auto* item = new QListWidgetItem(entry.channel, channel_list_);
    item->setData(kChannelRole, entry.channel);
    item->setData(kShortNameRole, ChannelShortName(entry.channel));
    item->setData(kCountRole, matched_fields.size());
    item->setData(kFieldsRole, matched_fields);
    item->setData(kDisplayTypeRole, entry.display_type);
    item->setToolTip(
        entry.display_type.isEmpty()
            ? entry.channel
            : entry.channel + QStringLiteral("  ·  ") + entry.display_type);
    if (entry.channel == previous_channel) {
      restore_row = channel_list_->count() - 1;
    }
  }

  const int channel_count = channel_list_->count();
  if (channel_caption_ != nullptr) {
    channel_caption_->setText(
        channel_count > 0 ? tr("CHANNELS · %1").arg(channel_count)
                          : tr("CHANNELS"));
  }
  if (status_label_ != nullptr) {
    status_label_->setText(tr("%1 ch · %2 fields")
                               .arg(channel_count)
                               .arg(visible_fields));
  }

  if (channel_count == 0) {
    auto* empty =
        new QListWidgetItem(tr("No matching channels"), channel_list_);
    empty->setFlags(empty->flags() & ~Qt::ItemIsSelectable);
    if (field_caption_ != nullptr) {
      field_caption_->setText(tr("FIELDS"));
    }
    updatePathPreview();
    updateAddEnabled();
    return;
  }

  channel_list_->setCurrentRow(restore_row >= 0 ? restore_row : 0);
  onChannelChanged();
}

void PlotFieldPickerDialog::onChannelChanged() {
  field_list_->clear();
  QListWidgetItem* channel_item = channel_list_->currentItem();
  if (channel_item == nullptr ||
      !(channel_item->flags() & Qt::ItemIsSelectable)) {
    if (field_caption_ != nullptr) {
      field_caption_->setText(tr("FIELDS"));
    }
    updatePathPreview();
    updateAddEnabled();
    return;
  }

  const QString channel = channel_item->data(kChannelRole).toString();
  const QStringList fields = channel_item->data(kFieldsRole).toStringList();
  for (const QString& field : fields) {
    auto* item = new QListWidgetItem(field, field_list_);
    item->setData(kChannelRole, channel);
    item->setData(kFieldRole, field);
    item->setToolTip(CombinedPlotValuePath(channel, field));
  }
  if (field_caption_ != nullptr) {
    field_caption_->setText(fields.isEmpty()
                                ? tr("FIELDS")
                                : tr("FIELDS · %1").arg(fields.size()));
  }
  if (field_list_->count() > 0) {
    field_list_->setCurrentRow(0);
  }
  onFieldChanged();
}

void PlotFieldPickerDialog::onFieldChanged() {
  QListWidgetItem* item = field_list_->currentItem();
  if (item == nullptr) {
    selected_channel_.clear();
    selected_field_path_.clear();
  } else {
    selected_channel_ = item->data(kChannelRole).toString();
    selected_field_path_ = item->data(kFieldRole).toString();
  }
  updatePathPreview();
  updateAddEnabled();
}

void PlotFieldPickerDialog::updatePathPreview() {
  if (path_preview_ == nullptr) {
    return;
  }
  if (selected_channel_.isEmpty() || selected_field_path_.isEmpty()) {
    path_preview_->setText(tr("No field selected"));
    path_preview_->setToolTip(QString());
    return;
  }
  const QString path =
      CombinedPlotValuePath(selected_channel_, selected_field_path_);
  path_preview_->setText(path);
  path_preview_->setToolTip(path);
}

void PlotFieldPickerDialog::updateAddEnabled() {
  if (add_button_ == nullptr) {
    return;
  }
  add_button_->setEnabled(!selected_channel_.isEmpty() &&
                          !selected_field_path_.isEmpty());
}

void PlotFieldPickerDialog::acceptCurrent() {
  if (selected_channel_.isEmpty() || selected_field_path_.isEmpty()) {
    return;
  }
  accept();
}

bool PlotFieldPickerDialog::pick(QWidget* parent,
                                 common::VisualizationManager* manager,
                                 QString* channel_out, QString* field_path_out) {
  PlotFieldPickerDialog dialog(manager, parent);
  if (dialog.exec() != QDialog::Accepted) {
    return false;
  }
  if (channel_out != nullptr) {
    *channel_out = dialog.selectedChannel();
  }
  if (field_path_out != nullptr) {
    *field_path_out = dialog.selectedFieldPath();
  }
  return !dialog.selectedChannel().isEmpty() &&
         !dialog.selectedFieldPath().isEmpty();
}

}  // namespace plot
}  // namespace autoviz
