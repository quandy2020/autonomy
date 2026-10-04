/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/tf_tree/panel.hpp"

#include <algorithm>
#include <cmath>

#include <QApplication>
#include <QClipboard>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFocusEvent>
#include <QShowEvent>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QPainter>
#include <QPixmap>
#include <QSignalBlocker>
#include <QSplitter>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <functional>
#include <vector>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/panel/title_tools.hpp"
#include "autoviz/ui/tf_tree/graph_view.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

constexpr int kRoleFrameId = Qt::UserRole;

/** Light tokens shared with Channel Graph / TfTreeGraphView. */
constexpr char kBg[] = "rgba(255,255,255,128)";
constexpr char kSurface[] = "rgba(240,249,255,155)";
constexpr char kSurfaceRaised[] = "rgba(255,255,255,175)";
constexpr char kBorder[] = "rgba(186,230,253,140)";
constexpr char kText[] = "#1e293b";
constexpr char kTextMuted[] = "#64748b";
constexpr char kAccent[] = "#0891b2";
constexpr char kStatic[] = "#d97706";

QFrame* MakeToolbarSeparator(QWidget* parent) {
  auto* separator = new QFrame(parent);
  separator->setFrameShape(QFrame::VLine);
  separator->setFixedWidth(1);
  separator->setStyleSheet(
      style::mark(style::Mark::VLine));
  return separator;
}

QToolButton* MakeIconToolButton(QWidget* parent, const QIcon& icon,
                                const QString& tooltip) {
  auto* button = new QToolButton(parent);
  button->setObjectName(QStringLiteral("AutovizLightToolbarIcon"));
  button->setIcon(icon);
  button->setIconSize(QSize(16, 16));
  button->setToolButtonStyle(Qt::ToolButtonIconOnly);
  button->setAutoRaise(true);
  button->setCursor(Qt::PointingHandCursor);
  button->setToolTip(tooltip);
  button->setStyleSheet(
      style::sheet(QStringLiteral("widget/light_toolbar"), style::Tone::Frost));
  return button;
}

QIcon MakeGlyphIcon(const std::function<void(QPainter&, const QRectF&)>& paint,
                    int size = 18) {
  QPixmap pixmap(size, size);
  pixmap.fill(Qt::transparent);
  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing, true);
  paint(painter, QRectF(0, 0, size, size));
  return QIcon(pixmap);
}

/** Matches Channel Graph node glyph (cyan card). */
QIcon MakeFrameGlyphIcon() {
  return MakeGlyphIcon([](QPainter& p, const QRectF& r) {
    const QRectF card(r.left() + 1.5, r.top() + 3.5, r.width() - 3, r.height() - 7);
    p.setPen(QPen(QColor(0x67, 0xe8, 0xf9), 1.2));
    p.setBrush(QColor(0xec, 0xfe, 0xff));
    p.drawRoundedRect(card, 3.5, 3.5);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(QLatin1String(kAccent)));
    p.drawEllipse(QPointF(card.left() + 5.5, card.center().y()), 3.2, 3.2);
  });
}

/** Amber card for static TF frames. */
QIcon MakeStaticGlyphIcon() {
  return MakeGlyphIcon([](QPainter& p, const QRectF& r) {
    const QRectF card(r.left() + 1.5, r.top() + 3.5, r.width() - 3, r.height() - 7);
    p.setPen(QPen(QColor(QLatin1String(kStatic)), 1.2, Qt::DashLine));
    p.setBrush(QColor(0xff, 0xfb, 0xeb));
    p.drawRoundedRect(card, 3.5, 3.5);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(QLatin1String(kStatic)));
    p.drawEllipse(QPointF(card.left() + 5.5, card.center().y()), 3.2, 3.2);
  });
}

QWidget* MakeLegendChip(const QIcon& icon, const QString& tip, QWidget* parent) {
  auto* label = new QLabel(parent);
  label->setPixmap(icon.pixmap(16, 16));
  label->setToolTip(tip);
  label->setFixedSize(18, 18);
  label->setAlignment(Qt::AlignCenter);
  return label;
}

QLabel* MakeDetailLabel(const QString& text, QWidget* parent) {
  auto* label = new QLabel(text, parent);
  label->setStyleSheet(style::type(style::Role::PanelMuted, 11));
  label->setWordWrap(true);
  return label;
}

QLabel* MakeDetailValue(QWidget* parent) {
  auto* label = new QLabel(QStringLiteral("—"), parent);
  label->setStyleSheet(style::type(style::Role::PanelBody, 12));
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  label->setWordWrap(true);
  return label;
}

QString NormalizeParent(const QString& parent) {
  if (parent.isEmpty() || parent == QLatin1String("NO_PARENT")) {
    return {};
  }
  return parent;
}

constexpr double kDefaultMaxAgeSec = 1.0;
constexpr double kMaxPlausibleAgeSeconds = 86400.0;

}  // namespace

TfTreePanel::TfTreePanel(transform::Buffer* tf_buffer,
                         common::VisualizationManager* manager, QWidget* parent)
    : QWidget(parent), tf_buffer_(tf_buffer), manager_(manager) {
  setFocusPolicy(Qt::StrongFocus);
  setupUi();

  // Cap tree rebuilds: transforms-changed fires per setTransform (often >> 30Hz).
  refresh_timer_ = new QTimer(this);
  refresh_timer_->setTimerType(Qt::PreciseTimer);
  refresh_timer_->setSingleShot(true);
  refresh_timer_->setInterval(250);
  connect(refresh_timer_, &QTimer::timeout, this,
          &TfTreePanel::onCoalescedRefresh);

  if (tf_buffer_ != nullptr) {
    transforms_changed_connection_ =
        tf_buffer_->_addTransformsChangedListener([this]() { scheduleRefresh(); });
  }
  force_rebuild_ = true;
  onCoalescedRefresh();
}

TfTreePanel::~TfTreePanel() { transforms_changed_connection_.disconnect(); }

void TfTreePanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("TfTreeDock");
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.split = [this](Qt::Orientation orientation) {
    emit panelSplitRequested(orientation);
  };
  callbacks.expand = [this]() { emit panelExpandRequested(); };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };

  PanelTitleBarOptions options;
  options.on_expand = [this]() { emit panelExpandRequested(); };

  const PanelTitleBarTools tools =
      CreatePanelTitleBarTools(dock, callbacks, options);
  expand_button_ = tools.expand_button;
  dock->setTitleBarTools(tools.widget);
}

void TfTreePanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ == nullptr) {
    return;
  }
  expand_button_->blockSignals(true);
  expand_button_->setChecked(checked);
  expand_button_->blockSignals(false);
}

common::TfTreePanelPersistConfig TfTreePanel::config() const {
  common::TfTreePanelPersistConfig out;
  if (filter_edit_ != nullptr) {
    out.filter = filter_edit_->text().trimmed().toStdString();
  }
  if (view_tabs_ != nullptr) {
    out.tab_index = view_tabs_->currentIndex();
  }
  out.stale_only =
      stale_only_button_ != nullptr && stale_only_button_->isChecked();
  out.max_age_sec = max_age_spin_ != nullptr ? max_age_spin_->value()
                                             : kDefaultMaxAgeSec;
  return out;
}

void TfTreePanel::setConfig(const common::TfTreePanelPersistConfig& config) {
  applying_config_ = true;
  if (filter_edit_ != nullptr) {
    const QSignalBlocker blocker(filter_edit_);
    filter_edit_->setText(QString::fromStdString(config.filter));
  }
  if (stale_only_button_ != nullptr) {
    const QSignalBlocker blocker(stale_only_button_);
    stale_only_button_->setChecked(config.stale_only);
  }
  if (max_age_spin_ != nullptr) {
    const QSignalBlocker blocker(max_age_spin_);
    const double age =
        config.max_age_sec > 0.0 ? config.max_age_sec : kDefaultMaxAgeSec;
    max_age_spin_->setValue(age);
    max_age_spin_->setEnabled(config.stale_only);
  }
  if (view_tabs_ != nullptr) {
    const QSignalBlocker blocker(view_tabs_);
    const int tab_count = view_tabs_->count();
    if (tab_count > 0) {
      view_tabs_->setCurrentIndex(
          std::clamp(config.tab_index, 0, tab_count - 1));
    }
  }
  applying_config_ = false;
  force_rebuild_ = true;
  refresh_pending_ = true;
  onCoalescedRefresh();
}

void TfTreePanel::emitConfigChanged() {
  if (!applying_config_) {
    emit configChanged();
  }
}

void TfTreePanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void TfTreePanel::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  // App activation can emit show for docks; do not force a full tree rebuild
  // (combined with GL restore + TF catch-up that freezes the UI).
  scheduleRefresh();
}

void TfTreePanel::setupUi() {
  ApplyPanelShell(this);
  setStyleSheet(
      style::sheet(QStringLiteral("tf_tree"), style::Tone::Frost));
  setObjectName(QStringLiteral("TfTreePanelContent"));

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* toolbar = new QFrame(this);
  ApplyPanelToolbarChrome(toolbar);
  auto* row = new QHBoxLayout(toolbar);
  row->setContentsMargins(8, 4, 8, 4);
  row->setSpacing(4);

  auto* fit_button = MakeIconToolButton(
      toolbar, IconLoader::panelTitleIcon(QStringLiteral("plot.reset_view")),
      tr("Zoom graph to fit"));
  row->addWidget(fit_button);

  row->addWidget(MakeToolbarSeparator(toolbar));

  filter_edit_ = new QLineEdit(toolbar);
  filter_edit_->setObjectName(QStringLiteral("AutovizCompactFilter"));
  filter_edit_->setPlaceholderText(tr("Filter…"));
  filter_edit_->setClearButtonEnabled(true);
  filter_edit_->setMinimumWidth(120);
  filter_edit_->setStyleSheet(
      style::sheet(QStringLiteral("widget/fields"), style::Tone::Frost));
  row->addWidget(filter_edit_, 1);

  row->addWidget(MakeToolbarSeparator(toolbar));

  stale_only_button_ = new QToolButton(toolbar);
  stale_only_button_->setObjectName(QStringLiteral("AutovizLightToolbarIcon"));
  stale_only_button_->setText(tr("Stale"));
  stale_only_button_->setCheckable(true);
  stale_only_button_->setToolButtonStyle(Qt::ToolButtonTextOnly);
  stale_only_button_->setAutoRaise(true);
  stale_only_button_->setCursor(Qt::PointingHandCursor);
  stale_only_button_->setToolTip(
      tr("Show only frames older than the max-age threshold"));
  stale_only_button_->setStyleSheet(
      style::sheet(QStringLiteral("widget/light_toolbar"), style::Tone::Frost));
  row->addWidget(stale_only_button_);

  max_age_spin_ = new QDoubleSpinBox(toolbar);
  max_age_spin_->setObjectName(QStringLiteral("AutovizCompactSpin"));
  max_age_spin_->setRange(0.1, 3600.0);
  max_age_spin_->setDecimals(1);
  max_age_spin_->setSingleStep(0.5);
  max_age_spin_->setValue(kDefaultMaxAgeSec);
  max_age_spin_->setSuffix(tr(" s"));
  max_age_spin_->setToolTip(tr("Max age threshold for Stale filter"));
  max_age_spin_->setEnabled(false);
  max_age_spin_->setFixedWidth(72);
  max_age_spin_->setStyleSheet(
      style::sheet(QStringLiteral("widget/fields"), style::Tone::Frost));
  row->addWidget(max_age_spin_);

  row->addWidget(MakeToolbarSeparator(toolbar));
  row->addWidget(MakeLegendChip(MakeFrameGlyphIcon(), tr("Frame"), toolbar));
  row->addWidget(MakeLegendChip(MakeStaticGlyphIcon(), tr("Static"), toolbar));

  root->addWidget(toolbar);

  view_tabs_ = new QTabWidget(this);
  view_tabs_->setDocumentMode(true);
  view_tabs_->setTabPosition(QTabWidget::North);
  view_tabs_->setStyleSheet(
      style::sheet(QStringLiteral("tf_tree"), style::Tone::Frost));
  root->addWidget(view_tabs_, 1);

  auto* tree_page = new QWidget(view_tabs_);
  auto* tree_page_layout = new QVBoxLayout(tree_page);
  tree_page_layout->setContentsMargins(0, 0, 0, 0);
  tree_page_layout->setSpacing(0);

  auto* splitter = new QSplitter(Qt::Horizontal, tree_page);
  splitter->setChildrenCollapsible(false);
  splitter->setHandleWidth(1);
  splitter->setStyleSheet(
      style::sheet(QStringLiteral("tf_tree"), style::Tone::Frost));

  tree_ = new QTreeWidget(splitter);
  tree_->setHeaderHidden(true);
  tree_->setRootIsDecorated(true);
  tree_->setUniformRowHeights(true);
  tree_->setAnimated(false);
  tree_->setIndentation(18);
  tree_->setContextMenuPolicy(Qt::CustomContextMenu);
  tree_->setStyleSheet(
      style::sheet(QStringLiteral("tf_tree"), style::Tone::Frost));

  auto* detail_panel = new QFrame(splitter);
  detail_panel->setMinimumWidth(240);
  detail_panel->setStyleSheet(
      style::sheet(QStringLiteral("tf_tree"), style::Tone::Frost));
  auto* detail_layout = new QVBoxLayout(detail_panel);
  detail_layout->setContentsMargins(14, 16, 14, 14);
  detail_layout->setSpacing(12);

  auto* badge = new QLabel(tr("FRAME"), detail_panel);
  badge->setStyleSheet(
      style::sheet(QStringLiteral("tf_tree"), style::Tone::Frost));
  badge->setFixedWidth(56);
  detail_layout->addWidget(badge, 0, Qt::AlignLeft);

  detail_title_ = new QLabel(tr("Frame details"), detail_panel);
  detail_title_->setWordWrap(true);
  detail_title_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  detail_title_->setStyleSheet(
      style::type(style::Role::PanelBody, 15, 600));
  detail_layout->addWidget(detail_title_);

  detail_hint_ = new QLabel(tr("Select a frame to inspect transform metadata."),
                            detail_panel);
  detail_hint_->setWordWrap(true);
  detail_hint_->setStyleSheet(
      style::type(style::Role::PanelMuted, 12));
  detail_layout->addWidget(detail_hint_);

  detail_body_ = new QWidget(detail_panel);
  detail_body_->setStyleSheet(
      style::sheet(QStringLiteral("tf_tree"), style::Tone::Frost));
  auto* form = new QFormLayout(detail_body_);
  form->setContentsMargins(12, 12, 12, 12);
  form->setHorizontalSpacing(12);
  form->setVerticalSpacing(10);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignTop);
  detail_parent_value_ = MakeDetailValue(detail_body_);
  detail_type_value_ = MakeDetailValue(detail_body_);
  detail_authority_value_ = MakeDetailValue(detail_body_);
  detail_last_time_value_ = MakeDetailValue(detail_body_);
  detail_age_value_ = MakeDetailValue(detail_body_);
  detail_count_value_ = MakeDetailValue(detail_body_);
  form->addRow(MakeDetailLabel(tr("Parent"), detail_body_), detail_parent_value_);
  form->addRow(MakeDetailLabel(tr("Type"), detail_body_), detail_type_value_);
  form->addRow(MakeDetailLabel(tr("Authority"), detail_body_),
               detail_authority_value_);
  form->addRow(MakeDetailLabel(tr("Last transform"), detail_body_),
               detail_last_time_value_);
  form->addRow(MakeDetailLabel(tr("Time since update"), detail_body_),
               detail_age_value_);
  form->addRow(MakeDetailLabel(tr("Transforms received"), detail_body_),
               detail_count_value_);
  detail_body_->hide();
  detail_layout->addWidget(detail_body_);
  detail_layout->addStretch();

  splitter->addWidget(tree_);
  splitter->addWidget(detail_panel);
  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);
  tree_page_layout->addWidget(splitter, 1);
  view_tabs_->addTab(tree_page, tr("Tree"));

  graph_view_ = new TfTreeGraphView(view_tabs_);
  view_tabs_->addTab(graph_view_, tr("Graph"));

  auto* status_bar = new QFrame(this);
  ApplyPanelFooterChrome(status_bar);
  auto* status_layout = new QHBoxLayout(status_bar);
  status_layout->setContentsMargins(PanelChromeLayout::kFooterMarginH,
                                    PanelChromeLayout::kFooterMarginV,
                                    PanelChromeLayout::kFooterMarginH,
                                    PanelChromeLayout::kFooterMarginV);
  summary_label_ = new QLabel(status_bar);
  StylePanelStatusLabel(summary_label_);
  summary_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  status_layout->addWidget(summary_label_, 1);
  root->addWidget(status_bar);

  connect(filter_edit_, &QLineEdit::textChanged, this, &TfTreePanel::onFilterChanged);
  connect(stale_only_button_, &QToolButton::toggled, this,
          &TfTreePanel::onStaleOnlyToggled);
  connect(max_age_spin_, &QDoubleSpinBox::valueChanged, this,
          &TfTreePanel::onMaxAgeChanged);
  connect(tree_, &QTreeWidget::itemSelectionChanged, this,
          &TfTreePanel::onFrameSelectionChanged);
  connect(tree_, &QTreeWidget::customContextMenuRequested, this,
          &TfTreePanel::onTreeContextMenu);
  connect(graph_view_, &TfTreeGraphView::frameActivated, this, [this](const QString& frame_id) {
    if (tree_ == nullptr || frame_id.isEmpty()) {
      return;
    }
    const auto node_it = frame_nodes_.find(frame_id);
    if (node_it != frame_nodes_.end() && node_it->item != nullptr) {
      tree_->setCurrentItem(node_it->item);
      updateDetailsForItem(node_it->item);
    }
  });
  connect(fit_button, &QToolButton::clicked, this, [this]() {
    if (graph_view_ != nullptr) {
      graph_view_->zoomToFit();
      if (view_tabs_ != nullptr) {
        view_tabs_->setCurrentWidget(graph_view_);
      }
    }
  });
  connect(view_tabs_, &QTabWidget::currentChanged, this,
          &TfTreePanel::onTabChanged);
}

double TfTreePanel::currentTimeSec() const {
  return manager_ != nullptr ? manager_->simTimeSec() : 0.0;
}

QString TfTreePanel::formatTimestampSec(double sec) const {
  if (!(sec > 0.0) || !std::isfinite(sec)) {
    return tr("Unknown");
  }
  const QDateTime date_time = QDateTime::fromSecsSinceEpoch(
      static_cast<qint64>(sec), Qt::UTC);
  return date_time.toString(QStringLiteral("yyyy-MM-dd HH:mm:ss.zzz"));
}

QString TfTreePanel::formatAgeSec(double age_sec) const {
  if (age_sec < 0.0 || !std::isfinite(age_sec)) {
    return tr("Unknown");
  }
  if (age_sec < 0.05) {
    return tr("just now");
  }
  if (age_sec < 60.0) {
    return tr("%1 s ago").arg(QString::number(age_sec, 'f', 2));
  }
  if (age_sec < 3600.0) {
    return tr("%1 min ago").arg(QString::number(age_sec / 60.0, 'f', 1));
  }
  return tr("%1 h ago").arg(QString::number(age_sec / 3600.0, 'f', 1));
}

void TfTreePanel::setPaused(bool paused) {
  paused_ = paused;
  if (paused) {
    if (refresh_timer_ != nullptr) {
      refresh_timer_->stop();
    }
    refresh_pending_ = false;
    return;
  }
  scheduleRefresh();
}

void TfTreePanel::scheduleRefresh() {
  if (paused_) {
    return;
  }
  refresh_pending_ = true;
  if (refresh_timer_ != nullptr && !refresh_timer_->isActive()) {
    refresh_timer_->start();
  }
}

void TfTreePanel::refresh() {
  // Periodic tick / external callers: rate-limited like transforms-changed.
  scheduleRefresh();
}

QString TfTreePanel::structureFingerprint(
    const std::vector<transform::TfFrameStats>& frames) const {
  const QString filter =
      filter_edit_ != nullptr ? filter_edit_->text().trimmed() : QString();
  const bool stale_only =
      stale_only_button_ != nullptr && stale_only_button_->isChecked();
  const double max_age =
      max_age_spin_ != nullptr ? max_age_spin_->value() : kDefaultMaxAgeSec;
  const double now = currentTimeSec();
  QStringList parts;
  parts.reserve(static_cast<int>(frames.size()) + 3);
  parts.push_back(filter);
  parts.push_back(stale_only ? QStringLiteral("stale") : QStringLiteral("all"));
  parts.push_back(QString::number(max_age, 'f', 2));
  for (const transform::TfFrameStats& stats : frames) {
    QString token = QString::fromStdString(stats.frame_id) + QLatin1Char('>') +
                    NormalizeParent(QString::fromStdString(stats.parent_id)) +
                    (stats.is_static ? QLatin1Char('S') : QLatin1Char('D'));
    if (stale_only) {
      const double age = frameAgeSec(stats, now);
      const bool is_stale =
          !stats.is_static && age >= 0.0 && age > max_age &&
          age < kMaxPlausibleAgeSeconds;
      token += is_stale ? QLatin1Char('X') : QLatin1Char('F');
    }
    parts.push_back(token);
  }
  parts.sort(Qt::CaseSensitive);
  return parts.join(QLatin1Char('|'));
}

double TfTreePanel::frameAgeSec(const transform::TfFrameStats& stats,
                                double now_sec) {
  if (stats.is_static || stats.last_stamp_ns <= 0 || !(now_sec > 0.0)) {
    return -1.0;
  }
  const double stamp_sec =
      static_cast<double>(stats.last_stamp_ns) * 1e-9;
  return now_sec - stamp_sec;
}

bool TfTreePanel::framePassesFilters(
    const QString& frame_id, const QString& parent_id,
    const transform::TfFrameStats& stats, double now_sec) const {
  const QString filter =
      filter_edit_ != nullptr ? filter_edit_->text().trimmed() : QString();
  if (!filter.isEmpty() &&
      !frame_id.contains(filter, Qt::CaseInsensitive) &&
      !parent_id.contains(filter, Qt::CaseInsensitive)) {
    return false;
  }
  if (stale_only_button_ == nullptr || !stale_only_button_->isChecked()) {
    return true;
  }
  // Static transforms never age out — exclude them from Stale-only view.
  if (stats.is_static) {
    return false;
  }
  const double max_age =
      max_age_spin_ != nullptr ? max_age_spin_->value() : kDefaultMaxAgeSec;
  const double age = frameAgeSec(stats, now_sec);
  return age >= 0.0 && age > max_age && age < kMaxPlausibleAgeSeconds;
}

QString TfTreePanel::framePathForItem(const QTreeWidgetItem* item) {
  QStringList parts;
  for (const QTreeWidgetItem* cur = item; cur != nullptr; cur = cur->parent()) {
    parts.prepend(cur->data(0, kRoleFrameId).toString());
  }
  return parts.join(QLatin1Char('/'));
}

void TfTreePanel::updateStatsInPlace(
    const std::vector<transform::TfFrameStats>& frames) {
  const double now = currentTimeSec();
  std::vector<transform::TfFrameStats> visible;
  visible.reserve(frames.size());
  int root_count = 0;
  for (const transform::TfFrameStats& stats : frames) {
    const QString frame_id = QString::fromStdString(stats.frame_id);
    const QString parent =
        NormalizeParent(QString::fromStdString(stats.parent_id));
    if (!framePassesFilters(frame_id, parent, stats, now)) {
      continue;
    }
    visible.push_back(stats);
    auto it = frame_nodes_.find(frame_id);
    if (it == frame_nodes_.end()) {
      continue;
    }
    it->stats = stats;
    if (parent.isEmpty()) {
      ++root_count;
    }
  }
  updateSummaryLabel(static_cast<int>(visible.size()),
                     std::max(1, tree_ != nullptr ? tree_->topLevelItemCount()
                                                  : root_count));
  if (graph_view_ != nullptr && view_tabs_ != nullptr &&
      view_tabs_->currentWidget() == graph_view_) {
    // Text + stale already applied; pass empty filter to avoid double-filter.
    graph_view_->setFrames(visible, now, QString());
  }
  if (tree_ != nullptr) {
    updateDetailsForItem(tree_->currentItem());
  }
}

void TfTreePanel::onCoalescedRefresh() {
  if (!refresh_pending_ && !force_rebuild_) {
    return;
  }
  refresh_pending_ = false;

  // Skip work while the dock is hidden; showEvent will force a rebuild.
  if (!isVisible() && !force_rebuild_) {
    return;
  }

  if (tf_buffer_ == nullptr) {
    force_rebuild_ = false;
    structure_fingerprint_.clear();
    tree_->clear();
    frame_nodes_.clear();
    if (graph_view_ != nullptr) {
      graph_view_->showMessage(tr("TF buffer unavailable."));
    }
    updateSummaryLabel(0, 0);
    detail_body_->hide();
    detail_hint_->show();
    detail_hint_->setText(tr("TF buffer unavailable."));
    return;
  }

  const std::vector<transform::TfFrameStats> frames = tf_buffer_->frameStats();
  const QString fingerprint = structureFingerprint(frames);
  if (!force_rebuild_ && fingerprint == structure_fingerprint_ &&
      !frame_nodes_.isEmpty()) {
    updateStatsInPlace(frames);
    return;
  }
  force_rebuild_ = false;
  structure_fingerprint_ = fingerprint;
  rebuildTree();
}

void TfTreePanel::rebuildTree() {
  selected_frame_id_.clear();
  if (tree_->currentItem() != nullptr) {
    selected_frame_id_ = tree_->currentItem()->data(0, kRoleFrameId).toString();
  }

  const QSignalBlocker blocker(tree_);
  tree_->clear();
  frame_nodes_.clear();

  if (tf_buffer_ == nullptr) {
    if (graph_view_ != nullptr) {
      graph_view_->showMessage(tr("TF buffer unavailable."));
    }
    updateSummaryLabel(0, 0);
    detail_body_->hide();
    detail_hint_->show();
    detail_hint_->setText(tr("TF buffer unavailable."));
    return;
  }

  const std::vector<transform::TfFrameStats> frames = tf_buffer_->frameStats();
  structure_fingerprint_ = structureFingerprint(frames);
  if (frames.empty()) {
    if (graph_view_ != nullptr) {
      graph_view_->showMessage(tr("No transforms received yet."));
    }
    updateSummaryLabel(0, 0);
    detail_body_->hide();
    detail_hint_->show();
    detail_hint_->setText(tr("No transforms received yet."));
    return;
  }

  const double now = currentTimeSec();
  std::vector<transform::TfFrameStats> visible;
  visible.reserve(frames.size());
  QHash<QString, QTreeWidgetItem*> item_by_frame;
  int root_count = 0;

  for (const transform::TfFrameStats& stats : frames) {
    const QString frame_id = QString::fromStdString(stats.frame_id);
    const QString parent = NormalizeParent(QString::fromStdString(stats.parent_id));
    if (!framePassesFilters(frame_id, parent, stats, now)) {
      continue;
    }
    visible.push_back(stats);
    auto* item = new QTreeWidgetItem({frame_id});
    item->setData(0, kRoleFrameId, frame_id);
    if (stats.is_static) {
      item->setText(0, frame_id + QStringLiteral("  · static"));
      item->setToolTip(0, tr("%1 (static transform)").arg(frame_id));
      item->setForeground(0, QBrush(QColor(kStatic)));
    } else {
      item->setToolTip(0, frame_id);
      item->setForeground(0, QBrush(QColor(kText)));
    }
    frame_nodes_.insert(frame_id, FrameNode{stats, item});
    item_by_frame.insert(frame_id, item);
  }

  for (auto it = item_by_frame.begin(); it != item_by_frame.end(); ++it) {
    const QString frame_id = it.key();
    const transform::TfFrameStats stats = frame_nodes_.value(frame_id).stats;
    QTreeWidgetItem* item = it.value();
    const QString parent = NormalizeParent(QString::fromStdString(stats.parent_id));
    if (!parent.isEmpty() && item_by_frame.contains(parent)) {
      item_by_frame.value(parent)->addChild(item);
    } else {
      tree_->addTopLevelItem(item);
      ++root_count;
    }
  }

  for (int i = 0; i < tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* root = tree_->topLevelItem(i);
    std::function<void(QTreeWidgetItem*)> expand_all = [&](QTreeWidgetItem* node) {
      node->setExpanded(true);
      for (int c = 0; c < node->childCount(); ++c) {
        expand_all(node->child(c));
      }
    };
    expand_all(root);
  }

  tree_->sortItems(0, Qt::AscendingOrder);
  updateSummaryLabel(static_cast<int>(visible.size()), root_count);
  if (graph_view_ != nullptr) {
    graph_view_->requestFit();
    // Text + stale already applied; pass empty filter to avoid double-filter.
    graph_view_->setFrames(visible, now, QString());
  }

  if (!selected_frame_id_.isEmpty()) {
    const auto nodes = frame_nodes_.values();
    for (const FrameNode& node : nodes) {
      if (node.item != nullptr &&
          node.item->data(0, kRoleFrameId).toString() == selected_frame_id_) {
        tree_->setCurrentItem(node.item);
        break;
      }
    }
  }

  if (tree_->currentItem() == nullptr && tree_->topLevelItemCount() > 0) {
    tree_->setCurrentItem(tree_->topLevelItem(0));
  }
  onFrameSelectionChanged();
}

void TfTreePanel::updateSummaryLabel(int frame_count, int tree_count) {
  QString text;
  if (frame_count == 0) {
    text = tr("No frames");
  } else if (tree_count <= 1) {
    text = tr("%1 frames").arg(frame_count);
  } else {
    text = tr("%1 frames · %2 disconnected trees")
               .arg(frame_count)
               .arg(tree_count);
  }
  if (summary_label_->text() != text) {
    summary_label_->setText(text);
  }
}

void TfTreePanel::onFilterChanged(const QString& /*text*/) {
  force_rebuild_ = true;
  refresh_pending_ = true;
  onCoalescedRefresh();
  emitConfigChanged();
}

void TfTreePanel::onStaleOnlyToggled(bool checked) {
  if (max_age_spin_ != nullptr) {
    max_age_spin_->setEnabled(checked);
  }
  force_rebuild_ = true;
  refresh_pending_ = true;
  onCoalescedRefresh();
  emitConfigChanged();
}

void TfTreePanel::onMaxAgeChanged(double /*value*/) {
  if (stale_only_button_ == nullptr || !stale_only_button_->isChecked()) {
    return;
  }
  force_rebuild_ = true;
  refresh_pending_ = true;
  onCoalescedRefresh();
  emitConfigChanged();
}

void TfTreePanel::onTabChanged(int /*index*/) {
  force_rebuild_ = true;
  refresh_pending_ = true;
  onCoalescedRefresh();
  emitConfigChanged();
}

void TfTreePanel::onTreeContextMenu(const QPoint& pos) {
  if (tree_ == nullptr) {
    return;
  }
  QTreeWidgetItem* item = tree_->itemAt(pos);
  if (item == nullptr) {
    return;
  }
  const QString frame_id = item->data(0, kRoleFrameId).toString();
  if (frame_id.isEmpty()) {
    return;
  }
  const QString path = framePathForItem(item);
  QMenu menu(tree_);
  QAction* copy_id = menu.addAction(tr("Copy frame id"));
  QAction* copy_path = menu.addAction(tr("Copy frame path"));
  QAction* chosen = menu.exec(tree_->viewport()->mapToGlobal(pos));
  if (chosen == nullptr) {
    return;
  }
  QClipboard* clipboard = QApplication::clipboard();
  if (clipboard == nullptr) {
    return;
  }
  if (chosen == copy_id) {
    clipboard->setText(frame_id);
  } else if (chosen == copy_path) {
    clipboard->setText(path);
  }
}

void TfTreePanel::onFrameSelectionChanged() {
  if (graph_view_ != nullptr) {
    const QTreeWidgetItem* item = tree_ != nullptr ? tree_->currentItem() : nullptr;
    graph_view_->setCurrentFrame(item != nullptr
                                     ? item->data(0, kRoleFrameId).toString()
                                     : QString());
  }
  updateDetailsForItem(tree_->currentItem());
}

void TfTreePanel::updateDetailsForItem(QTreeWidgetItem* item) {
  auto set_if_changed = [](QLabel* label, const QString& text) {
    if (label != nullptr && label->text() != text) {
      label->setText(text);
    }
  };

  if (item == nullptr) {
    detail_body_->hide();
    detail_hint_->show();
    set_if_changed(detail_hint_,
                   tr("Select a frame to inspect transform metadata."));
    set_if_changed(detail_title_, tr("Frame details"));
    return;
  }

  const QString frame_id = item->data(0, kRoleFrameId).toString();
  const FrameNode node = frame_nodes_.value(frame_id);
  set_if_changed(detail_title_, frame_id);
  detail_hint_->hide();
  detail_body_->show();

  const QString parent = NormalizeParent(QString::fromStdString(node.stats.parent_id));
  set_if_changed(detail_parent_value_,
                 parent.isEmpty() ? tr("(root)") : parent);
  set_if_changed(detail_type_value_,
                 node.stats.is_static ? tr("Static") : tr("Dynamic"));
  if (detail_type_value_ != nullptr) {
    detail_type_value_->setStyleSheet(style::sheet(
        node.stats.is_static ? QStringLiteral("tf_tree/detail_type_static")
                             : QStringLiteral("tf_tree/detail_type_dynamic"),
        style::Tone::Frost));
  }
  set_if_changed(
      detail_authority_value_,
      node.stats.authority.empty() ? tr("Unknown")
                                   : QString::fromStdString(node.stats.authority));

  const double stamp_sec =
      static_cast<double>(node.stats.last_stamp_ns) / 1e9;
  set_if_changed(detail_last_time_value_, formatTimestampSec(stamp_sec));

  if (node.stats.is_static) {
    set_if_changed(detail_age_value_, tr("Static transform"));
  } else if (node.stats.last_stamp_ns <= 0) {
    set_if_changed(detail_age_value_, tr("Unknown"));
  } else {
    const double age = currentTimeSec() - stamp_sec;
    set_if_changed(detail_age_value_, formatAgeSec(age));
  }

  set_if_changed(detail_count_value_,
                 node.stats.transforms_received > 0
                     ? QString::number(node.stats.transforms_received)
                     : tr("0"));
}

}  // namespace autoviz
