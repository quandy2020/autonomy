/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/record/record_panel.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QMouseEvent>
#include <QPainter>
#include <QPen>
#include <QSizePolicy>
#include <QSlider>
#include <QTimer>
#include <QToolButton>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/playback_controller.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/panel/title_tools.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

constexpr int kTimelineTicks = 1000;
constexpr int kColumnChannel = 0;
constexpr int kColumnType = 1;
constexpr int kColumnCount = 2;
constexpr double kStepSec = 0.1;
constexpr int kScrubThrottleMs = 40;
constexpr int kDensityHandleHitPx = 6;

QToolButton* MakeTransportButton(QWidget* parent, const QString& text,
                                 const QString& tip) {
  auto* button = new QToolButton(parent);
  button->setText(text);
  button->setToolTip(tip);
  button->setAutoRaise(true);
  button->setFocusPolicy(Qt::NoFocus);
  return button;
}

}  // namespace

class RecordPanel::DensityBar : public QWidget {
 public:
  using SeekCallback = std::function<void(double /*normalized*/, bool /*final*/)>;
  using RangeCallback = std::function<void(double /*start*/, double /*end*/)>;

  explicit DensityBar(QWidget* parent = nullptr) : QWidget(parent) {
    setFixedHeight(22);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    setMouseTracking(true);
    setCursor(Qt::PointingHandCursor);
    setToolTip(QObject::tr(
        "Click to seek · drag to scrub · drag orange handles to set In/Out"));
  }

  void setBins(std::vector<float> bins) {
    bins_ = std::move(bins);
    update();
  }

  void setRangeNormalized(double start, double end) {
    range_start_ = std::clamp(start, 0.0, 1.0);
    range_end_ = std::clamp(end, 0.0, 1.0);
    if (range_end_ < range_start_) {
      std::swap(range_start_, range_end_);
    }
    update();
  }

  void setPlayheadNormalized(double t) {
    playhead_ = std::clamp(t, 0.0, 1.0);
    update();
  }

  void setSeekCallback(SeekCallback cb) { seek_cb_ = std::move(cb); }
  void setRangeCallback(RangeCallback cb) { range_cb_ = std::move(cb); }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);
    const QRect r = rect().adjusted(0, 1, 0, -1);
    painter.fillRect(r, QColor(0xee, 0xf1, 0xf4));

    if (!bins_.empty()) {
      const int n = static_cast<int>(bins_.size());
      for (int i = 0; i < n; ++i) {
        const int x0 = r.left() + (i * r.width()) / n;
        const int x1 = r.left() + ((i + 1) * r.width()) / n;
        const int h = static_cast<int>(std::lround(bins_[static_cast<size_t>(i)] *
                                                   static_cast<float>(r.height())));
        if (h <= 0) {
          continue;
        }
        painter.fillRect(x0, r.bottom() - h + 1, std::max(1, x1 - x0), h,
                         QColor(0x54, 0xa2, 0xff, 160));
      }
    }

    const int x_in =
        r.left() + static_cast<int>(std::lround(range_start_ * r.width()));
    const int x_out =
        r.left() + static_cast<int>(std::lround(range_end_ * r.width()));
    if (range_end_ > range_start_ + 1e-6 &&
        (range_start_ > 1e-6 || range_end_ < 1.0 - 1e-6)) {
      painter.fillRect(r.left(), r.top(), x_in - r.left(), r.height(),
                       QColor(0, 0, 0, 35));
      painter.fillRect(x_out, r.top(), r.right() - x_out + 1, r.height(),
                       QColor(0, 0, 0, 35));
    }
    painter.setPen(QPen(QColor(0xd4, 0x6b, 0x08), 2));
    painter.drawLine(x_in, r.top(), x_in, r.bottom());
    painter.drawLine(x_out, r.top(), x_out, r.bottom());
    painter.fillRect(x_in - 2, r.top(), 5, 5, QColor(0xd4, 0x6b, 0x08));
    painter.fillRect(x_out - 2, r.bottom() - 4, 5, 5, QColor(0xd4, 0x6b, 0x08));

    const int x_ph =
        r.left() + static_cast<int>(std::lround(playhead_ * r.width()));
    painter.setPen(QPen(QColor(0x1f, 0x23, 0x28), 1));
    painter.drawLine(x_ph, r.top(), x_ph, r.bottom());
  }

  void mousePressEvent(QMouseEvent* event) override {
    if (event->button() != Qt::LeftButton || width() <= 1) {
      QWidget::mousePressEvent(event);
      return;
    }
    const double n = xToNormalized(event->position().x());
    const Hit hit = hitTest(event->position().x());
    drag_ = hit;
    if (drag_ == Hit::RangeStart || drag_ == Hit::RangeEnd) {
      applyRangeDrag(n);
    } else {
      drag_ = Hit::Seek;
      if (seek_cb_) {
        seek_cb_(n, false);
      }
    }
    event->accept();
  }

  void mouseMoveEvent(QMouseEvent* event) override {
    if (drag_ == Hit::None) {
      const Hit hit = hitTest(event->position().x());
      setCursor(hit == Hit::RangeStart || hit == Hit::RangeEnd
                    ? Qt::SizeHorCursor
                    : Qt::PointingHandCursor);
      QWidget::mouseMoveEvent(event);
      return;
    }
    const double n = xToNormalized(event->position().x());
    if (drag_ == Hit::Seek) {
      if (seek_cb_) {
        seek_cb_(n, false);
      }
    } else {
      applyRangeDrag(n);
    }
    event->accept();
  }

  void mouseReleaseEvent(QMouseEvent* event) override {
    if (event->button() == Qt::LeftButton && drag_ != Hit::None) {
      const double n = xToNormalized(event->position().x());
      if (drag_ == Hit::Seek && seek_cb_) {
        seek_cb_(n, true);
      } else if (drag_ != Hit::Seek) {
        applyRangeDrag(n);
      }
      drag_ = Hit::None;
      event->accept();
      return;
    }
    QWidget::mouseReleaseEvent(event);
  }

 private:
  enum class Hit { None, Seek, RangeStart, RangeEnd };

  double xToNormalized(double x) const {
    if (width() <= 1) {
      return 0.0;
    }
    return std::clamp(x / static_cast<double>(width() - 1), 0.0, 1.0);
  }

  Hit hitTest(double x) const {
    const double w = static_cast<double>(std::max(1, width() - 1));
    const double x_in = range_start_ * w;
    const double x_out = range_end_ * w;
    if (std::abs(x - x_in) <= kDensityHandleHitPx) {
      return Hit::RangeStart;
    }
    if (std::abs(x - x_out) <= kDensityHandleHitPx) {
      return Hit::RangeEnd;
    }
    return Hit::Seek;
  }

  void applyRangeDrag(double n) {
    constexpr double kMinSpan = 0.01;
    if (drag_ == Hit::RangeStart) {
      range_start_ = std::clamp(n, 0.0, range_end_ - kMinSpan);
    } else if (drag_ == Hit::RangeEnd) {
      range_end_ = std::clamp(n, range_start_ + kMinSpan, 1.0);
    }
    update();
    if (range_cb_) {
      range_cb_(range_start_, range_end_);
    }
  }

  std::vector<float> bins_;
  double range_start_ = 0.0;
  double range_end_ = 1.0;
  double playhead_ = 0.0;
  Hit drag_ = Hit::None;
  SeekCallback seek_cb_;
  RangeCallback range_cb_;
};

RecordPanel::RecordPanel(common::VisualizationManager* manager, QWidget* parent)
    : QWidget(parent), manager_(manager) {
  setFocusPolicy(Qt::StrongFocus);
  ApplyPanelShell(this);
  applyChromeStyles();

  open_button_ = new QToolButton(this);
  open_button_->setText(tr("Open"));
  open_button_->setToolTip(tr("Open an Autolink .record / .bag / .mcap file"));
  open_button_->setIcon(IconLoader::menuIcon(QStringLiteral("file.open_record")));
  open_button_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
  open_button_->setAutoRaise(true);

  seek_start_button_ =
      MakeTransportButton(this, QStringLiteral("|◀"), tr("Seek to range start (Home)"));
  step_back_button_ = MakeTransportButton(
      this, QStringLiteral("◀"), tr("Step back 100 ms (Left)"));
  play_pause_button_ =
      MakeTransportButton(this, tr("Play"), tr("Play / pause (Space)"));
  step_forward_button_ = MakeTransportButton(
      this, QStringLiteral("▶"), tr("Step forward 100 ms (Right)"));
  seek_end_button_ =
      MakeTransportButton(this, QStringLiteral("▶|"), tr("Seek to range end (End)"));
  stop_button_ =
      MakeTransportButton(this, tr("Stop"), tr("Stop and reset to range start"));
  prev_msg_button_ = MakeTransportButton(
      this, QStringLiteral("[◀]"),
      tr("Previous message (Alt+Left); uses selected channel if any"));
  next_msg_button_ = MakeTransportButton(
      this, QStringLiteral("[▶]"),
      tr("Next message (Alt+Right); uses selected channel if any"));

  loop_check_ = new QCheckBox(tr("Loop"), this);
  loop_check_->setToolTip(tr("Restart at range start when playback ends"));

  rate_combo_ = new QComboBox(this);
  rate_combo_->setToolTip(tr("Playback speed"));
  for (const char* label :
       {"0.25x", "0.5x", "1.0x", "1.5x", "2.0x", "4.0x", "8.0x"}) {
    rate_combo_->addItem(QLatin1String(label));
  }
  rate_combo_->setCurrentIndex(2);

  auto* transport = new QHBoxLayout();
  transport->setContentsMargins(0, 0, 0, 0);
  transport->setSpacing(3);
  transport->addWidget(open_button_);
  transport->addWidget(seek_start_button_);
  transport->addWidget(step_back_button_);
  transport->addWidget(play_pause_button_);
  transport->addWidget(step_forward_button_);
  transport->addWidget(seek_end_button_);
  transport->addWidget(stop_button_);
  transport->addWidget(prev_msg_button_);
  transport->addWidget(next_msg_button_);
  transport->addWidget(loop_check_);
  transport->addStretch(1);
  transport->addWidget(new QLabel(tr("Rate"), this));
  transport->addWidget(rate_combo_);

  file_label_ = new QLabel(tr("No record loaded"), this);
  file_label_->setWordWrap(true);
  StyleHintLabel(file_label_);

  density_bar_ = new DensityBar(this);
  density_bar_->setSeekCallback([this](double normalized, bool final) {
    if (manager_ == nullptr || !ensureFileOpen()) {
      return;
    }
    integration::PlaybackController& playback = manager_->playback();
    const double total = playback.totalTimeSec();
    if (total <= 0.0) {
      return;
    }
    if (!seeking_) {
      scrub_resume_play_ =
          playback.isPlaying() && !playback.isPaused();
      if (scrub_resume_play_) {
        playback.pause();
      }
      seeking_ = true;
    }
    const double t = std::clamp(normalized, 0.0, 1.0) * total;
    timeline_->blockSignals(true);
    timeline_->setValue(static_cast<int>(
        std::lround(std::clamp(normalized, 0.0, 1.0) * kTimelineTicks)));
    timeline_->blockSignals(false);
    const int pct =
        static_cast<int>(std::lround(std::clamp(normalized, 0.0, 1.0) * 100.0));
    time_label_->setText(tr("%1 / %2 (%3%)")
                             .arg(FormatClock(t), FormatClock(total))
                             .arg(pct));
    density_bar_->setPlayheadNormalized(normalized);
    if (final) {
      seeking_ = false;
      last_scrub_ms_ = 0;
      playback.seekTo(t);
      if (scrub_resume_play_) {
        scrub_resume_play_ = false;
        if (playback.isPaused()) {
          playback.resume();
        }
      }
      syncTimelineFromPlayback();
      syncTransportButtons();
      return;
    }
    const qint64 now = QDateTime::currentMSecsSinceEpoch();
    if (last_scrub_ms_ != 0 && now - last_scrub_ms_ < kScrubThrottleMs) {
      return;
    }
    last_scrub_ms_ = now;
    playback.scrubTo(t);
  });
  density_bar_->setRangeCallback([this](double start_n, double end_n) {
    if (manager_ == nullptr || !ensureFileOpen()) {
      return;
    }
    integration::PlaybackController& playback = manager_->playback();
    const double total = playback.totalTimeSec();
    if (total <= 0.0) {
      return;
    }
    playback.setPlaybackRange(start_n * total, end_n * total);
    syncRangeLabel();
    syncDensityBar();
    syncTimelineFromPlayback();
  });

  timeline_ = new QSlider(Qt::Horizontal, this);
  timeline_->setRange(0, kTimelineTicks);
  timeline_->setValue(0);
  timeline_->setEnabled(false);
  timeline_->setToolTip(tr("Drag to seek"));

  time_label_ = new QLabel(QStringLiteral("00:00.0 / 00:00.0"), this);
  time_label_->setMinimumWidth(150);
  time_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

  auto* timeline_row = new QHBoxLayout();
  timeline_row->setContentsMargins(0, 0, 0, 0);
  timeline_row->setSpacing(8);
  timeline_row->addWidget(timeline_, 1);
  timeline_row->addWidget(time_label_);

  range_in_button_ = MakeTransportButton(
      this, tr("In"), tr("Set range start to playhead (Foxglove trim)"));
  range_out_button_ = MakeTransportButton(
      this, tr("Out"), tr("Set range end to playhead (Foxglove trim)"));
  range_clear_button_ =
      MakeTransportButton(this, tr("Clear"), tr("Clear playback range"));
  range_label_ = new QLabel(tr("Range: full"), this);
  StyleHintLabel(range_label_);

  auto* range_row = new QHBoxLayout();
  range_row->setContentsMargins(0, 0, 0, 0);
  range_row->setSpacing(6);
  range_row->addWidget(range_in_button_);
  range_row->addWidget(range_out_button_);
  range_row->addWidget(range_clear_button_);
  range_row->addWidget(range_label_, 1);

  filter_edit_ = new QLineEdit(this);
  filter_edit_->setPlaceholderText(tr("Filter channels…"));
  filter_edit_->setClearButtonEnabled(true);
  StyleFilterLineEdit(filter_edit_);

  select_all_button_ =
      MakeTransportButton(this, tr("All"), tr("Enable all channels"));
  select_none_button_ =
      MakeTransportButton(this, tr("None"), tr("Mute all channels"));

  auto* filter_row = new QHBoxLayout();
  filter_row->setContentsMargins(0, 0, 0, 0);
  filter_row->setSpacing(6);
  filter_row->addWidget(filter_edit_, 1);
  filter_row->addWidget(select_all_button_);
  filter_row->addWidget(select_none_button_);

  channels_header_ = new QLabel(tr("Channels"), this);
  StyleSectionTitle(channels_header_);

  channel_tree_ = new QTreeWidget(this);
  channel_tree_->setColumnCount(3);
  channel_tree_->setHeaderLabels({tr("Channel"), tr("Type"), tr("#")});
  channel_tree_->setRootIsDecorated(false);
  channel_tree_->setUniformRowHeights(true);
  channel_tree_->setAlternatingRowColors(true);
  channel_tree_->setSelectionMode(QAbstractItemView::SingleSelection);
  channel_tree_->setToolTip(
      tr("Uncheck to mute. Double-click → Messages. Selection scopes [◀]/[▶]."));
  channel_tree_->header()->setStretchLastSection(false);
  channel_tree_->header()->setSectionResizeMode(kColumnChannel,
                                                QHeaderView::Stretch);
  channel_tree_->header()->setSectionResizeMode(kColumnType,
                                                QHeaderView::ResizeToContents);
  channel_tree_->header()->setSectionResizeMode(kColumnCount,
                                                QHeaderView::ResizeToContents);
  StylePanelTree(channel_tree_);

  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(8, 8, 8, 8);
  layout->setSpacing(6);
  layout->addLayout(transport);
  layout->addWidget(file_label_);
  layout->addWidget(density_bar_);
  layout->addLayout(timeline_row);
  layout->addLayout(range_row);
  layout->addLayout(filter_row);
  layout->addWidget(channels_header_);
  layout->addWidget(channel_tree_, 1);

  connect(open_button_, &QToolButton::clicked, this,
          &RecordPanel::onOpenClicked);
  connect(seek_start_button_, &QToolButton::clicked, this,
          &RecordPanel::onSeekStartClicked);
  connect(step_back_button_, &QToolButton::clicked, this,
          &RecordPanel::onStepBackClicked);
  connect(play_pause_button_, &QToolButton::clicked, this,
          &RecordPanel::onPlayPauseClicked);
  connect(step_forward_button_, &QToolButton::clicked, this,
          &RecordPanel::onStepForwardClicked);
  connect(seek_end_button_, &QToolButton::clicked, this,
          &RecordPanel::onSeekEndClicked);
  connect(stop_button_, &QToolButton::clicked, this, &RecordPanel::onStopClicked);
  connect(prev_msg_button_, &QToolButton::clicked, this,
          &RecordPanel::onStepPrevMessage);
  connect(next_msg_button_, &QToolButton::clicked, this,
          &RecordPanel::onStepNextMessage);
  connect(range_in_button_, &QToolButton::clicked, this,
          &RecordPanel::onSetRangeStart);
  connect(range_out_button_, &QToolButton::clicked, this,
          &RecordPanel::onSetRangeEnd);
  connect(range_clear_button_, &QToolButton::clicked, this,
          &RecordPanel::onClearRange);
  connect(loop_check_, &QCheckBox::toggled, this, &RecordPanel::onLoopToggled);
  connect(rate_combo_, qOverload<int>(&QComboBox::activated), this,
          &RecordPanel::onRateChanged);
  connect(timeline_, &QSlider::sliderPressed, this, &RecordPanel::onSeekPressed);
  connect(timeline_, &QSlider::sliderReleased, this,
          &RecordPanel::onSeekReleased);
  connect(timeline_, &QSlider::valueChanged, this, &RecordPanel::onSeekMoved);
  connect(channel_tree_, &QTreeWidget::itemChanged, this,
          &RecordPanel::onChannelItemChanged);
  connect(channel_tree_, &QTreeWidget::itemDoubleClicked, this,
          &RecordPanel::onChannelDoubleClicked);
  connect(filter_edit_, &QLineEdit::textChanged, this,
          &RecordPanel::onFilterTextChanged);
  connect(select_all_button_, &QToolButton::clicked, this,
          &RecordPanel::onSelectAllChannels);
  connect(select_none_button_, &QToolButton::clicked, this,
          &RecordPanel::onSelectNoneChannels);

  tick_timer_ = new QTimer(this);
  connect(tick_timer_, &QTimer::timeout, this, &RecordPanel::onTick);
  tick_timer_->start(50);

  syncTransportButtons();
}

void RecordPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("RecordDock");
  callbacks.change_panel = [this](const QString& object_name) {
    emit panelChangeRequested(object_name);
  };
  callbacks.expand = [this]() { emit panelExpandRequested(); };
  callbacks.remove = [this]() { emit panelRemoveRequested(); };

  PanelTitleBarOptions options;
  options.show_split = false;
  options.show_change = true;
  options.show_expand = true;
  options.on_expand = [this]() { emit panelExpandRequested(); };

  const PanelTitleBarTools tools =
      CreatePanelTitleBarTools(dock, callbacks, options);
  expand_button_ = tools.expand_button;
  dock->setTitleBarTools(tools.widget);
}

void RecordPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ != nullptr) {
    expand_button_->setChecked(checked);
  }
}

void RecordPanel::reloadFromPlayback() {
  if (manager_ == nullptr) {
    return;
  }
  integration::PlaybackController& playback = manager_->playback();
  const std::string& path = playback.currentFile();
  if (path != loaded_file_) {
    loaded_file_ = path;
    rebuildChannelTree();
    syncDensityBar();
  }
  if (path.empty()) {
    file_label_->setText(tr("No record loaded"));
    timeline_->setEnabled(false);
    timeline_->setValue(0);
    time_label_->setText(QStringLiteral("00:00.0 / 00:00.0"));
    channels_header_->setText(tr("Channels"));
  } else {
    file_label_->setText(QFileInfo(QString::fromStdString(path)).fileName());
    timeline_->setEnabled(playback.totalTimeSec() > 0.0);
    channels_header_->setText(
        tr("Channels (%1)").arg(playback.channelCount()));
  }
  loop_check_->blockSignals(true);
  loop_check_->setChecked(playback.loop());
  loop_check_->blockSignals(false);

  const QString rate_text =
      QString::number(playback.playRate(), 'f', 2).replace(QStringLiteral(".00"),
                                                           QStringLiteral(".0")) +
      QLatin1Char('x');
  int rate_index = rate_combo_->findText(rate_text);
  if (rate_index < 0) {
    rate_index = rate_combo_->findText(
        QString::number(playback.playRate(), 'f', 1) + QLatin1Char('x'));
  }
  if (rate_index >= 0) {
    rate_combo_->blockSignals(true);
    rate_combo_->setCurrentIndex(rate_index);
    rate_combo_->blockSignals(false);
  }

  syncTimelineFromPlayback();
  syncRangeLabel();
  syncDensityBar();
  syncTransportButtons();
}

void RecordPanel::keyPressEvent(QKeyEvent* event) {
  if (event == nullptr) {
    return;
  }
  const bool alt = event->modifiers() & Qt::AltModifier;
  switch (event->key()) {
    case Qt::Key_Space:
      onPlayPauseClicked();
      event->accept();
      return;
    case Qt::Key_Left:
      if (alt) {
        onStepPrevMessage();
      } else {
        onStepBackClicked();
      }
      event->accept();
      return;
    case Qt::Key_Right:
      if (alt) {
        onStepNextMessage();
      } else {
        onStepForwardClicked();
      }
      event->accept();
      return;
    case Qt::Key_Home:
      onSeekStartClicked();
      event->accept();
      return;
    case Qt::Key_End:
      onSeekEndClicked();
      event->accept();
      return;
    case Qt::Key_I:
      if (alt) {
        onSetRangeStart();
        event->accept();
        return;
      }
      break;
    case Qt::Key_O:
      if (alt) {
        onSetRangeEnd();
        event->accept();
        return;
      }
      break;
    default:
      break;
  }
  QWidget::keyPressEvent(event);
}

void RecordPanel::onOpenClicked() { emit openRecordRequested(); }

void RecordPanel::onPlayPauseClicked() {
  if (!ensureFileOpen()) {
    return;
  }
  integration::PlaybackController& playback = manager_->playback();
  if (!playback.isPlaying()) {
    applyExcludedChannelsFromTree();
    const double rate = rate_combo_->currentText().chopped(1).toDouble();
    playback.setPlayRate(rate > 0.0 ? rate : 1.0);
    playback.setLoop(loop_check_->isChecked());
    playback.play(playback.playRate(), playback.loop());
  } else if (playback.isPaused()) {
    playback.resume();
  } else {
    playback.pause();
  }
  syncTransportButtons();
}

void RecordPanel::onStopClicked() {
  if (manager_ == nullptr) {
    return;
  }
  manager_->playback().stop();
  syncTimelineFromPlayback();
  syncTransportButtons();
}

void RecordPanel::onSeekStartClicked() {
  if (!ensureFileOpen()) {
    return;
  }
  manager_->playback().seekTo(manager_->playback().rangeStartSec());
  syncTimelineFromPlayback();
  syncTransportButtons();
}

void RecordPanel::onSeekEndClicked() {
  if (!ensureFileOpen()) {
    return;
  }
  manager_->playback().seekTo(manager_->playback().rangeEndSec());
  syncTimelineFromPlayback();
  syncTransportButtons();
}

void RecordPanel::onStepBackClicked() {
  if (!ensureFileOpen()) {
    return;
  }
  seekByDelta(-kStepSec);
}

void RecordPanel::onStepForwardClicked() {
  if (!ensureFileOpen()) {
    return;
  }
  seekByDelta(kStepSec);
}

void RecordPanel::onStepPrevMessage() {
  if (!ensureFileOpen()) {
    return;
  }
  manager_->playback().stepMessage(false, selectedChannel());
  syncTimelineFromPlayback();
  syncTransportButtons();
}

void RecordPanel::onStepNextMessage() {
  if (!ensureFileOpen()) {
    return;
  }
  manager_->playback().stepMessage(true, selectedChannel());
  syncTimelineFromPlayback();
  syncTransportButtons();
}

void RecordPanel::onSetRangeStart() {
  if (!ensureFileOpen()) {
    return;
  }
  integration::PlaybackController& playback = manager_->playback();
  playback.setPlaybackRange(playback.currentTimeSec(), playback.rangeEndSec());
  syncRangeLabel();
  syncDensityBar();
  syncTimelineFromPlayback();
}

void RecordPanel::onSetRangeEnd() {
  if (!ensureFileOpen()) {
    return;
  }
  integration::PlaybackController& playback = manager_->playback();
  playback.setPlaybackRange(playback.rangeStartSec(), playback.currentTimeSec());
  syncRangeLabel();
  syncDensityBar();
  syncTimelineFromPlayback();
}

void RecordPanel::onClearRange() {
  if (manager_ == nullptr) {
    return;
  }
  manager_->playback().clearPlaybackRange();
  syncRangeLabel();
  syncDensityBar();
  syncTimelineFromPlayback();
}

void RecordPanel::onLoopToggled(bool checked) {
  if (manager_ == nullptr) {
    return;
  }
  manager_->playback().setLoop(checked);
}

void RecordPanel::onRateChanged(int /*index*/) {
  if (manager_ == nullptr) {
    return;
  }
  const double rate = rate_combo_->currentText().chopped(1).toDouble();
  if (rate > 0.0) {
    manager_->playback().setPlayRate(rate);
  }
}

void RecordPanel::onSeekPressed() {
  seeking_ = true;
  scrub_resume_play_ = false;
  last_scrub_ms_ = 0;
  if (manager_ == nullptr) {
    return;
  }
  integration::PlaybackController& playback = manager_->playback();
  scrub_resume_play_ = playback.isPlaying() && !playback.isPaused();
  if (scrub_resume_play_) {
    playback.pause();
  }
}

void RecordPanel::onSeekReleased() {
  if (manager_ == nullptr) {
    seeking_ = false;
    scrub_resume_play_ = false;
    return;
  }
  const double normalized =
      static_cast<double>(timeline_->value()) / kTimelineTicks;
  integration::PlaybackController& playback = manager_->playback();
  const double total = playback.totalTimeSec();
  const double t =
      total > 0.0 ? std::clamp(normalized, 0.0, 1.0) * total : 0.0;
  seeking_ = false;
  playback.seekTo(t);
  if (scrub_resume_play_) {
    scrub_resume_play_ = false;
    if (playback.isPaused()) {
      playback.resume();
    }
  }
  syncTimelineFromPlayback();
  syncTransportButtons();
  syncDensityBar();
}

void RecordPanel::onSeekMoved(int value) {
  if (!seeking_ || manager_ == nullptr) {
    return;
  }
  integration::PlaybackController& playback = manager_->playback();
  const double total = playback.totalTimeSec();
  const double normalized = static_cast<double>(value) / kTimelineTicks;
  const double t =
      total > 0.0 ? std::clamp(normalized, 0.0, 1.0) * total : 0.0;
  const int pct =
      total > 0.0 ? static_cast<int>(std::lround((t / total) * 100.0)) : 0;
  time_label_->setText(tr("%1 / %2 (%3%)")
                           .arg(FormatClock(t), FormatClock(total))
                           .arg(pct));
  if (density_bar_ != nullptr && total > 0.0) {
    density_bar_->setPlayheadNormalized(normalized);
  }
  if (total <= 0.0) {
    return;
  }
  const qint64 now = QDateTime::currentMSecsSinceEpoch();
  if (last_scrub_ms_ != 0 && now - last_scrub_ms_ < kScrubThrottleMs) {
    return;
  }
  last_scrub_ms_ = now;
  playback.scrubTo(t);
}

void RecordPanel::onChannelItemChanged(QTreeWidgetItem* item, int column) {
  if (rebuilding_channels_ || item == nullptr || column != kColumnChannel) {
    return;
  }
  applyExcludedChannelsFromTree();
}

void RecordPanel::onChannelDoubleClicked(QTreeWidgetItem* item, int /*column*/) {
  if (item == nullptr || item->isDisabled()) {
    return;
  }
  emit openInRawMessagesRequested(item->text(kColumnChannel));
}

void RecordPanel::onFilterTextChanged(const QString& /*text*/) {
  applyChannelFilter();
}

void RecordPanel::onSelectAllChannels() {
  if (channel_tree_ == nullptr) {
    return;
  }
  rebuilding_channels_ = true;
  for (int i = 0; i < channel_tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = channel_tree_->topLevelItem(i);
    if (item == nullptr || item->isDisabled()) {
      continue;
    }
    item->setCheckState(kColumnChannel, Qt::Checked);
  }
  rebuilding_channels_ = false;
  applyExcludedChannelsFromTree();
}

void RecordPanel::onSelectNoneChannels() {
  if (channel_tree_ == nullptr) {
    return;
  }
  rebuilding_channels_ = true;
  for (int i = 0; i < channel_tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = channel_tree_->topLevelItem(i);
    if (item == nullptr || item->isDisabled()) {
      continue;
    }
    item->setCheckState(kColumnChannel, Qt::Unchecked);
  }
  rebuilding_channels_ = false;
  applyExcludedChannelsFromTree();
}

void RecordPanel::onTick() {
  if (manager_ == nullptr || seeking_) {
    return;
  }
  const std::string& path = manager_->playback().currentFile();
  if (path != loaded_file_) {
    reloadFromPlayback();
    return;
  }
  syncTimelineFromPlayback();
  syncTransportButtons();
}

void RecordPanel::applyChromeStyles() {
  setStyleSheet(QStringLiteral(
      "QToolButton { padding: 3px 6px; }"
      "QTreeWidget { border: 1px solid #d0d7de; border-radius: 4px; }"));
}

void RecordPanel::syncTransportButtons() {
  if (manager_ == nullptr) {
    return;
  }
  const integration::PlaybackController& playback = manager_->playback();
  const bool has_file = !playback.currentFile().empty() || !loaded_file_.empty();
  const bool has_duration = has_file && playback.totalTimeSec() > 0.0;

  play_pause_button_->setEnabled(true);
  stop_button_->setEnabled(has_file);
  seek_start_button_->setEnabled(has_duration);
  seek_end_button_->setEnabled(has_duration);
  step_back_button_->setEnabled(has_duration);
  step_forward_button_->setEnabled(has_duration);
  prev_msg_button_->setEnabled(has_duration);
  next_msg_button_->setEnabled(has_duration);
  range_in_button_->setEnabled(has_duration);
  range_out_button_->setEnabled(has_duration);
  range_clear_button_->setEnabled(has_duration);
  select_all_button_->setEnabled(has_file);
  select_none_button_->setEnabled(has_file);
  timeline_->setEnabled(has_duration);
  filter_edit_->setEnabled(has_file);

  if (playback.isPlaying() && !playback.isPaused()) {
    play_pause_button_->setText(tr("Pause"));
    play_pause_button_->setToolTip(tr("Pause playback (Space)"));
  } else {
    play_pause_button_->setText(tr("Play"));
    play_pause_button_->setToolTip(
        has_file ? tr("Start or resume playback (Space)")
                 : tr("Open a record, then play"));
  }
}

void RecordPanel::syncTimelineFromPlayback() {
  if (manager_ == nullptr || seeking_) {
    return;
  }
  const integration::PlaybackController& playback = manager_->playback();
  const double total = playback.totalTimeSec();
  const double current = playback.currentTimeSec();
  timeline_->blockSignals(true);
  if (total > 0.0) {
    const int value = static_cast<int>(
        std::lround(std::clamp(current / total, 0.0, 1.0) * kTimelineTicks));
    timeline_->setValue(value);
  } else {
    timeline_->setValue(0);
  }
  timeline_->blockSignals(false);
  const int pct = total > 0.0
                      ? static_cast<int>(std::lround(
                            std::clamp(current / total, 0.0, 1.0) * 100.0))
                      : 0;
  time_label_->setText(tr("%1 / %2 (%3%)")
                           .arg(FormatClock(current), FormatClock(total))
                           .arg(pct));
  if (density_bar_ != nullptr && total > 0.0) {
    density_bar_->setPlayheadNormalized(
        std::clamp(current / total, 0.0, 1.0));
  }
}

void RecordPanel::syncRangeLabel() {
  if (manager_ == nullptr || range_label_ == nullptr) {
    return;
  }
  const integration::PlaybackController& playback = manager_->playback();
  if (playback.currentFile().empty() || playback.totalTimeSec() <= 0.0) {
    range_label_->setText(tr("Range: full"));
    return;
  }
  if (!playback.hasPlaybackRange()) {
    range_label_->setText(tr("Range: full"));
    return;
  }
  range_label_->setText(
      tr("Range: %1 – %2")
          .arg(FormatClock(playback.rangeStartSec()),
               FormatClock(playback.rangeEndSec())));
}

void RecordPanel::syncDensityBar() {
  if (manager_ == nullptr || density_bar_ == nullptr) {
    return;
  }
  const integration::PlaybackController& playback = manager_->playback();
  density_bar_->setBins(playback.densityBins());
  const double total = playback.totalTimeSec();
  if (total > 0.0) {
    density_bar_->setRangeNormalized(playback.rangeStartSec() / total,
                                     playback.rangeEndSec() / total);
    density_bar_->setPlayheadNormalized(
        std::clamp(playback.currentTimeSec() / total, 0.0, 1.0));
  } else {
    density_bar_->setRangeNormalized(0.0, 1.0);
    density_bar_->setPlayheadNormalized(0.0);
  }
}

void RecordPanel::applyExcludedChannelsFromTree() {
  if (manager_ == nullptr || channel_tree_ == nullptr) {
    return;
  }
  std::set<std::string> excluded;
  for (int i = 0; i < channel_tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = channel_tree_->topLevelItem(i);
    if (item == nullptr) {
      continue;
    }
    if (item->checkState(kColumnChannel) != Qt::Checked) {
      excluded.insert(item->text(kColumnChannel).toStdString());
    }
  }
  manager_->playback().setExcludedChannels(std::move(excluded));
}

void RecordPanel::rebuildChannelTree() {
  if (manager_ == nullptr || channel_tree_ == nullptr) {
    return;
  }
  rebuilding_channels_ = true;
  channel_tree_->clear();
  const integration::PlaybackController& playback = manager_->playback();
  const auto& excluded = playback.excludedChannels();
  for (const std::string& channel : playback.channelNames()) {
    auto* item = new QTreeWidgetItem(channel_tree_);
    item->setText(kColumnChannel, QString::fromStdString(channel));
    item->setText(kColumnType,
                  ShortMessageType(playback.channelMessageType(channel)));
    item->setText(kColumnCount,
                  QString::number(playback.channelMessageCount(channel)));
    item->setTextAlignment(kColumnCount, Qt::AlignRight | Qt::AlignVCenter);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    const bool muted = excluded.find(channel) != excluded.end();
    item->setCheckState(kColumnChannel, muted ? Qt::Unchecked : Qt::Checked);
    if (channel.rfind("/autolink/", 0) == 0) {
      item->setCheckState(kColumnChannel, Qt::Unchecked);
      item->setDisabled(true);
    }
  }
  rebuilding_channels_ = false;
  applyChannelFilter();
}

void RecordPanel::applyChannelFilter() {
  if (channel_tree_ == nullptr || filter_edit_ == nullptr) {
    return;
  }
  const QString needle = filter_edit_->text().trimmed();
  for (int i = 0; i < channel_tree_->topLevelItemCount(); ++i) {
    QTreeWidgetItem* item = channel_tree_->topLevelItem(i);
    if (item == nullptr) {
      continue;
    }
    const bool match =
        needle.isEmpty() ||
        item->text(kColumnChannel).contains(needle, Qt::CaseInsensitive) ||
        item->text(kColumnType).contains(needle, Qt::CaseInsensitive);
    item->setHidden(!match);
  }
}

void RecordPanel::seekByDelta(double delta_sec) {
  if (manager_ == nullptr) {
    return;
  }
  integration::PlaybackController& playback = manager_->playback();
  const double next = playback.currentTimeSec() + delta_sec;
  playback.seekTo(next);
  syncTimelineFromPlayback();
  syncTransportButtons();
}

void RecordPanel::seekToNormalized(double normalized) {
  if (manager_ == nullptr) {
    return;
  }
  integration::PlaybackController& playback = manager_->playback();
  const double total = playback.totalTimeSec();
  if (total <= 0.0) {
    return;
  }
  playback.seekTo(std::clamp(normalized, 0.0, 1.0) * total);
  syncTimelineFromPlayback();
  syncTransportButtons();
}

bool RecordPanel::ensureFileOpen() {
  if (manager_ == nullptr) {
    return false;
  }
  if (manager_->playback().currentFile().empty()) {
    emit openRecordRequested();
    return false;
  }
  return true;
}

std::string RecordPanel::selectedChannel() const {
  if (channel_tree_ == nullptr) {
    return {};
  }
  QTreeWidgetItem* item = channel_tree_->currentItem();
  if (item == nullptr || item->isDisabled()) {
    return {};
  }
  return item->text(kColumnChannel).toStdString();
}

QString RecordPanel::FormatClock(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0.0) {
    return QStringLiteral("00:00.0");
  }
  const int total_tenths = static_cast<int>(std::lround(seconds * 10.0));
  const int tenths = total_tenths % 10;
  const int total_sec = total_tenths / 10;
  const int mins = total_sec / 60;
  const int secs = total_sec % 60;
  return QStringLiteral("%1:%2.%3")
      .arg(mins, 2, 10, QLatin1Char('0'))
      .arg(secs, 2, 10, QLatin1Char('0'))
      .arg(tenths);
}

QString RecordPanel::ShortMessageType(const std::string& message_type) {
  QString type = QString::fromStdString(message_type);
  const int dot = type.lastIndexOf(QLatin1Char('.'));
  if (dot >= 0 && dot + 1 < type.size()) {
    return type.mid(dot + 1);
  }
  return type;
}

}  // namespace autoviz
