/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/evidence/evidence_panel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <QHash>

#include <QBrush>
#include <QColor>
#include <QComboBox>
#include <QDir>
#include <QPen>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QGraphicsItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include <QGraphicsSimpleTextItem>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QMouseEvent>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>
#include <QSplitter>
#include <QTabWidget>
#include <QToolButton>
#include <QTreeWidget>
#include <QVBoxLayout>

#include "autoviz/common/interaction/playback_controller.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/panel/title_tools.hpp"

namespace autoviz {
namespace {

qint64 ClockNs(const QJsonObject& obj) {
  const QJsonObject clock = obj.value(QStringLiteral("clock")).toObject();
  if (!clock.isEmpty()) {
    return static_cast<qint64>(
        clock.value(QStringLiteral("monotonic_ns")).toDouble());
  }
  return static_cast<qint64>(
      obj.value(QStringLiteral("monotonic_ns")).toDouble());
}

QColor KindColor(const QString& kind) {
  if (kind.contains(QLatin1String("FAILURE"))) {
    return QColor(220, 70, 70);
  }
  if (kind.contains(QLatin1String("DECISION"))) {
    return QColor(70, 130, 210);
  }
  if (kind.contains(QLatin1String("ACTION"))) {
    return QColor(60, 160, 110);
  }
  if (kind.contains(QLatin1String("OUTCOME"))) {
    return QColor(200, 140, 40);
  }
  return QColor(120, 120, 140);
}

QString OverlaySummary(const QJsonObject& o, const QString& title) {
  if (o.isEmpty()) {
    return title + QStringLiteral(" (empty)\n");
  }
  return title + QStringLiteral("\n  valid=") +
         (o.value(QStringLiteral("valid")).toBool() ? QStringLiteral("true")
                                                    : QStringLiteral("false")) +
         QStringLiteral("\n  match=") +
         QString::number(o.value(QStringLiteral("action_match")).toDouble()) +
         QStringLiteral("\n  bit_exact=") +
         (o.value(QStringLiteral("bit_exact")).toBool() ? QStringLiteral("true")
                                                        : QStringLiteral("false")) +
         QStringLiteral("\n  mode=") + o.value(QStringLiteral("mode")).toString() +
         QStringLiteral("\n  reason=") + o.value(QStringLiteral("reason")).toString() +
         QStringLiteral("\n");
}

}  // namespace

class EvidencePanel::GanttWidget : public QWidget {
 public:
  explicit GanttWidget(EvidencePanel* owner) : QWidget(owner), owner_(owner) {
    setMinimumHeight(140);
    setMouseTracking(true);
  }

  void setCursorNs(qint64 ns) {
    cursor_ns_ = ns;
    update();
  }

 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter p(this);
    p.fillRect(rect(), palette().base());
    if (owner_ == nullptr || owner_->nodes_.isEmpty()) {
      p.setPen(palette().placeholderText().color());
      p.drawText(rect(), Qt::AlignCenter, tr("Open a sidecar.json"));
      return;
    }
    const qint64 span = std::max<qint64>(1, owner_->end_ns_ - owner_->start_ns_);
    const int row_h = 22;
    const int label_w = 140;
    const int n = owner_->nodes_.size();
    setMinimumHeight(std::max(140, 8 + n * row_h));
    for (int i = 0; i < n; ++i) {
      const QJsonObject node = owner_->nodes_.at(i).toObject();
      const QRect row(0, 4 + i * row_h, width(), row_h - 2);
      p.setPen(palette().text().color());
      p.drawText(QRect(8, row.y(), label_w - 12, row.height()),
                 Qt::AlignVCenter | Qt::AlignLeft,
                 node.value(QStringLiteral("id")).toString());
      const qint64 t = ClockNs(node);
      const double u =
          static_cast<double>(t - owner_->start_ns_) / static_cast<double>(span);
      const int x = label_w + static_cast<int>(u * (width() - label_w - 8));
      p.fillRect(QRect(x, row.y() + 4, 48, row.height() - 8),
                 KindColor(node.value(QStringLiteral("kind")).toString()));
    }
    const double cu = static_cast<double>(cursor_ns_ - owner_->start_ns_) /
                      static_cast<double>(span);
    const int cx =
        label_w + static_cast<int>(std::clamp(cu, 0.0, 1.0) * (width() - label_w - 8));
    p.setPen(QPen(QColor(220, 80, 80), 1));
    p.drawLine(cx, 0, cx, height());
  }

  void mousePressEvent(QMouseEvent* event) override {
    if (owner_ == nullptr || width() <= 148) {
      return;
    }
    const int label_w = 140;
    const int x = event->position().toPoint().x();
    if (x <= label_w) {
      const int row = (event->position().toPoint().y() - 4) / 22;
      if (row >= 0 && row < owner_->nodes_.size()) {
        owner_->onNodeActivated(
            owner_->nodes_.at(row).toObject().value(QStringLiteral("id")).toString());
      }
      return;
    }
    const double u = static_cast<double>(x - label_w) /
                     static_cast<double>(std::max(1, width() - label_w - 8));
    const qint64 ns =
        owner_->start_ns_ +
        static_cast<qint64>(u * static_cast<double>(owner_->end_ns_ - owner_->start_ns_));
    owner_->applyCursor(ns, true);
  }

 private:
  EvidencePanel* owner_ = nullptr;
  qint64 cursor_ns_ = 0;
};

class EvidencePanel::CausalView : public QGraphicsView {
 public:
  explicit CausalView(EvidencePanel* owner)
      : QGraphicsView(owner), owner_(owner), scene_(new QGraphicsScene(this)) {
    setScene(scene_);
    setRenderHint(QPainter::Antialiasing, true);
    setDragMode(QGraphicsView::ScrollHandDrag);
  }

  void rebuild(const QJsonArray& nodes, const QJsonArray& edges,
               const QString& selected) {
    scene_->clear();
    items_.clear();
    QHash<QString, QPointF> pos;
    int col[6] = {0, 0, 0, 0, 0, 0};
    for (const auto& v : nodes) {
      const QJsonObject n = v.toObject();
      const QString kind = n.value(QStringLiteral("kind")).toString();
      int lane = 1;
      if (kind.contains(QLatin1String("DECISION"))) {
        lane = 2;
      } else if (kind.contains(QLatin1String("ACTION"))) {
        lane = 3;
      } else if (kind.contains(QLatin1String("OUTCOME"))) {
        lane = 4;
      } else if (kind.contains(QLatin1String("FAILURE"))) {
        lane = 5;
      }
      const QPointF p(80.0 + lane * 160.0, 40.0 + col[lane] * 90.0);
      ++col[lane];
      const QString id = n.value(QStringLiteral("id")).toString();
      pos.insert(id, p);
      auto* ell = scene_->addEllipse(-28, -18, 56, 36, QPen(Qt::NoPen),
                                     QBrush(KindColor(kind)));
      ell->setPos(p);
      ell->setData(0, id);
      ell->setFlag(QGraphicsItem::ItemIsSelectable, true);
      if (id == selected) {
        ell->setPen(QPen(Qt::white, 2));
      }
      auto* label = scene_->addSimpleText(id);
      label->setBrush(Qt::white);
      label->setPos(p.x() - 24, p.y() - 8);
      label->setData(0, id);
      items_.insert(id, ell);
    }
    for (const auto& v : edges) {
      const QJsonObject e = v.toObject();
      const QPointF a = pos.value(e.value(QStringLiteral("from_id")).toString());
      const QPointF b = pos.value(e.value(QStringLiteral("to_id")).toString());
      if (a.isNull() || b.isNull()) {
        continue;
      }
      scene_->addLine(QLineF(a, b), QPen(QColor(180, 180, 190), 1.5));
    }
    scene_->setSceneRect(scene_->itemsBoundingRect().adjusted(-40, -40, 40, 40));
  }

 protected:
  void mousePressEvent(QMouseEvent* event) override {
    QGraphicsView::mousePressEvent(event);
    if (owner_ == nullptr) {
      return;
    }
    QGraphicsItem* hit = itemAt(event->position().toPoint());
    if (hit == nullptr) {
      return;
    }
    const QString id = hit->data(0).toString();
    if (!id.isEmpty()) {
      owner_->onNodeActivated(id);
    }
  }

 private:
  EvidencePanel* owner_ = nullptr;
  QGraphicsScene* scene_ = nullptr;
  QHash<QString, QGraphicsItem*> items_;
};

EvidencePanel::~EvidencePanel() = default;

EvidencePanel::EvidencePanel(common::VisualizationManager* manager, QWidget* parent)
    : QWidget(parent), manager_(manager) {
  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(8, 8, 8, 8);
  root->setSpacing(6);

  auto* tools = new QHBoxLayout;
  auto* open = new QPushButton(tr("Open sidecar"), this);
  auto* overlay_b = new QPushButton(tr("Overlay B"), this);
  header_ = new QLabel(tr("No episode"), this);
  header_->setWordWrap(true);
  tools->addWidget(open);
  tools->addWidget(overlay_b);
  tools->addWidget(header_, 1);
  root->addLayout(tools);

  auto* cursor_row = new QHBoxLayout;
  cursor_ = new QSlider(Qt::Horizontal, this);
  cursor_->setRange(0, 1000);
  cursor_label_ = new QLabel(QStringLiteral("0 ns"), this);
  cursor_row->addWidget(new QLabel(tr("Cursor"), this));
  cursor_row->addWidget(cursor_, 1);
  cursor_row->addWidget(cursor_label_);
  root->addLayout(cursor_row);

  tabs_ = new QTabWidget(this);
  gantt_ = new GanttWidget(this);
  causal_ = new CausalView(this);
  bt_tree_ = new QTreeWidget(this);
  bt_tree_->setHeaderLabels({tr("BT / node"), tr("kind"), tr("t")});
  data_list_ = new QListWidget(this);
  compare_ = new QPlainTextEdit(this);
  compare_->setReadOnly(true);

  auto* data_page = new QWidget(this);
  auto* data_split = new QSplitter(Qt::Vertical, data_page);
  auto* data_layout = new QVBoxLayout(data_page);
  data_layout->setContentsMargins(0, 0, 0, 0);
  data_split->addWidget(data_list_);
  auto* form = new QWidget(data_page);
  auto* form_l = new QVBoxLayout(form);
  obs_id_ = new QLineEdit(form);
  obs_id_->setPlaceholderText(tr("observation_id"));
  obs_desc_ = new QPlainTextEdit(form);
  obs_desc_->setPlaceholderText(tr("description (required)"));
  obs_desc_->setMaximumHeight(72);
  obs_node_ = new QComboBox(form);
  obs_bt_ = new QLineEdit(form);
  obs_bt_->setPlaceholderText(tr("bt_node"));
  auto* save = new QPushButton(tr("Write observation jsonl"), form);
  form_l->addWidget(obs_id_);
  form_l->addWidget(obs_desc_);
  form_l->addWidget(obs_node_);
  form_l->addWidget(obs_bt_);
  form_l->addWidget(save);
  data_split->addWidget(form);
  data_layout->addWidget(data_split);

  tabs_->addTab(gantt_, tr("Timeline"));
  tabs_->addTab(causal_, tr("Causal"));
  tabs_->addTab(bt_tree_, tr("BT"));
  tabs_->addTab(data_page, tr("Observe"));
  tabs_->addTab(compare_, tr("Compare"));
  root->addWidget(tabs_, 1);

  connect(open, &QPushButton::clicked, this, &EvidencePanel::onOpenSidecar);
  connect(overlay_b, &QPushButton::clicked, this, &EvidencePanel::onOpenOverlayB);
  connect(cursor_, &QSlider::valueChanged, this, &EvidencePanel::onCursorChanged);
  connect(save, &QPushButton::clicked, this, &EvidencePanel::onSaveObservation);
  connect(bt_tree_, &QTreeWidget::itemClicked, this,
          [this](QTreeWidgetItem* item, int) {
            if (item != nullptr) {
              onNodeActivated(item->data(0, Qt::UserRole).toString());
            }
          });
  connect(data_list_, &QListWidget::itemClicked, this, [this](QListWidgetItem* item) {
    if (item == nullptr) {
      return;
    }
    const QString id = item->data(Qt::UserRole).toString();
    if (!id.isEmpty()) {
      onNodeActivated(id);
    }
  });
}

void EvidencePanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("EvidenceDock");
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
  const PanelTitleBarTools tools = CreatePanelTitleBarTools(dock, callbacks, options);
  expand_button_ = tools.expand_button;
  dock->setTitleBarTools(tools.widget);
}

void EvidencePanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ == nullptr) {
    return;
  }
  expand_button_->blockSignals(true);
  expand_button_->setChecked(checked);
  expand_button_->blockSignals(false);
}

bool EvidencePanel::openSidecarFile(const QString& path) {
  if (path.isEmpty() || !loadSidecar(path)) {
    return false;
  }
  return true;
}

void EvidencePanel::onOpenSidecar() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Open sidecar.json"), QString(),
      tr("Sidecar JSON (*.sidecar.json *.json)"));
  if (path.isEmpty()) {
    return;
  }
  if (!openSidecarFile(path)) {
    QMessageBox::warning(this, tr("Evidence"), tr("Failed to parse sidecar."));
  }
}

void EvidencePanel::onOpenOverlayB() {
  const QString path = QFileDialog::getOpenFileName(
      this, tr("Open overlay B"), QString(), tr("Overlay JSON (*.json)"));
  if (path.isEmpty()) {
    return;
  }
  if (!loadOverlay(path, &overlay_b_)) {
    QMessageBox::warning(this, tr("Evidence"), tr("Failed to parse overlay B."));
    return;
  }
  compare_->setPlainText(OverlaySummary(overlay_a_, QStringLiteral("A")) +
                         QStringLiteral("\n") +
                         OverlaySummary(overlay_b_, QStringLiteral("B")));
}

void EvidencePanel::onCursorChanged(int value) {
  applyCursor(sliderToNs(value), true);
}

void EvidencePanel::onSaveObservation() {
  if (observations_path_.isEmpty()) {
    QMessageBox::warning(this, tr("Evidence"), tr("Open a sidecar first."));
    return;
  }
  const QString desc = obs_desc_->toPlainText().trimmed();
  const QString node = obs_node_->currentText().trimmed();
  if (desc.isEmpty() || node.isEmpty()) {
    QMessageBox::warning(this, tr("Evidence"),
                         tr("Observation needs description and evidence_node_id."));
    return;
  }
  QJsonObject o;
  o.insert(QStringLiteral("observation_id"), obs_id_->text().trimmed());
  o.insert(QStringLiteral("description"), desc);
  o.insert(QStringLiteral("bt_node"), obs_bt_->text().trimmed());
  o.insert(QStringLiteral("start_monotonic_ns"), static_cast<double>(cursor_ns_));
  o.insert(QStringLiteral("end_monotonic_ns"),
           static_cast<double>(cursor_ns_ + 50'000'000));
  o.insert(QStringLiteral("evidence_node_ids"), QJsonArray{node});
  QFile file(observations_path_);
  if (!file.open(QIODevice::Append | QIODevice::Text)) {
    QMessageBox::warning(this, tr("Evidence"), tr("Cannot write observations jsonl."));
    return;
  }
  file.write(QJsonDocument(o).toJson(QJsonDocument::Compact));
  file.write("\n");
  appendObservationRow(o);
  observations_.append(o);
}

void EvidencePanel::onNodeActivated(const QString& node_id) {
  if (node_id.isEmpty()) {
    return;
  }
  for (const auto& v : nodes_) {
    const QJsonObject n = v.toObject();
    if (n.value(QStringLiteral("id")).toString() == node_id) {
      applyCursor(ClockNs(n), true);
      obs_node_->setCurrentText(node_id);
      obs_bt_->setText(n.value(QStringLiteral("bt_node")).toString());
      break;
    }
  }
  causal_->rebuild(nodes_, edges_, node_id);
}

bool EvidencePanel::loadSidecar(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    return false;
  }
  const auto doc = QJsonDocument::fromJson(file.readAll());
  if (!doc.isObject()) {
    return false;
  }
  sidecar_path_ = path;
  sidecar_ = doc.object();
  nodes_ = sidecar_.value(QStringLiteral("graph_nodes")).toArray();
  edges_ = sidecar_.value(QStringLiteral("graph_edges")).toArray();
  triggers_ = sidecar_.value(QStringLiteral("triggers")).toArray();
  observations_ = sidecar_.value(QStringLiteral("observations")).toArray();
  start_ns_ = ClockNs(sidecar_.value(QStringLiteral("start")).toObject());
  end_ns_ = ClockNs(sidecar_.value(QStringLiteral("end")).toObject());
  if (start_ns_ == 0 && end_ns_ == 0) {
    start_ns_ = std::numeric_limits<qint64>::max();
    end_ns_ = 0;
    auto consider = [&](qint64 t) {
      if (t <= 0) {
        return;
      }
      start_ns_ = std::min(start_ns_, t);
      end_ns_ = std::max(end_ns_, t);
    };
    for (const auto& v : nodes_) {
      consider(ClockNs(v.toObject()));
    }
    for (const auto& v : triggers_) {
      consider(ClockNs(v.toObject()));
    }
    if (start_ns_ == std::numeric_limits<qint64>::max()) {
      start_ns_ = 0;
      end_ns_ = 1;
    }
  }
  if (end_ns_ <= start_ns_) {
    end_ns_ = start_ns_ + 1;
  }
  observations_path_ = siblingPath(path, QStringLiteral(".observations.jsonl"));
  loadObservations(observations_path_);
  const QString overlay = siblingPath(path, QStringLiteral(".overlay.json"));
  loadOverlay(overlay, &overlay_a_);
  rebuildViews();
  applyCursor(start_ns_, false);
  const QJsonObject gate = sidecar_.value(QStringLiteral("gate")).toObject();
  const QJsonObject fail = sidecar_.value(QStringLiteral("bt_failure")).toObject();
  header_->setText(
      sidecar_.value(QStringLiteral("episode_id")).toString() +
      QStringLiteral("  admitted=") +
      (sidecar_.value(QStringLiteral("admitted")).toBool() ? QStringLiteral("true")
                                                           : QStringLiteral("false")) +
      QStringLiteral("  bt=") +
      fail.value(QStringLiteral("node_name")).toString() +
      QStringLiteral("  gate.passed=") +
      (gate.value(QStringLiteral("passed")).toBool() ? QStringLiteral("true")
                                                     : QStringLiteral("false")) +
      QStringLiteral("  ") +
      gate.value(QStringLiteral("reason")).toString());
  QString cards;
  for (const auto& v : sidecar_.value(QStringLiteral("root_causes")).toArray()) {
    const QJsonObject c = v.toObject();
    cards += c.value(QStringLiteral("hypothesis_id")).toString() +
             QStringLiteral("  ") +
             c.value(QStringLiteral("statement")).toString() +
             QStringLiteral("\n");
  }
  QString gate_text = QStringLiteral("Gate passed=") +
                      (gate.value(QStringLiteral("passed")).toBool()
                           ? QStringLiteral("true")
                           : QStringLiteral("false")) +
                      QStringLiteral(" reason=") +
                      gate.value(QStringLiteral("reason")).toString() +
                      QStringLiteral("\n  replay_ok=") +
                      QString::number(gate.value(QStringLiteral("replay_ok")).toBool()) +
                      QStringLiteral(" safety_ok=") +
                      QString::number(gate.value(QStringLiteral("safety_ok")).toBool()) +
                      QStringLiteral(" coverage_ok=") +
                      QString::number(
                          gate.value(QStringLiteral("coverage_ok")).toBool()) +
                      QStringLiteral("\nbt_failure=") +
                      fail.value(QStringLiteral("node_name")).toString() +
                      QStringLiteral("  priority=") +
                      sidecar_.value(QStringLiteral("priority")).toString() +
                      QStringLiteral("\n");
  compare_->setPlainText(gate_text + OverlaySummary(overlay_a_, QStringLiteral("A")) +
                         QStringLiteral("\n") +
                         OverlaySummary(overlay_b_, QStringLiteral("B")) +
                         QStringLiteral("\nRoot causes\n") + cards);
  return true;
}

bool EvidencePanel::loadOverlay(const QString& path, QJsonObject* dest) {
  if (dest == nullptr) {
    return false;
  }
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    return false;
  }
  const auto doc = QJsonDocument::fromJson(file.readAll());
  if (!doc.isObject()) {
    return false;
  }
  *dest = doc.object();
  return true;
}

void EvidencePanel::loadObservations(const QString& jsonl_path) {
  QFile file(jsonl_path);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    return;
  }
  while (!file.atEnd()) {
    const auto line = file.readLine().trimmed();
    if (line.isEmpty()) {
      continue;
    }
    const auto doc = QJsonDocument::fromJson(line);
    if (doc.isObject()) {
      observations_.append(doc.object());
    }
  }
}

void EvidencePanel::rebuildViews() {
  obs_node_->clear();
  bt_tree_->clear();
  data_list_->clear();
  QHash<QString, QTreeWidgetItem*> roots;
  for (const auto& v : nodes_) {
    const QJsonObject n = v.toObject();
    const QString id = n.value(QStringLiteral("id")).toString();
    obs_node_->addItem(id);
    const QString bt = n.value(QStringLiteral("bt_node")).toString();
    const QString key = bt.isEmpty() ? QStringLiteral("(unbound)") : bt;
    QTreeWidgetItem* parent = roots.value(key);
    if (parent == nullptr) {
      parent = new QTreeWidgetItem(bt_tree_, {key});
      roots.insert(key, parent);
    }
    auto* child = new QTreeWidgetItem(
        parent, {id, n.value(QStringLiteral("kind")).toString(),
                 QString::number(ClockNs(n))});
    child->setData(0, Qt::UserRole, id);
  }
  bt_tree_->expandAll();
  for (const auto& v : observations_) {
    appendObservationRow(v.toObject());
  }
  gantt_->update();
  causal_->rebuild(nodes_, edges_, QString());
  gantt_->setCursorNs(cursor_ns_);
}

void EvidencePanel::applyCursor(qint64 monotonic_ns, bool seek_playback) {
  cursor_ns_ = std::clamp(monotonic_ns, start_ns_, end_ns_);
  const QSignalBlocker block(cursor_);
  cursor_->setValue(nsToSlider(cursor_ns_));
  cursor_label_->setText(QString::number(cursor_ns_) + QStringLiteral(" ns"));
  gantt_->setCursorNs(cursor_ns_);
  if (seek_playback) {
    seekPlayback(cursor_ns_);
  }
}

qint64 EvidencePanel::sliderToNs(int value) const {
  const double u = static_cast<double>(std::clamp(value, 0, 1000)) / 1000.0;
  return start_ns_ + static_cast<qint64>(u * static_cast<double>(end_ns_ - start_ns_));
}

int EvidencePanel::nsToSlider(qint64 ns) const {
  const qint64 span = std::max<qint64>(1, end_ns_ - start_ns_);
  const double u =
      static_cast<double>(ns - start_ns_) / static_cast<double>(span);
  return static_cast<int>(std::clamp(u, 0.0, 1.0) * 1000.0);
}

void EvidencePanel::seekPlayback(qint64 monotonic_ns) {
  if (manager_ == nullptr) {
    return;
  }
  auto& playback = manager_->playback();
  const double lo = playback.rangeStartSec();
  const double hi = playback.rangeEndSec();
  if (!(hi > lo)) {
    return;
  }
  const qint64 span = std::max<qint64>(1, end_ns_ - start_ns_);
  const double u =
      static_cast<double>(monotonic_ns - start_ns_) / static_cast<double>(span);
  playback.seekTo(lo + std::clamp(u, 0.0, 1.0) * (hi - lo));
}

void EvidencePanel::appendObservationRow(const QJsonObject& obs) {
  const auto ids = obs.value(QStringLiteral("evidence_node_ids")).toArray();
  const QString first = ids.isEmpty() ? QString() : ids.at(0).toString();
  auto* item = new QListWidgetItem(
      obs.value(QStringLiteral("observation_id")).toString() + QStringLiteral("  ") +
      obs.value(QStringLiteral("description")).toString());
  item->setData(Qt::UserRole, first);
  data_list_->addItem(item);
}

QString EvidencePanel::siblingPath(const QString& sidecar,
                                   const QString& suffix) const {
  QString stem = sidecar;
  constexpr char kSidecar[] = ".sidecar.json";
  if (stem.endsWith(QLatin1String(kSidecar))) {
    stem.chop(static_cast<int>(sizeof(kSidecar) - 1));
    return stem + suffix;
  }
  const QFileInfo info(sidecar);
  return info.dir().filePath(info.completeBaseName() + suffix);
}

}  // namespace autoviz
