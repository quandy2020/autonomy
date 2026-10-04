/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/channel_graph/channel_graph_panel.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <utility>

#include <QAbstractItemView>
#include <QApplication>
#include <QClipboard>
#include <QColor>
#include <QDialog>
#include <QFocusEvent>
#include <QFrame>
#include <QHash>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QPainter>
#include <QPixmap>
#include <QPolygonF>
#include <QPushButton>
#include <QScrollArea>
#include <QShowEvent>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <QVector>

#include "autolink/proto/role_attributes.pb.h"
#include "autolink/service_discovery/topology_manager.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/integration/channel_stats_registry.hpp"
#include "autoviz/integration/topology_graph_builder.hpp"
#include "autoviz/ui/channel_graph/channel_graph_view.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/theme/panel.hpp"
#include "autoviz/ui/panel/context_menu.hpp"
#include "autoviz/ui/panel/dock.hpp"
#include "autoviz/ui/panel/title_tools.hpp"
#include "autoviz/ui/plot/message_field_tree.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace channel_graph {
namespace {

constexpr int kAutoRefreshMs = 2000;
constexpr int kFilterDebounceMs = 250;

constexpr char kNodeColor[] = "#0891b2";
constexpr char kChannelColor[] = "#7c3aed";
constexpr char kServiceColor[] = "#ef4444";

QIcon MakeGlyphIcon(std::function<void(QPainter&, const QRectF&)> paint,
                    int size = 18) {
  QPixmap pixmap(size, size);
  pixmap.fill(Qt::transparent);
  QPainter painter(&pixmap);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setRenderHint(QPainter::TextAntialiasing, true);
  paint(painter, QRectF(0, 0, size, size));
  return QIcon(pixmap);
}

QIcon MakeChannelGlyphIcon() {
  return MakeGlyphIcon([](QPainter& p, const QRectF& r) {
    const QRectF pill(r.left() + 1.5, r.center().y() - 4.5, r.width() - 3, 9);
    p.setPen(QPen(QColor(0xd8, 0xb4, 0xfe), 1.2));
    p.setBrush(QColor(0xfa, 0xf5, 0xff));
    p.drawRoundedRect(pill, 4.5, 4.5);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(QLatin1String(kChannelColor)));
    p.drawEllipse(QPointF(pill.left() + 4.0, pill.center().y()), 2.4, 2.4);
    p.drawEllipse(QPointF(pill.right() - 4.0, pill.center().y()), 2.4, 2.4);
  });
}

QIcon MakeServiceGlyphIcon() {
  return MakeGlyphIcon([](QPainter& p, const QRectF& r) {
    const QRectF card(r.left() + 1.5, r.top() + 3.5, r.width() - 3, r.height() - 7);
    p.setPen(QPen(QColor(0xfe, 0xa3, 0xb4), 1.2));
    p.setBrush(QColor(0xff, 0xf1, 0xf2));
    p.drawRoundedRect(card, 3.5, 3.5);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(QLatin1String(kServiceColor)));
    p.drawEllipse(QPointF(card.left() + 5.5, card.center().y()), 3.2, 3.2);
  });
}

QIcon MakeRefreshGlyphIcon() {
  return MakeGlyphIcon([](QPainter& p, const QRectF& r) {
    const QColor ink(0x1e, 0x29, 0x3b);
    const QPointF center = r.center();
    constexpr qreal kRadius = 5.2;
    const QRectF arc(center.x() - kRadius, center.y() - kRadius, kRadius * 2.0,
                     kRadius * 2.0);
    p.setPen(QPen(ink, 1.6, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
    p.setBrush(Qt::NoBrush);
    p.drawArc(arc, 40 * 16, -300 * 16);

    const qreal end = (40.0 - 300.0) * 3.141592653589793 / 180.0;
    const QPointF tip(center.x() + kRadius * std::cos(end),
                      center.y() - kRadius * std::sin(end));
    const QPointF forward(std::sin(end), std::cos(end));
    const QPointF side(-forward.y(), forward.x());
    QPolygonF head;
    head << tip << tip - forward * 3.6 + side * 2.2
         << tip - forward * 3.6 - side * 2.2;
    p.setPen(Qt::NoPen);
    p.setBrush(ink);
    p.drawPolygon(head);
  });
}

QIcon MakeHopGlyphIcon() {
  return MakeGlyphIcon([](QPainter& p, const QRectF& r) {
    const QPointF center = r.center();
    p.setPen(QPen(QColor(QStringLiteral("#0891b2")), 1.6));
    p.setBrush(Qt::NoBrush);
    p.drawEllipse(center, 6.5, 6.5);
    p.setBrush(QColor(QStringLiteral("#0891b2")));
    p.setPen(Qt::NoPen);
    p.drawEllipse(center, 2.2, 2.2);
  });
}

QToolButton* MakeIconToolButton(QWidget* parent, const QIcon& icon,
                                const QString& tooltip, bool checkable = false) {
  auto* button = new QToolButton(parent);
  button->setObjectName(QStringLiteral("AutovizLightToolbarIcon"));
  button->setIcon(icon);
  button->setIconSize(QSize(16, 16));
  button->setToolButtonStyle(Qt::ToolButtonIconOnly);
  button->setAutoRaise(true);
  button->setCursor(Qt::PointingHandCursor);
  button->setToolTip(tooltip);
  button->setCheckable(checkable);
  button->setFixedSize(24, 22);
  button->setFocusPolicy(Qt::NoFocus);
  return button;
}

QString FormatHzText(double hz) {
  if (hz <= 0.0) {
    return QStringLiteral("—");
  }
  if (hz >= 100.0) {
    return QString::number(hz, 'f', 0);
  }
  if (hz >= 10.0) {
    return QString::number(hz, 'f', 1);
  }
  return QString::number(hz, 'f', 2);
}

QString KindTitle(integration::GraphVertexKind kind) {
  switch (kind) {
    case integration::GraphVertexKind::kNode:
      return QObject::tr("Node");
    case integration::GraphVertexKind::kChannel:
      return QObject::tr("Channel");
    case integration::GraphVertexKind::kService:
      return QObject::tr("Service");
  }
  return QObject::tr("Vertex");
}

QColor KindAccent(integration::GraphVertexKind kind) {
  switch (kind) {
    case integration::GraphVertexKind::kNode:
      return QColor(QLatin1String(kNodeColor));
    case integration::GraphVertexKind::kChannel:
      return QColor(QLatin1String(kChannelColor));
    case integration::GraphVertexKind::kService:
      return QColor(QLatin1String(kServiceColor));
  }
  return QColor(120, 120, 120);
}

QString KindGlyph(integration::GraphVertexKind kind) {
  switch (kind) {
    case integration::GraphVertexKind::kNode:
      return QStringLiteral("▢");
    case integration::GraphVertexKind::kChannel:
      return QStringLiteral("▬");
    case integration::GraphVertexKind::kService:
      return QStringLiteral("▣");
  }
  return QStringLiteral("•");
}

struct PropertyEntry {
  QString title;
  QString subtitle;
};

struct PropertySectionData {
  QString title;
  QVector<PropertyEntry> entries;
};

struct PropertyPage {
  integration::GraphVertexKind kind = integration::GraphVertexKind::kNode;
  QString name;
  QString subtitle;
  QVector<QPair<QString, QString>> metrics;
  QVector<PropertySectionData> sections;
};

QString RoleHostPid(const autolink::proto::RoleAttributes& attr) {
  QStringList parts;
  if (!attr.host_name().empty()) {
    parts << QString::fromStdString(attr.host_name());
  } else if (!attr.host_ip().empty()) {
    parts << QString::fromStdString(attr.host_ip());
  }
  if (attr.process_id() > 0) {
    parts << QStringLiteral("pid %1").arg(attr.process_id());
  }
  if (!attr.message_type().empty()) {
    parts << QString::fromStdString(attr.message_type());
  }
  return parts.join(QStringLiteral(" · "));
}

void AppendRoleEntries(PropertySectionData* section,
                       const std::vector<autolink::proto::RoleAttributes>& attrs,
                       bool use_channel_name) {
  if (section == nullptr) {
    return;
  }
  for (const auto& attr : attrs) {
    PropertyEntry entry;
    entry.title = use_channel_name
                      ? QString::fromStdString(attr.channel_name())
                      : QString::fromStdString(attr.node_name());
    if (entry.title.isEmpty() && !attr.service_name().empty()) {
      entry.title = QString::fromStdString(attr.service_name());
    }
    entry.subtitle = RoleHostPid(attr);
    section->entries.push_back(entry);
  }
  if (section->entries.isEmpty()) {
    section->entries.push_back({QObject::tr("None"), QString()});
  }
}

PropertyPage BuildNodePage(const QString& node_name) {
  PropertyPage page;
  page.kind = integration::GraphVertexKind::kNode;
  page.name = node_name;
  page.subtitle = QObject::tr("Process endpoint in the Autolink topology");

  auto* topology = autolink::service_discovery::TopologyManager::Instance();
  if (topology == nullptr) {
    return page;
  }

  if (topology->channel_manager() != nullptr) {
    std::vector<autolink::proto::RoleAttributes> writers;
    std::vector<autolink::proto::RoleAttributes> readers;
    topology->channel_manager()->GetWritersOfNode(node_name.toStdString(), &writers);
    topology->channel_manager()->GetReadersOfNode(node_name.toStdString(), &readers);
    std::sort(writers.begin(), writers.end(),
              [](const auto& a, const auto& b) {
                return a.channel_name() < b.channel_name();
              });
    std::sort(readers.begin(), readers.end(),
              [](const auto& a, const auto& b) {
                return a.channel_name() < b.channel_name();
              });

    page.metrics.push_back(
        {QObject::tr("Publishes"), QString::number(writers.size())});
    page.metrics.push_back(
        {QObject::tr("Subscribes"), QString::number(readers.size())});

    PropertySectionData pub;
    pub.title = QObject::tr("Publishes");
    AppendRoleEntries(&pub, writers, true);
    page.sections.push_back(pub);

    PropertySectionData sub;
    sub.title = QObject::tr("Subscribes");
    AppendRoleEntries(&sub, readers, true);
    page.sections.push_back(sub);
  }

  if (topology->service_manager() != nullptr) {
    std::vector<autolink::proto::RoleAttributes> servers;
    topology->service_manager()->GetServers(&servers);
    std::vector<autolink::proto::RoleAttributes> as_server;
    std::vector<autolink::proto::RoleAttributes> as_client;
    for (const auto& server : servers) {
      if (server.node_name() == node_name.toStdString()) {
        as_server.push_back(server);
      }
      std::vector<autolink::proto::RoleAttributes> clients;
      topology->service_manager()->GetClients(server.service_name(), &clients);
      for (auto client : clients) {
        if (client.node_name() == node_name.toStdString()) {
          client.set_service_name(server.service_name());
          as_client.push_back(client);
        }
      }
    }

    page.metrics.push_back(
        {QObject::tr("Servers"), QString::number(as_server.size())});
    page.metrics.push_back(
        {QObject::tr("Clients"), QString::number(as_client.size())});

    PropertySectionData srv;
    srv.title = QObject::tr("Service servers");
    for (const auto& attr : as_server) {
      srv.entries.push_back(
          {QString::fromStdString(attr.service_name()), RoleHostPid(attr)});
    }
    if (srv.entries.isEmpty()) {
      srv.entries.push_back({QObject::tr("None"), QString()});
    }
    page.sections.push_back(srv);

    PropertySectionData cli;
    cli.title = QObject::tr("Service clients");
    for (const auto& attr : as_client) {
      cli.entries.push_back(
          {QString::fromStdString(attr.service_name()), RoleHostPid(attr)});
    }
    if (cli.entries.isEmpty()) {
      cli.entries.push_back({QObject::tr("None"), QString()});
    }
    page.sections.push_back(cli);
  }
  return page;
}

PropertyPage BuildChannelPage(const QString& channel_name,
                              const QString& message_type) {
  PropertyPage page;
  page.kind = integration::GraphVertexKind::kChannel;
  page.name = channel_name;
  page.subtitle = message_type.isEmpty() ? QObject::tr("Message channel")
                                         : message_type;

  const integration::ChannelStats stats =
      integration::ChannelStatsRegistry::instance().stats(
          channel_name.toStdString());
  page.metrics.push_back(
      {QObject::tr("Hz"), FormatHzText(stats.frequency_hz)});
  page.metrics.push_back(
      {QObject::tr("Messages"),
       stats.message_count > 0 ? QString::number(stats.message_count)
                               : QStringLiteral("—")});

  auto* topology = autolink::service_discovery::TopologyManager::Instance();
  if (topology == nullptr || topology->channel_manager() == nullptr) {
    return page;
  }

  std::vector<autolink::proto::RoleAttributes> writers;
  std::vector<autolink::proto::RoleAttributes> readers;
  topology->channel_manager()->GetWritersOfChannel(channel_name.toStdString(),
                                                   &writers);
  topology->channel_manager()->GetReadersOfChannel(channel_name.toStdString(),
                                                   &readers);
  std::sort(writers.begin(), writers.end(),
            [](const auto& a, const auto& b) {
              return a.node_name() < b.node_name();
            });
  std::sort(readers.begin(), readers.end(),
            [](const auto& a, const auto& b) {
              return a.node_name() < b.node_name();
            });

  page.metrics.push_back(
      {QObject::tr("Writers"), QString::number(writers.size())});
  page.metrics.push_back(
      {QObject::tr("Readers"), QString::number(readers.size())});

  PropertySectionData pub;
  pub.title = QObject::tr("Writers");
  AppendRoleEntries(&pub, writers, false);
  page.sections.push_back(pub);

  PropertySectionData sub;
  sub.title = QObject::tr("Readers");
  AppendRoleEntries(&sub, readers, false);
  page.sections.push_back(sub);
  return page;
}

PropertyPage BuildServicePage(const QString& service_name,
                              const QString& message_type) {
  PropertyPage page;
  page.kind = integration::GraphVertexKind::kService;
  page.name = service_name;
  page.subtitle = message_type.isEmpty() ? QObject::tr("RPC service")
                                         : message_type;

  auto* topology = autolink::service_discovery::TopologyManager::Instance();
  if (topology == nullptr || topology->service_manager() == nullptr) {
    return page;
  }

  std::vector<autolink::proto::RoleAttributes> servers;
  topology->service_manager()->GetServers(&servers);
  std::vector<autolink::proto::RoleAttributes> matched_servers;
  for (const auto& server : servers) {
    if (server.service_name() == service_name.toStdString()) {
      matched_servers.push_back(server);
    }
  }
  std::vector<autolink::proto::RoleAttributes> clients;
  topology->service_manager()->GetClients(service_name.toStdString(), &clients);
  std::sort(clients.begin(), clients.end(),
            [](const auto& a, const auto& b) {
              return a.node_name() < b.node_name();
            });

  page.metrics.push_back(
      {QObject::tr("Servers"), QString::number(matched_servers.size())});
  page.metrics.push_back(
      {QObject::tr("Clients"), QString::number(clients.size())});

  PropertySectionData srv;
  srv.title = QObject::tr("Servers");
  AppendRoleEntries(&srv, matched_servers, false);
  page.sections.push_back(srv);

  PropertySectionData cli;
  cli.title = QObject::tr("Clients");
  AppendRoleEntries(&cli, clients, false);
  page.sections.push_back(cli);
  return page;
}

QWidget* MakeMetricChip(const QString& label, const QString& value,
                        const QColor& accent, QWidget* parent) {
  auto* chip = new QFrame(parent);
  chip->setObjectName(QStringLiteral("PropertyMetricChip"));
  chip->setStyleSheet(
      style::sheet(QStringLiteral("channel_graph"), style::Tone::Frost));
  auto* layout = new QVBoxLayout(chip);
  layout->setContentsMargins(12, 8, 12, 8);
  layout->setSpacing(2);

  auto* value_label = new QLabel(value, chip);
  value_label->setAlignment(Qt::AlignCenter);
  QFont value_font = value_label->font();
  value_font.setPointSize(13);
  value_font.setBold(true);
  value_label->setFont(value_font);
  QHash<QString, QString> value_tokens = style::tokens(style::Tone::Frost);
  value_tokens.insert(QStringLiteral("{{accent-color}}"), accent.name());
  value_label->setStyleSheet(
      style::sheet(QStringLiteral("channel_graph"), value_tokens));

  auto* key_label = new QLabel(label, chip);
  key_label->setAlignment(Qt::AlignCenter);
  key_label->setStyleSheet(
      style::sheet(QStringLiteral("channel_graph"), style::Tone::Frost));

  layout->addWidget(value_label);
  layout->addWidget(key_label);
  return chip;
}

QWidget* MakeSectionCard(const PropertySectionData& section, QWidget* parent) {
  auto* card = new QFrame(parent);
  card->setObjectName(QStringLiteral("PropertySectionCard"));
  card->setStyleSheet(
      style::sheet(QStringLiteral("channel_graph"), style::Tone::Frost));
  auto* layout = new QVBoxLayout(card);
  layout->setContentsMargins(14, 12, 14, 12);
  layout->setSpacing(8);

  auto* header = new QHBoxLayout();
  header->setContentsMargins(0, 0, 0, 0);
  auto* title = new QLabel(section.title, card);
  QFont title_font = title->font();
  title_font.setPointSize(11);
  title_font.setBold(true);
  title->setFont(title_font);
  title->setStyleSheet(
      style::sheet(QStringLiteral("channel_graph"), style::Tone::Frost));
  auto* count = new QLabel(QString::number(section.entries.size()), card);
  count->setStyleSheet(
      style::sheet(QStringLiteral("channel_graph"), style::Tone::Frost));
  // Don't count the placeholder "None" as a real entry count for empty lists.
  if (section.entries.size() == 1 &&
      section.entries.front().title == QObject::tr("None")) {
    count->setText(QStringLiteral("0"));
  }
  header->addWidget(title);
  header->addStretch(1);
  header->addWidget(count);
  layout->addLayout(header);

  for (int i = 0; i < section.entries.size(); ++i) {
    const PropertyEntry& entry = section.entries[i];
    auto* row = new QFrame(card);
    row->setStyleSheet(style::sheet(
        i == 0 ? QStringLiteral("channel_graph/entry_row_top")
               : QStringLiteral("channel_graph/entry_row_divider"),
        style::Tone::Frost));
    auto* row_layout = new QVBoxLayout(row);
    row_layout->setContentsMargins(0, 8, 0, 4);
    row_layout->setSpacing(2);

    auto* name = new QLabel(entry.title, row);
    name->setWordWrap(true);
    name->setTextInteractionFlags(Qt::TextSelectableByMouse);
    name->setStyleSheet(
        style::sheet(QStringLiteral("channel_graph"), style::Tone::Frost));
    row_layout->addWidget(name);

    if (!entry.subtitle.isEmpty()) {
      auto* meta = new QLabel(entry.subtitle, row);
      meta->setWordWrap(true);
      meta->setTextInteractionFlags(Qt::TextSelectableByMouse);
      meta->setStyleSheet(
          style::sheet(QStringLiteral("channel_graph"), style::Tone::Frost));
      row_layout->addWidget(meta);
    }
    layout->addWidget(row);
  }
  return card;
}

QDialog* CreatePropertyDialog(QWidget* parent, const PropertyPage& page) {
  const QColor accent = KindAccent(page.kind);
  auto* dialog = new QDialog(parent);
  dialog->setAttribute(Qt::WA_DeleteOnClose);
  dialog->setWindowTitle(
      QObject::tr("%1 properties").arg(KindTitle(page.kind)));
  dialog->resize(520, 560);
  dialog->setStyleSheet(
      style::sheet(QStringLiteral("channel_graph"), style::Tone::Frost));

  auto* root = new QVBoxLayout(dialog);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* header = new QFrame(dialog);
  QHash<QString, QString> header_tokens = style::tokens(style::Tone::Frost);
  header_tokens.insert(
      QStringLiteral("{{header-start}}"),
      QColor(accent.red(), accent.green(), accent.blue(), 36).name(QColor::HexArgb));
  header->setStyleSheet(
      style::sheet(QStringLiteral("channel_graph"), header_tokens));
  auto* header_layout = new QVBoxLayout(header);
  header_layout->setContentsMargins(18, 16, 18, 16);
  header_layout->setSpacing(10);

  auto* top_row = new QHBoxLayout();
  auto* badge = new QLabel(
      QStringLiteral("%1  %2").arg(KindGlyph(page.kind), KindTitle(page.kind)),
      header);
  QHash<QString, QString> badge_tokens = style::tokens(style::Tone::Frost);
  badge_tokens.insert(QStringLiteral("{{accent-color}}"), accent.name());
  badge->setStyleSheet(
      style::sheet(QStringLiteral("channel_graph"), badge_tokens));
  top_row->addWidget(badge, 0, Qt::AlignLeft | Qt::AlignVCenter);
  top_row->addStretch(1);

  auto* close_button = new QPushButton(QObject::tr("Close"), header);
  close_button->setCursor(Qt::PointingHandCursor);
  close_button->setStyleSheet(
      style::sheet(QStringLiteral("channel_graph"), style::Tone::Frost));
  QObject::connect(close_button, &QPushButton::clicked, dialog, &QDialog::accept);
  top_row->addWidget(close_button, 0, Qt::AlignRight);
  header_layout->addLayout(top_row);

  auto* name = new QLabel(page.name, header);
  name->setWordWrap(true);
  name->setTextInteractionFlags(Qt::TextSelectableByMouse);
  QFont name_font = name->font();
  name_font.setPointSize(16);
  name_font.setBold(true);
  name->setFont(name_font);
  name->setStyleSheet(
      style::sheet(QStringLiteral("channel_graph"), style::Tone::Frost));
  header_layout->addWidget(name);

  if (!page.subtitle.isEmpty()) {
    auto* subtitle = new QLabel(page.subtitle, header);
    subtitle->setWordWrap(true);
    subtitle->setTextInteractionFlags(Qt::TextSelectableByMouse);
    subtitle->setStyleSheet(style::type(style::Role::PanelMuted, 12));
    header_layout->addWidget(subtitle);
  }

  if (!page.metrics.isEmpty()) {
    auto* metrics = new QHBoxLayout();
    metrics->setSpacing(8);
    for (const auto& metric : page.metrics) {
      metrics->addWidget(
          MakeMetricChip(metric.first, metric.second, accent, header), 1);
    }
    header_layout->addLayout(metrics);
  }
  root->addWidget(header);

  auto* scroll = new QScrollArea(dialog);
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  auto* body = new QWidget(scroll);
  body->setStyleSheet(style::sheet(QStringLiteral("channel_graph"), style::Tone::Frost));
  auto* body_layout = new QVBoxLayout(body);
  body_layout->setContentsMargins(16, 16, 16, 16);
  body_layout->setSpacing(12);
  for (const PropertySectionData& section : page.sections) {
    body_layout->addWidget(MakeSectionCard(section, body));
  }
  body_layout->addStretch(1);
  scroll->setWidget(body);
  root->addWidget(scroll, 1);

  return dialog;
}

}  // namespace

ChannelGraphPanelConfig DefaultChannelGraphPanelConfig() {
  ChannelGraphPanelConfig config;
  config.show_services = true;
  config.show_channels = true;
  config.auto_refresh = true;
  config.neighborhood_mode = true;
  config.show_edge_labels = false;
  config.quiet_mode = false;
  config.hide_leaf_channels = false;
  config.hide_dead_end_channels = false;
  config.probe_enabled = false;
  config.channel_arrange = VertexArrangeMode::kGrid;
  config.service_arrange = VertexArrangeMode::kGrid;
  return config;
}

ChannelGraphPanel::ChannelGraphPanel(common::VisualizationManager* manager,
                                     QWidget* parent)
    : manager_(manager), config_(DefaultChannelGraphPanelConfig()), QWidget(parent) {
  Q_UNUSED(manager_);
  setFocusPolicy(Qt::StrongFocus);
  setupUi();
  applyConfigToUi();
  rebuildPrefixCombo();
  refreshGraph();
}

ChannelGraphPanel::~ChannelGraphPanel() { clearStatsProbes(); }

void ChannelGraphPanel::setupUi() {
  ApplyPanelShell(this);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  graph_tools_ = new QWidget(this);
  auto* row = new QHBoxLayout(graph_tools_);
  row->setContentsMargins(0, 0, 0, 0);
  row->setSpacing(0);

  auto* zoom_fit_button = MakeIconToolButton(
      graph_tools_, IconLoader::panelTitleIcon(QStringLiteral("plot.reset_view")),
      tr("Zoom to fit"));
  row->addWidget(zoom_fit_button);

  auto* refresh_button = MakeIconToolButton(
      graph_tools_, MakeRefreshGlyphIcon(), tr("Refresh"));
  row->addWidget(refresh_button);

  show_channels_button_ = MakeIconToolButton(
      graph_tools_, MakeChannelGlyphIcon(), tr("Show channels"), true);
  show_channels_button_->setChecked(true);
  row->addWidget(show_channels_button_);

  show_services_button_ = MakeIconToolButton(
      graph_tools_, MakeServiceGlyphIcon(), tr("Show services"), true);
  show_services_button_->setChecked(true);
  row->addWidget(show_services_button_);

  neighborhood_button_ = MakeIconToolButton(
      graph_tools_, MakeHopGlyphIcon(),
      tr("Focus neighbors of the selection"), true);
  neighborhood_button_->setChecked(true);
  row->addWidget(neighborhood_button_);

  filter_edit_ = new QLineEdit(graph_tools_);
  filter_edit_->setObjectName(QStringLiteral("AutovizCompactFilter"));
  filter_edit_->setPlaceholderText(tr("Filter"));
  filter_edit_->setToolTip(
      tr("Comma-separated terms. Prefix with - to exclude "
         "(e.g. sensing,-/tf,-parameter)."));
  filter_edit_->setClearButtonEnabled(true);
  filter_edit_->setFixedWidth(140);
  filter_edit_->setFixedHeight(22);
  filter_edit_->setStyleSheet(QStringLiteral(
      "QLineEdit { background: transparent; border: none;"
      " border-bottom: 1px solid palette(mid); padding: 0 4px;"
      " font-size: 12px; }"
      "QLineEdit:focus { border-bottom: 1px solid palette(highlight); }"));
  row->addWidget(filter_edit_);

  graph_view_ = new ChannelGraphView(this);
  graph_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  root->addWidget(graph_view_, 1);

  auto* status_bar = new QFrame(this);
  ApplyPanelFooterChrome(status_bar);
  auto* status_layout = new QHBoxLayout(status_bar);
  status_layout->setContentsMargins(PanelChromeLayout::kFooterMarginH,
                                    PanelChromeLayout::kFooterMarginV,
                                    PanelChromeLayout::kFooterMarginH,
                                    PanelChromeLayout::kFooterMarginV);
  status_label_ = new QLabel(status_bar);
  StylePanelStatusLabel(status_label_);
  status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  status_layout->addWidget(status_label_, 1);
  root->addWidget(status_bar);

  refresh_timer_ = new QTimer(this);
  refresh_timer_->setInterval(kAutoRefreshMs);
  filter_timer_ = new QTimer(this);
  filter_timer_->setSingleShot(true);
  filter_timer_->setInterval(kFilterDebounceMs);

  connect(zoom_fit_button, &QToolButton::clicked, this,
          &ChannelGraphPanel::onZoomFitClicked);
  connect(refresh_button, &QToolButton::clicked, this,
          &ChannelGraphPanel::onRefreshClicked);
  connect(show_services_button_, &QToolButton::toggled, this,
          &ChannelGraphPanel::onShowServicesToggled);
  connect(show_channels_button_, &QToolButton::toggled, this,
          &ChannelGraphPanel::onShowChannelsToggled);
  connect(neighborhood_button_, &QToolButton::toggled, this,
          &ChannelGraphPanel::onNeighborhoodToggled);
  connect(filter_edit_, &QLineEdit::textChanged, this,
          &ChannelGraphPanel::onFilterChanged);
  connect(filter_timer_, &QTimer::timeout, this, &ChannelGraphPanel::refreshGraph);
  connect(refresh_timer_, &QTimer::timeout, this, &ChannelGraphPanel::refreshGraph);
  connect(graph_view_, &ChannelGraphView::graphRendered, this,
          &ChannelGraphPanel::onGraphRendered);
  connect(graph_view_, &ChannelGraphView::vertexDoubleClicked, this,
          &ChannelGraphPanel::onVertexDoubleClicked);
  connect(graph_view_, &ChannelGraphView::vertexContextMenuRequested, this,
          &ChannelGraphPanel::onVertexContextMenuRequested);
  connect(graph_view_, &ChannelGraphView::vertexSelectionChanged, this,
          &ChannelGraphPanel::onVertexSelectionChanged);
}

void ChannelGraphPanel::installTitleBarTools(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  PanelContextMenuCallbacks callbacks;
  callbacks.current_object_name = QStringLiteral("ChannelGraphDock");
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
  if (graph_tools_ != nullptr) {
    if (auto* layout = qobject_cast<QHBoxLayout*>(tools.widget->layout())) {
      graph_tools_->setParent(tools.widget);
      layout->insertWidget(0, CreateTitleSeparator(tools.widget));
      layout->insertWidget(0, graph_tools_);
    }
  }
  dock->setTitleBarTools(tools.widget);
}

void ChannelGraphPanel::setExpandButtonChecked(bool checked) {
  if (expand_button_ == nullptr) {
    return;
  }
  expand_button_->blockSignals(true);
  expand_button_->setChecked(checked);
  expand_button_->blockSignals(false);
}

ChannelGraphPanelConfig ChannelGraphPanel::config() const { return config_; }

void ChannelGraphPanel::setConfig(const ChannelGraphPanelConfig& config) {
  config_ = config;
  applyConfigToUi();
  rebuildPrefixCombo();
  if (!config_.probe_enabled) {
    clearStatsProbes();
  }
  refreshGraph();
}

void ChannelGraphPanel::cloneConfigFrom(const ChannelGraphPanelConfig& config) {
  setConfig(config);
}

void ChannelGraphPanel::refreshGraph() {
  integration::TopologyGraphBuildOptions options;
  options.show_services = config_.show_services;
  options.show_channels = config_.show_channels;
  options.quiet_mode = config_.quiet_mode;
  options.hide_leaf_channels = config_.hide_leaf_channels;
  options.hide_dead_end_channels = config_.hide_dead_end_channels;
  options.filter_text = config_.filter.toStdString();
  options.prefix_filter = config_.prefix_filter.toStdString();

  applyViewOptions();

  const integration::TopologyGraph graph = integration::BuildTopologyGraph(options);
  const bool topology_unchanged =
      !graph.topology_hash.empty() &&
      QString::fromStdString(graph.topology_hash) == last_topology_hash_;
  if (!topology_unchanged) {
    last_topology_hash_ = QString::fromStdString(graph.topology_hash);
  }
  graph_view_->setGraph(graph, topology_unchanged);
  syncStatsProbes();
  if (graph.vertices.empty()) {
    updateStatusText(tr("No matching topology"));
  } else if (!selected_vertex_id_.isEmpty()) {
    updateStatusText(formatSelectionSummary(selected_vertex_kind_,
                                            selected_vertex_label_,
                                            selected_vertex_detail_));
  }
}

void ChannelGraphPanel::focusInEvent(QFocusEvent* event) {
  QWidget::focusInEvent(event);
  emit activated();
}

void ChannelGraphPanel::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  if (config_.auto_refresh) {
    refresh_timer_->start();
  }
  refreshGraph();
}

void ChannelGraphPanel::hideEvent(QHideEvent* event) {
  refresh_timer_->stop();
  QWidget::hideEvent(event);
}

void ChannelGraphPanel::onFilterChanged(const QString& text) {
  config_.filter = text;
  emit configChanged();
  filter_timer_->start();
}

void ChannelGraphPanel::onPrefixChanged(const QString& prefix) {
  if (config_.prefix_filter == prefix) {
    return;
  }
  config_.prefix_filter = prefix;
  emit configChanged();
  refreshGraph();
}

void ChannelGraphPanel::onShowServicesToggled(bool enabled) {
  config_.show_services = enabled;
  emit configChanged();
  refreshGraph();
}

void ChannelGraphPanel::onShowChannelsToggled(bool enabled) {
  config_.show_channels = enabled;
  emit configChanged();
  graph_view_->resetSavedPositions();
  refreshGraph();
}

void ChannelGraphPanel::onNeighborhoodToggled(bool enabled) {
  config_.neighborhood_mode = enabled;
  emit configChanged();
  applyViewOptions();
}

void ChannelGraphPanel::onShowEdgeLabelsToggled(bool enabled) {
  config_.show_edge_labels = enabled;
  emit configChanged();
  applyViewOptions();
}

void ChannelGraphPanel::onQuietModeToggled(bool enabled) {
  config_.quiet_mode = enabled;
  emit configChanged();
  last_topology_hash_.clear();
  refreshGraph();
}

void ChannelGraphPanel::onHideLeafToggled(bool enabled) {
  config_.hide_leaf_channels = enabled;
  emit configChanged();
  last_topology_hash_.clear();
  refreshGraph();
}

void ChannelGraphPanel::onHideDeadEndToggled(bool enabled) {
  config_.hide_dead_end_channels = enabled;
  emit configChanged();
  last_topology_hash_.clear();
  refreshGraph();
}

void ChannelGraphPanel::onProbeToggled(bool enabled) {
  config_.probe_enabled = enabled;
  emit configChanged();
  if (!config_.probe_enabled) {
    clearStatsProbes();
  } else {
    syncStatsProbes();
  }
}

void ChannelGraphPanel::onChannelArrangeChanged(VertexArrangeMode mode) {
  if (config_.channel_arrange == mode) {
    return;
  }
  config_.channel_arrange = mode;
  emit configChanged();
  last_topology_hash_.clear();
  graph_view_->resetSavedPositions();
  refreshGraph();
  graph_view_->zoomToFit();
}

void ChannelGraphPanel::onServiceArrangeChanged(VertexArrangeMode mode) {
  if (config_.service_arrange == mode) {
    return;
  }
  config_.service_arrange = mode;
  emit configChanged();
  last_topology_hash_.clear();
  graph_view_->resetSavedPositions();
  refreshGraph();
  graph_view_->zoomToFit();
}

void ChannelGraphPanel::onAutoRefreshToggled(bool enabled) {
  config_.auto_refresh = enabled;
  emit configChanged();
  if (enabled && isVisible()) {
    refresh_timer_->start();
  } else {
    refresh_timer_->stop();
  }
}

void ChannelGraphPanel::onRefreshClicked() {
  rebuildPrefixCombo();
  last_topology_hash_.clear();
  graph_view_->resetSavedPositions();
  refreshGraph();
}

void ChannelGraphPanel::onZoomFitClicked() { graph_view_->zoomToFit(); }

void ChannelGraphPanel::onGraphRendered(int vertex_count, int edge_count) {
  idle_status_text_ =
      tr("%1 vertices · %2 connections · click to focus · double-click for details")
          .arg(vertex_count)
          .arg(edge_count);
  if (!selected_vertex_id_.isEmpty()) {
    updateStatusText(formatSelectionSummary(selected_vertex_kind_,
                                            selected_vertex_label_,
                                            selected_vertex_detail_));
  } else {
    updateStatusText(idle_status_text_);
  }
}

void ChannelGraphPanel::onVertexDoubleClicked(
    const QString& vertex_id, integration::GraphVertexKind kind,
    const QString& label, const QString& detail) {
  Q_UNUSED(vertex_id);
  showVertexProperties(kind, label, detail);
}

void ChannelGraphPanel::onVertexSelectionChanged(
    bool has_selection, const QString& vertex_id,
    integration::GraphVertexKind kind, const QString& label,
    const QString& detail) {
  if (!has_selection) {
    selected_vertex_id_.clear();
    selected_vertex_label_.clear();
    selected_vertex_detail_.clear();
    updateStatusText(idle_status_text_.isEmpty()
                         ? tr("Click a vertex for details")
                         : idle_status_text_);
    return;
  }
  selected_vertex_id_ = vertex_id;
  selected_vertex_kind_ = kind;
  selected_vertex_label_ = label;
  selected_vertex_detail_ = detail;
  updateStatusText(formatSelectionSummary(kind, label, detail));
}

void ChannelGraphPanel::onVertexContextMenuRequested(
    const QString& vertex_id, integration::GraphVertexKind kind,
    const QString& label, const QString& detail, const QPoint& global_pos) {
  Q_UNUSED(vertex_id);
  QMenu menu(this);
  QAction* copy_action = menu.addAction(
      kind == integration::GraphVertexKind::kChannel ? tr("Copy channel path")
                                                     : tr("Copy name"));
  QAction* open_raw_action = nullptr;
  QAction* add_plot_action = nullptr;
  QAction* open_table_action = nullptr;
  QString plot_field;
  QString table_field;

  if (kind == integration::GraphVertexKind::kChannel && !label.isEmpty()) {
    open_raw_action = menu.addAction(tr("Open in Raw Messages"));
    const QStringList numeric =
        plot::NumericFieldPathsForMessageType(detail.toStdString());
    if (!numeric.isEmpty()) {
      plot_field = numeric.first();
      add_plot_action =
          menu.addAction(tr("Add to Plot (%1)").arg(plot_field));
    }
    const QStringList arrays =
        plot::TableArrayFieldPathsForMessageType(detail.toStdString());
    if (!arrays.isEmpty()) {
      table_field = arrays.first();
      open_table_action =
          menu.addAction(tr("Open in Table (%1)").arg(table_field));
    }
  }

  QAction* chosen = menu.exec(global_pos);
  if (chosen == nullptr) {
    return;
  }
  if (chosen == copy_action) {
    if (QClipboard* clipboard = QApplication::clipboard()) {
      clipboard->setText(label);
    }
    return;
  }
  if (chosen == open_raw_action) {
    emit openInRawMessagesRequested(label);
    return;
  }
  if (chosen == add_plot_action && !plot_field.isEmpty()) {
    emit addToPlotRequested(label, plot_field);
    return;
  }
  if (chosen == open_table_action && !table_field.isEmpty()) {
    emit openInTableRequested(label, table_field);
  }
}

void ChannelGraphPanel::showVertexProperties(integration::GraphVertexKind kind,
                                             const QString& label,
                                             const QString& detail) {
  PropertyPage page;
  switch (kind) {
    case integration::GraphVertexKind::kNode:
      page = BuildNodePage(label);
      break;
    case integration::GraphVertexKind::kChannel:
      page = BuildChannelPage(label, detail);
      break;
    case integration::GraphVertexKind::kService:
      page = BuildServicePage(label, detail);
      break;
  }

  QDialog* dialog = CreatePropertyDialog(this, page);
  dialog->show();
  dialog->raise();
  dialog->activateWindow();
}

QString ChannelGraphPanel::formatSelectionSummary(
    integration::GraphVertexKind kind, const QString& label,
    const QString& detail) const {
  auto* topology = autolink::service_discovery::TopologyManager::Instance();
  switch (kind) {
    case integration::GraphVertexKind::kChannel: {
      const integration::ChannelStats stats =
          integration::ChannelStatsRegistry::instance().stats(
              label.toStdString());
      int writers = 0;
      int readers = 0;
      if (topology != nullptr && topology->channel_manager() != nullptr) {
        std::vector<autolink::proto::RoleAttributes> writer_attrs;
        std::vector<autolink::proto::RoleAttributes> reader_attrs;
        topology->channel_manager()->GetWritersOfChannel(label.toStdString(),
                                                         &writer_attrs);
        topology->channel_manager()->GetReadersOfChannel(label.toStdString(),
                                                         &reader_attrs);
        writers = static_cast<int>(writer_attrs.size());
        readers = static_cast<int>(reader_attrs.size());
      }
      const QString type =
          detail.isEmpty() ? tr("unknown type") : detail;
      return tr("Channel · %1 · %2 · %3 Hz · %4 writers / %5 readers")
          .arg(label, type, FormatHzText(stats.frequency_hz))
          .arg(writers)
          .arg(readers);
    }
    case integration::GraphVertexKind::kNode: {
      int pubs = 0;
      int subs = 0;
      int servers = 0;
      int clients = 0;
      if (topology != nullptr) {
        if (topology->channel_manager() != nullptr) {
          std::vector<autolink::proto::RoleAttributes> writers;
          std::vector<autolink::proto::RoleAttributes> readers;
          topology->channel_manager()->GetWritersOfNode(label.toStdString(),
                                                        &writers);
          topology->channel_manager()->GetReadersOfNode(label.toStdString(),
                                                        &readers);
          pubs = static_cast<int>(writers.size());
          subs = static_cast<int>(readers.size());
        }
        if (topology->service_manager() != nullptr) {
          std::vector<autolink::proto::RoleAttributes> all_servers;
          topology->service_manager()->GetServers(&all_servers);
          for (const auto& server : all_servers) {
            if (server.node_name() == label.toStdString()) {
              ++servers;
            }
            std::vector<autolink::proto::RoleAttributes> srv_clients;
            topology->service_manager()->GetClients(server.service_name(),
                                                    &srv_clients);
            for (const auto& client : srv_clients) {
              if (client.node_name() == label.toStdString()) {
                ++clients;
              }
            }
          }
        }
      }
      return tr("Node · %1 · %2 pub / %3 sub · %4 srv / %5 cli")
          .arg(label)
          .arg(pubs)
          .arg(subs)
          .arg(servers)
          .arg(clients);
    }
    case integration::GraphVertexKind::kService: {
      int server_count = 0;
      int client_count = 0;
      if (topology != nullptr && topology->service_manager() != nullptr) {
        std::vector<autolink::proto::RoleAttributes> servers;
        topology->service_manager()->GetServers(&servers);
        for (const auto& server : servers) {
          if (server.service_name() == label.toStdString()) {
            ++server_count;
          }
        }
        std::vector<autolink::proto::RoleAttributes> clients;
        topology->service_manager()->GetClients(label.toStdString(), &clients);
        client_count = static_cast<int>(clients.size());
      }
      const QString type =
          detail.isEmpty() ? tr("service") : detail;
      return tr("Service · %1 · %2 · %3 servers / %4 clients")
          .arg(label, type)
          .arg(server_count)
          .arg(client_count);
    }
  }
  return label;
}

void ChannelGraphPanel::applyConfigToUi() {
  const auto set_toggle = [](QToolButton* button, bool checked) {
    if (button == nullptr) {
      return;
    }
    button->blockSignals(true);
    button->setChecked(checked);
    button->blockSignals(false);
  };
  set_toggle(show_services_button_, config_.show_services);
  set_toggle(show_channels_button_, config_.show_channels);
  set_toggle(neighborhood_button_, config_.neighborhood_mode);
  if (filter_edit_ != nullptr) {
    filter_edit_->setText(config_.filter);
  }
  applyViewOptions();
}

void ChannelGraphPanel::applyViewOptions() {
  if (graph_view_ == nullptr) {
    return;
  }
  graph_view_->setArrangeModes(config_.channel_arrange, config_.service_arrange);
  graph_view_->setNeighborhoodMode(config_.neighborhood_mode);
  graph_view_->setShowEdgeLabels(config_.show_edge_labels);
}

void ChannelGraphPanel::rebuildPrefixCombo() {}

void ChannelGraphPanel::updateStatusText(const QString& text) {
  status_label_->setText(text);
}

void ChannelGraphPanel::clearStatsProbes() {
  for (const auto& entry : probe_subscriptions_) {
    integration::ChannelReaderRegistry::instance().unsubscribe(entry.second);
  }
  probe_subscriptions_.clear();
}

void ChannelGraphPanel::syncStatsProbes() {
  if (!config_.probe_enabled || graph_view_ == nullptr) {
    clearStatsProbes();
    return;
  }

  constexpr int kMaxProbeChannels = 48;
  std::unordered_map<std::string, std::string> wanted;
  for (const auto& entry : graph_view_->channelVertices()) {
    if (entry.first.isEmpty()) {
      continue;
    }
    if (static_cast<int>(wanted.size()) >= kMaxProbeChannels) {
      break;
    }
    wanted.emplace(entry.first.toStdString(), entry.second.toStdString());
  }

  for (auto it = probe_subscriptions_.begin(); it != probe_subscriptions_.end();) {
    if (wanted.find(it->first) == wanted.end()) {
      integration::ChannelReaderRegistry::instance().unsubscribe(it->second);
      it = probe_subscriptions_.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto& entry : wanted) {
    if (probe_subscriptions_.count(entry.first) > 0) {
      continue;
    }
    const auto id = integration::ChannelReaderRegistry::instance().subscribe(
        entry.first, [](const std::string& /*payload*/) {});
    if (id != 0) {
      probe_subscriptions_.emplace(entry.first, id);
    }
  }
}

}  // namespace channel_graph
}  // namespace autoviz
