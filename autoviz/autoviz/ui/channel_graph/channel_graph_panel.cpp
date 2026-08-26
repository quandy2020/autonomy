/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/channel_graph/channel_graph_panel.hpp"

#include <algorithm>
#include <utility>

#include <QAbstractItemView>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDialog>
#include <QFocusEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
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
#include "autoviz/integration/channel_stats_registry.hpp"
#include "autoviz/integration/topology_graph_builder.hpp"
#include "autoviz/ui/channel_graph/channel_graph_view.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"
#include "autoviz/ui/panel_context_menu.hpp"
#include "autoviz/ui/panel_dock_widget.hpp"
#include "autoviz/ui/panel_title_tools.hpp"

namespace autoviz {
namespace channel_graph {
namespace {

constexpr int kAutoRefreshMs = 2000;
constexpr int kFilterDebounceMs = 250;

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
      return QColor(66, 133, 244);
    case integration::GraphVertexKind::kChannel:
      return QColor(156, 39, 176);
    case integration::GraphVertexKind::kService:
      return QColor(229, 57, 53);
  }
  return QColor(120, 120, 120);
}

QString KindGlyph(integration::GraphVertexKind kind) {
  switch (kind) {
    case integration::GraphVertexKind::kNode:
      return QStringLiteral("●");
    case integration::GraphVertexKind::kChannel:
      return QStringLiteral("▬");
    case integration::GraphVertexKind::kService:
      return QStringLiteral("⬡");
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
  chip->setStyleSheet(QStringLiteral(
      "QFrame#PropertyMetricChip {"
      "  background: rgba(255,255,255,0.05);"
      "  border: 1px solid rgba(255,255,255,0.08);"
      "  border-radius: 10px;"
      "  padding: 2px;"
      "}"));
  auto* layout = new QVBoxLayout(chip);
  layout->setContentsMargins(12, 8, 12, 8);
  layout->setSpacing(2);

  auto* value_label = new QLabel(value, chip);
  value_label->setAlignment(Qt::AlignCenter);
  QFont value_font = value_label->font();
  value_font.setPointSize(13);
  value_font.setBold(true);
  value_label->setFont(value_font);
  value_label->setStyleSheet(
      QStringLiteral("color: %1;").arg(accent.name()));

  auto* key_label = new QLabel(label, chip);
  key_label->setAlignment(Qt::AlignCenter);
  key_label->setStyleSheet(QStringLiteral("color: #9aa0a6; font-size: 11px;"));

  layout->addWidget(value_label);
  layout->addWidget(key_label);
  return chip;
}

QWidget* MakeSectionCard(const PropertySectionData& section, QWidget* parent) {
  auto* card = new QFrame(parent);
  card->setObjectName(QStringLiteral("PropertySectionCard"));
  card->setStyleSheet(QStringLiteral(
      "QFrame#PropertySectionCard {"
      "  background: #1c1c20;"
      "  border: 1px solid #2a2a30;"
      "  border-radius: 12px;"
      "}"));
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
  title->setStyleSheet(QStringLiteral("color: #e8eaed;"));
  auto* count = new QLabel(QString::number(section.entries.size()), card);
  count->setStyleSheet(QStringLiteral(
      "color: #9aa0a6; background: rgba(255,255,255,0.06);"
      "border-radius: 8px; padding: 2px 8px; font-size: 11px;"));
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
    row->setStyleSheet(QStringLiteral(
        "QFrame { background: transparent; border: none;"
        "border-top: %1; }")
                           .arg(i == 0 ? QStringLiteral("none")
                                       : QStringLiteral("1px solid #2a2a30")));
    auto* row_layout = new QVBoxLayout(row);
    row_layout->setContentsMargins(0, 8, 0, 4);
    row_layout->setSpacing(2);

    auto* name = new QLabel(entry.title, row);
    name->setWordWrap(true);
    name->setTextInteractionFlags(Qt::TextSelectableByMouse);
    name->setStyleSheet(QStringLiteral("color: #f1f3f4; font-size: 12px;"));
    row_layout->addWidget(name);

    if (!entry.subtitle.isEmpty()) {
      auto* meta = new QLabel(entry.subtitle, row);
      meta->setWordWrap(true);
      meta->setTextInteractionFlags(Qt::TextSelectableByMouse);
      meta->setStyleSheet(QStringLiteral("color: #9aa0a6; font-size: 11px;"));
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
  dialog->setStyleSheet(QStringLiteral(
      "QDialog {"
      "  background: #141417;"
      "  color: #e8eaed;"
      "}"
      "QScrollArea { border: none; background: transparent; }"
      "QScrollBar:vertical {"
      "  background: transparent; width: 10px; margin: 4px 2px;"
      "}"
      "QScrollBar::handle:vertical {"
      "  background: #3c4043; border-radius: 4px; min-height: 24px;"
      "}"
      "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {"
      "  height: 0;"
      "}"));

  auto* root = new QVBoxLayout(dialog);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* header = new QFrame(dialog);
  header->setStyleSheet(QStringLiteral(
      "QFrame {"
      "  background: qlineargradient(x1:0,y1:0,x2:1,y2:1,"
      "    stop:0 %1, stop:1 #1a1a1f);"
      "  border-bottom: 1px solid #2a2a30;"
      "}")
                            .arg(QColor(accent.red(), accent.green(), accent.blue(), 70)
                                     .name(QColor::HexArgb)));
  auto* header_layout = new QVBoxLayout(header);
  header_layout->setContentsMargins(18, 16, 18, 16);
  header_layout->setSpacing(10);

  auto* top_row = new QHBoxLayout();
  auto* badge = new QLabel(
      QStringLiteral("%1  %2").arg(KindGlyph(page.kind), KindTitle(page.kind)),
      header);
  badge->setStyleSheet(QStringLiteral(
      "color: white; background: %1; border-radius: 11px;"
      "padding: 4px 10px; font-size: 11px; font-weight: 600;")
                           .arg(accent.name()));
  top_row->addWidget(badge, 0, Qt::AlignLeft | Qt::AlignVCenter);
  top_row->addStretch(1);

  auto* close_button = new QPushButton(QObject::tr("Close"), header);
  close_button->setCursor(Qt::PointingHandCursor);
  close_button->setStyleSheet(QStringLiteral(
      "QPushButton {"
      "  color: #e8eaed; background: rgba(255,255,255,0.06);"
      "  border: 1px solid rgba(255,255,255,0.10); border-radius: 8px;"
      "  padding: 6px 14px;"
      "}"
      "QPushButton:hover { background: rgba(255,255,255,0.12); }"));
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
  name->setStyleSheet(QStringLiteral("color: #f8f9fa;"));
  header_layout->addWidget(name);

  if (!page.subtitle.isEmpty()) {
    auto* subtitle = new QLabel(page.subtitle, header);
    subtitle->setWordWrap(true);
    subtitle->setTextInteractionFlags(Qt::TextSelectableByMouse);
    subtitle->setStyleSheet(QStringLiteral("color: #c5c8ce; font-size: 12px;"));
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
  body->setStyleSheet(QStringLiteral("background: #141417;"));
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

void ChannelGraphPanel::setupUi() {
  ApplyPanelShell(this);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(0, 0, 0, 0);
  root->setSpacing(0);

  auto* toolbar = new QFrame(this);
  ApplyPanelToolbarChrome(toolbar);
  auto* toolbar_layout = new QHBoxLayout(toolbar);
  ApplyPanelToolbarLayout(toolbar_layout);

  auto* zoom_fit_button = new QToolButton(toolbar);
  zoom_fit_button->setToolTip(tr("Zoom to fit"));
  zoom_fit_button->setIcon(
      IconLoader::panelTitleIcon(QStringLiteral("plot.reset_view")));
  toolbar_layout->addWidget(zoom_fit_button);

  auto* refresh_button = new QToolButton(toolbar);
  refresh_button->setToolTip(tr("Refresh graph"));
  refresh_button->setIcon(IconLoader::menuIcon(QStringLiteral("file.recent")));
  toolbar_layout->addWidget(refresh_button);

  show_channels_check_ = new QCheckBox(tr("Channels"), toolbar);
  show_channels_check_->setChecked(true);
  show_channels_check_->setToolTip(tr("Show channel vertices in the graph"));
  toolbar_layout->addWidget(show_channels_check_);

  show_services_check_ = new QCheckBox(tr("Services"), toolbar);
  show_services_check_->setChecked(true);
  show_services_check_->setToolTip(tr("Show service vertices in the graph"));
  toolbar_layout->addWidget(show_services_check_);

  neighborhood_check_ = new QCheckBox(tr("1-hop"), toolbar);
  neighborhood_check_->setChecked(true);
  neighborhood_check_->setToolTip(
      tr("When a vertex is selected, hide unrelated vertices and edges"));
  toolbar_layout->addWidget(neighborhood_check_);

  edge_labels_check_ = new QCheckBox(tr("Labels"), toolbar);
  edge_labels_check_->setChecked(false);
  edge_labels_check_->setToolTip(
      tr("Always show edge labels (otherwise only on focus / hover)"));
  toolbar_layout->addWidget(edge_labels_check_);

  toolbar_layout->addWidget(new QLabel(tr("Ch"), toolbar));
  channel_arrange_combo_ = new QComboBox(toolbar);
  channel_arrange_combo_->addItem(tr("Column"),
                                  static_cast<int>(VertexArrangeMode::kColumn));
  channel_arrange_combo_->addItem(tr("Grid"),
                                  static_cast<int>(VertexArrangeMode::kGrid));
  channel_arrange_combo_->setCurrentIndex(1);
  channel_arrange_combo_->setToolTip(
      tr("Channel arrange mode: single column or multi-column grid"));
  toolbar_layout->addWidget(channel_arrange_combo_);

  toolbar_layout->addWidget(new QLabel(tr("Svc"), toolbar));
  service_arrange_combo_ = new QComboBox(toolbar);
  service_arrange_combo_->addItem(tr("Column"),
                                  static_cast<int>(VertexArrangeMode::kColumn));
  service_arrange_combo_->addItem(tr("Grid"),
                                  static_cast<int>(VertexArrangeMode::kGrid));
  service_arrange_combo_->setCurrentIndex(1);
  service_arrange_combo_->setToolTip(
      tr("Service arrange mode: single column or multi-column grid"));
  toolbar_layout->addWidget(service_arrange_combo_);

  toolbar_layout->addWidget(new QLabel(tr("Group"), toolbar));
  prefix_combo_ = new QComboBox(toolbar);
  prefix_combo_->setMinimumWidth(120);
  prefix_combo_->setToolTip(tr("Filter channels by namespace prefix"));
  toolbar_layout->addWidget(prefix_combo_);

  filter_edit_ = new QLineEdit(toolbar);
  filter_edit_->setPlaceholderText(tr("Filter nodes, channels, services…"));
  filter_edit_->setClearButtonEnabled(true);
  filter_edit_->setMinimumWidth(160);
  StyleFilterLineEdit(filter_edit_);
  toolbar_layout->addWidget(filter_edit_, 1);

  auto_refresh_check_ = new QCheckBox(tr("Auto"), toolbar);
  auto_refresh_check_->setChecked(true);
  auto_refresh_check_->setToolTip(tr("Automatically refresh topology"));
  toolbar_layout->addWidget(auto_refresh_check_);

  root->addWidget(toolbar);

  auto* legend = new QLabel(
      tr("● Node  ▬ Channel (right: Hz)  ⬡ Service. "
         "Click to focus (1-hop hides others). Double-click to open properties. "
         "Edge labels appear on focus/hover unless Labels is on."),
      this);
  legend->setWordWrap(true);
  legend->setContentsMargins(PanelChromeLayout::kToolbarMarginH, 4,
                             PanelChromeLayout::kToolbarMarginH, 4);
  StyleHintLabel(legend);
  root->addWidget(legend);

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
  connect(show_services_check_, &QCheckBox::toggled, this,
          &ChannelGraphPanel::onShowServicesToggled);
  connect(show_channels_check_, &QCheckBox::toggled, this,
          &ChannelGraphPanel::onShowChannelsToggled);
  connect(neighborhood_check_, &QCheckBox::toggled, this,
          &ChannelGraphPanel::onNeighborhoodToggled);
  connect(edge_labels_check_, &QCheckBox::toggled, this,
          &ChannelGraphPanel::onShowEdgeLabelsToggled);
  connect(channel_arrange_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
          this, &ChannelGraphPanel::onChannelArrangeChanged);
  connect(service_arrange_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
          this, &ChannelGraphPanel::onServiceArrangeChanged);
  connect(auto_refresh_check_, &QCheckBox::toggled, this,
          &ChannelGraphPanel::onAutoRefreshToggled);
  connect(prefix_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          &ChannelGraphPanel::onPrefixChanged);
  connect(filter_edit_, &QLineEdit::textChanged, this,
          &ChannelGraphPanel::onFilterChanged);
  connect(filter_timer_, &QTimer::timeout, this, &ChannelGraphPanel::refreshGraph);
  connect(refresh_timer_, &QTimer::timeout, this, &ChannelGraphPanel::refreshGraph);
  connect(graph_view_, &ChannelGraphView::graphRendered, this,
          &ChannelGraphPanel::onGraphRendered);
  connect(graph_view_, &ChannelGraphView::vertexDoubleClicked, this,
          &ChannelGraphPanel::onVertexDoubleClicked);
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
      CreateRvizPanelTitleBarTools(dock, callbacks, options);
  expand_button_ = tools.expand_button;
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
  refreshGraph();
}

void ChannelGraphPanel::cloneConfigFrom(const ChannelGraphPanelConfig& config) {
  setConfig(config);
}

void ChannelGraphPanel::refreshGraph() {
  integration::TopologyGraphBuildOptions options;
  options.show_services = config_.show_services;
  options.show_channels = config_.show_channels;
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
  if (graph.vertices.empty()) {
    updateStatusText(tr("No matching topology"));
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

void ChannelGraphPanel::onPrefixChanged(int index) {
  if (index < 0) {
    return;
  }
  config_.prefix_filter = prefix_combo_->itemData(index).toString();
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

void ChannelGraphPanel::onChannelArrangeChanged(int index) {
  if (index < 0 || channel_arrange_combo_ == nullptr) {
    return;
  }
  config_.channel_arrange = static_cast<VertexArrangeMode>(
      channel_arrange_combo_->itemData(index).toInt());
  emit configChanged();
  last_topology_hash_.clear();
  graph_view_->resetSavedPositions();
  refreshGraph();
  graph_view_->zoomToFit();
}

void ChannelGraphPanel::onServiceArrangeChanged(int index) {
  if (index < 0 || service_arrange_combo_ == nullptr) {
    return;
  }
  config_.service_arrange = static_cast<VertexArrangeMode>(
      service_arrange_combo_->itemData(index).toInt());
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
  updateStatusText(tr("%1 vertices, %2 connections").arg(vertex_count).arg(edge_count));
}

void ChannelGraphPanel::onVertexDoubleClicked(
    const QString& vertex_id, integration::GraphVertexKind kind,
    const QString& label, const QString& detail) {
  Q_UNUSED(vertex_id);
  showVertexProperties(kind, label, detail);
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

void ChannelGraphPanel::applyConfigToUi() {
  show_services_check_->setChecked(config_.show_services);
  show_channels_check_->setChecked(config_.show_channels);
  if (neighborhood_check_ != nullptr) {
    neighborhood_check_->setChecked(config_.neighborhood_mode);
  }
  if (edge_labels_check_ != nullptr) {
    edge_labels_check_->setChecked(config_.show_edge_labels);
  }
  auto_refresh_check_->setChecked(config_.auto_refresh);
  filter_edit_->setText(config_.filter);

  auto set_arrange_combo = [](QComboBox* combo, VertexArrangeMode mode) {
    if (combo == nullptr) {
      return;
    }
    combo->blockSignals(true);
    const int index = combo->findData(static_cast<int>(mode));
    combo->setCurrentIndex(index >= 0 ? index : 0);
    combo->blockSignals(false);
  };
  set_arrange_combo(channel_arrange_combo_, config_.channel_arrange);
  set_arrange_combo(service_arrange_combo_, config_.service_arrange);
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

void ChannelGraphPanel::rebuildPrefixCombo() {
  const QString current = config_.prefix_filter;
  prefix_combo_->blockSignals(true);
  prefix_combo_->clear();
  prefix_combo_->addItem(tr("All"), QString());
  for (const std::string& prefix : integration::ListChannelPrefixGroups()) {
    const QString value = QString::fromStdString(prefix);
    prefix_combo_->addItem(value, value);
  }
  const int index = prefix_combo_->findData(current);
  prefix_combo_->setCurrentIndex(index >= 0 ? index : 0);
  prefix_combo_->blockSignals(false);
  if (index < 0) {
    config_.prefix_filter.clear();
  }
}

void ChannelGraphPanel::updateStatusText(const QString& text) {
  status_label_->setText(text);
}

}  // namespace channel_graph
}  // namespace autoviz
