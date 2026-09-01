/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/add_display_dialog.hpp"

#include <map>
#include <vector>

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTabWidget>
#include <QTextBrowser>
#include <QTreeWidget>
#include <QTreeWidgetItemIterator>
#include <QVBoxLayout>

#include "autoviz/common/display_catalog.hpp"
#include "autoviz/common/display_factory.hpp"
#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/integration/channel_manager.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {
namespace {

constexpr int kTypeRole = Qt::UserRole;
constexpr int kChannelRole = Qt::UserRole + 1;
constexpr int kTopicPathRole = Qt::UserRole + 2;
constexpr int kFilterRole = Qt::UserRole + 3;

AddDisplaySelection SelectionFromDisplayItem(QTreeWidgetItem* item) {
  AddDisplaySelection selection;
  if (item == nullptr || item->parent() == nullptr) {
    return selection;
  }
  selection.type = item->data(0, kTypeRole).toString();
  selection.display_name = item->text(0);
  selection.description_html =
      item->toolTip(0).toHtmlEscaped().replace(QLatin1Char('\n'),
                                               QStringLiteral("<br>"));
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
  selection.description_html =
      item->toolTip(0).toHtmlEscaped().replace(QLatin1Char('\n'),
                                               QStringLiteral("<br>"));
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
      IconLoader::load(QStringLiteral(":/autoviz/icons/default_package_icon.svg"));
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
        common::DisplayCatalog::typesForMessageType(channel.message_type);
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

void ApplyTopicFilter(QTreeWidget* tree, QLineEdit* filter_box,
                      QCheckBox* enable_hidden_box) {
  const QString filter = filter_box->text().trimmed();
  const bool hide_unvisualizable =
      enable_hidden_box->checkState() == Qt::Unchecked;

  std::vector<QTreeWidgetItem*> items;
  for (QTreeWidgetItemIterator it(tree); *it; ++it) {
    QTreeWidgetItem* item = *it;
    items.push_back(item);

    QString topic_path;
    if (item->data(0, kChannelRole).isValid()) {
      topic_path = item->data(0, kChannelRole).toString();
    } else {
      QTreeWidgetItem* parent = item->parent();
      while (parent != nullptr) {
        if (parent->data(0, kChannelRole).isValid()) {
          topic_path = parent->data(0, kChannelRole).toString();
          break;
        }
        parent = parent->parent();
      }
      if (topic_path.isEmpty()) {
        topic_path = item->text(0);
      }
    }
    item->setData(0, kTopicPathRole, topic_path);
    item->setData(0, kFilterRole,
                  filter.isEmpty() ||
                      topic_path.contains(filter, Qt::CaseInsensitive));
  }

  for (auto it = items.rbegin(); it != items.rend(); ++it) {
    QTreeWidgetItem* item = *it;
    bool matches_filter = item->data(0, kFilterRole).toBool();
    for (int i = 0; i < item->childCount(); ++i) {
      if (item->child(i)->data(0, kFilterRole).toBool()) {
        matches_filter = true;
        break;
      }
    }
    item->setData(0, kFilterRole, matches_filter);
  }

  for (QTreeWidgetItem* item : items) {
    const bool matches_filter = item->data(0, kFilterRole).toBool();
    const bool hide_disabled = hide_unvisualizable && item->isDisabled();
    item->setHidden(!matches_filter || hide_disabled);
  }
}

}  // namespace

AddDisplayDialog::AddDisplayDialog(
    std::shared_ptr<common::VisualizationManager> manager,
    const QStringList& disallowed_display_names, QWidget* parent)
    : QDialog(parent),
      manager_(std::move(manager)),
      disallowed_display_names_(disallowed_display_names) {
  setObjectName(QStringLiteral("AddDisplayDialog"));

  auto* type_box = new QGroupBox(tr("Create visualization"), this);
  type_box->setObjectName(QStringLiteral("AddDisplayDialog/Visualization_Typebox"));

  auto* display_tree = new QTreeWidget(type_box);
  display_tree->setObjectName(QStringLiteral("AddDisplayDialog/DisplayTypeTree"));
  display_tree->setHeaderHidden(true);
  display_tree->setIconSize(QSize(28, 28));
  display_tree->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  PopulateDisplayTypeTree(display_tree);

  auto* topic_widget = new QWidget(type_box);
  topic_tree_ = new QTreeWidget(topic_widget);
  topic_tree_->setObjectName(QStringLiteral("AddDisplayDialog/TopicTree"));
  topic_tree_->setHeaderHidden(true);
  topic_tree_->setIconSize(QSize(28, 28));
  topic_tree_->header()->setStretchLastSection(true);
  topic_tree_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  show_unvisualizable_topics_ =
      new QCheckBox(tr("Show unvisualizable topics"), topic_widget);
  topic_filter_box_ = new QLineEdit(topic_widget);
  auto* filter_body = new QWidget(topic_widget);
  auto* filter_body_layout = new QVBoxLayout(filter_body);
  filter_body_layout->setContentsMargins(0, 0, 0, 0);
  filter_body_layout->setSpacing(4);
  filter_body_layout->addWidget(
      new QLabel(tr("Filter topics by name:"), filter_body));
  filter_body_layout->addWidget(topic_filter_box_);
  auto* filter_section = MakeCollapsibleSection(
      topic_widget, tr("Filter topics by name"), filter_body, false);
  auto* topic_layout = new QVBoxLayout(topic_widget);
  topic_layout->setContentsMargins(0, 0, 0, 0);
  topic_layout->addWidget(topic_tree_, 1);
  topic_layout->addWidget(show_unvisualizable_topics_);
  topic_layout->addWidget(filter_section);
  refreshTopicTree();

  tab_widget_ = new QTabWidget(type_box);
  tab_widget_->setObjectName(QStringLiteral("Visualization_Typebox/TabWidget"));
  tab_widget_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  display_tab_ = tab_widget_->addTab(display_tree, tr("By display type"));
  topic_tab_ = tab_widget_->addTab(topic_widget, tr("By channel"));

  description_ = new QTextBrowser(type_box);
  description_->setMaximumHeight(100);
  description_->setOpenExternalLinks(true);
  auto* description_body = new QWidget(type_box);
  auto* description_body_layout = new QVBoxLayout(description_body);
  description_body_layout->setContentsMargins(0, 0, 0, 0);
  description_body_layout->addWidget(description_);
  auto* description_section =
      MakeCollapsibleSection(type_box, tr("Description"), description_body, false);

  auto* type_layout = new QVBoxLayout(type_box);
  type_layout->addWidget(tab_widget_, 1);
  type_layout->addWidget(description_section);

  auto* name_box = new QGroupBox(tr("Display Name"), this);
  name_box->setObjectName(QStringLiteral("AddDisplayDialog/DisplayNameBox"));
  name_editor_ = new QLineEdit(name_box);
  auto* name_layout = new QVBoxLayout(name_box);
  name_layout->addWidget(name_editor_);

  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                     Qt::Horizontal, this);
  button_box_->setObjectName(QStringLiteral("AddDisplayDialog/ButtonBox"));

  auto* main_layout = new QVBoxLayout(this);
  main_layout->addWidget(type_box);
  main_layout->addWidget(name_box);
  main_layout->addWidget(button_box_);
  setLayout(main_layout);

  connect(display_tree, &QTreeWidget::currentItemChanged, this,
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
  connect(display_tree, &QTreeWidget::itemActivated, this, &AddDisplayDialog::accept);
  connect(topic_tree_, &QTreeWidget::itemActivated, this, &AddDisplayDialog::accept);
  connect(show_unvisualizable_topics_, &QCheckBox::stateChanged, this,
          [this](int) {
            ApplyTopicFilter(topic_tree_, topic_filter_box_,
                             show_unvisualizable_topics_);
          });
  connect(topic_filter_box_, &QLineEdit::textChanged, this,
          [this](const QString&) {
            ApplyTopicFilter(topic_tree_, topic_filter_box_,
                             show_unvisualizable_topics_);
          });
  connect(button_box_, &QDialogButtonBox::accepted, this, &AddDisplayDialog::accept);
  connect(button_box_, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(tab_widget_, &QTabWidget::currentChanged, this,
          &AddDisplayDialog::onTabChanged);
  connect(name_editor_, &QLineEdit::textEdited, this, [this]() {
    user_edited_name_ = true;
    onNameChanged();
  });

  button_box_->button(QDialogButtonBox::Ok)->setEnabled(false);
}

QSize AddDisplayDialog::sizeHint() const { return {500, 660}; }

void AddDisplayDialog::refreshTopicTree() {
  if (manager_ == nullptr || topic_tree_ == nullptr ||
      topic_filter_box_ == nullptr || show_unvisualizable_topics_ == nullptr) {
    return;
  }
  manager_->refreshChannelList();
  PopulateTopicTree(topic_tree_, *manager_);
  ApplyTopicFilter(topic_tree_, topic_filter_box_, show_unvisualizable_topics_);
}

void AddDisplayDialog::onTabChanged(int index) {
  if (index == topic_tab_) {
    refreshTopicTree();
  }
  updateUi();
}

void AddDisplayDialog::onNameChanged() { updateUi(); }

void AddDisplayDialog::updateUi() {
  if (tab_widget_->currentIndex() == topic_tab_) {
    selection_ = topic_tab_selection_;
  } else {
    selection_ = display_tab_selection_;
  }

  const QString selection_key = selection_.type + QLatin1Char('|') + selection_.channel;
  if (selection_key != last_selection_key_) {
    last_selection_key_ = selection_key;
    user_edited_name_ = false;
  }

  if (selection_.description_html.isEmpty()) {
    description_->clear();
  } else {
    description_->setHtml(QStringLiteral("<html><body>%1</body></html>")
                            .arg(selection_.description_html));
  }

  if (!user_edited_name_ && !selection_.type.isEmpty()) {
    name_editor_->setText(SuggestDisplayName(
        selection_.type, selection_.channel, disallowed_display_names_));
  } else if (name_editor_->text().isEmpty() && !selection_.display_name.isEmpty()) {
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
