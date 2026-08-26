/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QHash>
#include <QWidget>

#include <QStringList>

#include "autoviz/integration/channel_reader_registry.hpp"
#include "autoviz/ui/publish/publish_types.hpp"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QGroupBox;
class QLabel;
class QPlainTextEdit;
class QPushButton;
class QSplitter;
class QTabWidget;
class QTimer;

namespace autoviz {
namespace common {
class VisualizationManager;
}

namespace publish_panel {

class PublishFieldTreeWidget;

class PublishEditorWidget : public QWidget {
  Q_OBJECT

 public:
  explicit PublishEditorWidget(common::VisualizationManager* manager,
                               QWidget* parent = nullptr);

  PublishPanelConfig config() const;
  void setConfig(const PublishPanelConfig& config);
  void refreshChannels();

 signals:
  void configChanged();
  void publishRequested();

 protected:
  void keyPressEvent(QKeyEvent* event) override;

 private slots:
  void onEditingModeToggled(bool enabled);
  void onChannelChanged(const QString& text);
  void onMessageTypeChanged(const QString& text);
  void onResetTemplate();
  void onFillFromLatest();
  void onRefreshTopics();
  void onPublishOnceClicked();
  void onAddPublisher();
  void onRemovePublisher();
  void onPublisherEdited(int index);
  void onPublisherPublishingChanged(int index, bool publishing);
  void onPublisherRateChanged(int index, double rate_hz);
  void onFieldEdited();
  void onPresetSelected(int index);
  void onSavePreset();
  void onDeletePreset();
  void onPublishRateChanged(double rate);
  void onMessageTabChanged(int index);
  void onFieldsEdited();

 private:
  QPlainTextEdit* makeJsonEditor(QWidget* parent, const QString& placeholder,
                                 bool read_only = false);
  void rebuildMessageTypeList();
  void rebuildPresetCombo();
  void rebuildPublishersTree();
  void applyEditingModeUi();
  void updatePublishButtonState();
  void showResult(bool success, const QString& summary, const QString& details);
  bool publishDraft(bool from_loop);
  bool publishEntry(int index, bool from_loop);
  void syncFieldsFromJson();
  void syncJsonFromFields();
  void updateMetadataPanel();
  void captureDraftFromUi();
  int selectedPublisherRow() const;
  bool selectedPublisherCanPublish() const;
  int findPublisherIndexById(const QString& id) const;
  int findPublisherIndexByChannel(const QString& channel) const;
  int resolveActivePublisherRow() const;
  void startPublisherTimer(const PublishEntry& entry);
  void stopPublisherTimer(const QString& entry_id);
  void stopAllPublisherTimers();
  void restorePublisherTimers();
  QString resolvedMessageType() const;
  QString messageTypeForChannel(const QString& channel) const;
  void rememberCustomChannel(const QString& channel);
  void setMessageTypeField(const QString& message_type);
  void fillMessageJson(const QString& json, bool user_edited);
  void maybeFillTemplateForType(const QString& message_type);
  void requestAutoFillMessage(const QString& channel, const QString& message_type);
  void cancelLatestMessageFill();
  void applyPreset(const PublishPreset& preset);
  void ensureDraftExpression();
  bool prepareDraftPublisher(PublishEntry* entry, QString* error);
  void enterDraftMode();
  void loadPublisherAt(int index);
  void syncConfigFromPublishersTree();
  void syncPublisherJsonFromTree(int index);
  void emitConfigChanged();
  void applyChromeStyles();
  void refreshPublishButtonAppearance();

  common::VisualizationManager* manager_ = nullptr;
  PublishPanelConfig config_;
  bool suppress_template_update_ = false;
  bool suppress_tree_update_ = false;
  bool suppress_draft_sync_ = false;
  bool message_user_edited_ = false;
  bool fields_dirty_ = false;
  QStringList custom_channels_;
  QHash<QString, QTimer*> publisher_timers_;
  integration::ChannelReaderRegistry::SubscriptionId latest_fill_subscription_ = 0;
  QString latest_fill_channel_;

  QCheckBox* editing_mode_check_ = nullptr;
  QWidget* editor_body_ = nullptr;
  QWidget* collection_bar_ = nullptr;
  QWidget* rqt_top_bar_ = nullptr;
  QWidget* expression_toolbar_ = nullptr;
  QComboBox* preset_combo_ = nullptr;
  QPushButton* save_preset_button_ = nullptr;
  QPushButton* delete_preset_button_ = nullptr;
  QComboBox* channel_combo_ = nullptr;
  QComboBox* message_type_combo_ = nullptr;
  QDoubleSpinBox* publish_rate_spin_ = nullptr;
  QPushButton* add_publisher_button_ = nullptr;
  QPushButton* remove_publisher_button_ = nullptr;
  QPushButton* refresh_topics_button_ = nullptr;
  QPushButton* publish_once_button_ = nullptr;
  PublishFieldTreeWidget* publishers_tree_ = nullptr;
  QPushButton* reset_template_button_ = nullptr;
  QPushButton* fill_latest_button_ = nullptr;
  QTabWidget* message_tabs_ = nullptr;
  QSplitter* payload_splitter_ = nullptr;
  QGroupBox* request_group_ = nullptr;
  QGroupBox* result_group_ = nullptr;
  QLabel* result_status_label_ = nullptr;
  QPlainTextEdit* message_edit_ = nullptr;
  PublishFieldTreeWidget* field_tree_ = nullptr;
  QPlainTextEdit* metadata_edit_ = nullptr;
  QPlainTextEdit* result_edit_ = nullptr;
  QTimer* latest_fill_timer_ = nullptr;
};

}  // namespace publish_panel
}  // namespace autoviz
