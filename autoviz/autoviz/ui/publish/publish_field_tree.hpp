/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QTreeWidget>

#include <memory>
#include <string>
#include <vector>

#include "autoviz/ui/publish/publish_types.hpp"

class QMenu;

namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

namespace autoviz {
namespace publish_panel {

/** rqt Message Publisher tree (channel | type | rate | expression) or Fields editor. */
class PublishFieldTreeWidget : public QTreeWidget {
  Q_OBJECT

 public:
  enum class DisplayMode { kFieldsEditor, kRqtPublishers };

  explicit PublishFieldTreeWidget(QWidget* parent = nullptr);

  void setDisplayMode(DisplayMode mode);
  void setReadOnly(bool read_only);
  void setValueColumnTitle(const QString& title);

  /** Single-message editor mode (Name | Type | Value). */
  bool loadFromJson(const std::string& message_type, const QString& json);
  bool loadTemplate(const std::string& message_type);
  QString toJson() const;
  bool hasMessage() const;
  void clearMessage();

  /** rqt multi-publisher mode (channel | type | rate | expression). */
  void setPublishers(const QVector<PublishEntry>& entries);
  QVector<PublishEntry> publishers() const;
  int selectedPublisherIndex() const;
  QString selectedPublisherId() const;
  void selectPublisher(int index);
  bool isPublisherRootSelected(int index) const;
  void setPublisherPublishing(int index, bool publishing);
  QString publisherJsonAt(int index) const;
  int editorValueColumn() const;

 public slots:
  void expandAllFields();
  void collapseAllFields();
  void addArrayElement();
  void removeArrayElement();

 signals:
  void messageEdited();
  void publisherEdited(int index);
  void publisherPublishingChanged(int index, bool publishing);
  void publisherRateChanged(int index, double rate_hz);
  void publisherSelectionChanged(int index);

 private slots:
  void onItemChanged(QTreeWidgetItem* item, int column);
  void onCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);
  void showContextMenu(const QPoint& pos);
  void addRepeatedElement();
  void removeRepeatedElement();
  void removeSelectedElement();
  void duplicateSelectedElement();

 private:
  void rebuildTree();
  void rebuildPublishersTree();
  void rebuildSingleMessageTree();
  google::protobuf::Message* messageForItem(QTreeWidgetItem* item) const;
  int publisherIndexForItem(QTreeWidgetItem* item) const;
  QTreeWidgetItem* publisherRootItem(int index) const;
  QTreeWidgetItem* findPublisherRootItem(int index) const;
  void syncPublisherJson(int index);
  int expressionColumn() const;

  QTreeWidgetItem* arrayItemAt(const QPoint& pos) const;
  QTreeWidgetItem* arrayElementItemAt(const QPoint& pos) const;
  bool addRepeatedElementAtPath(const QString& array_path,
                                google::protobuf::Message* message);
  bool removeRepeatedElementAtPath(const QString& element_path,
                                   google::protobuf::Message* message);
  bool duplicateRepeatedElementAtPath(const QString& element_path,
                                      google::protobuf::Message* message);
  void rebuildPublisherSubtree(int index);
  void applyReadOnlyFlags(QTreeWidgetItem* item);
  void applyReadOnlyFlags();

  DisplayMode display_mode_ = DisplayMode::kFieldsEditor;
  bool read_only_ = false;
  std::unique_ptr<google::protobuf::Message> message_;
  std::string message_type_;
  QVector<PublishEntry> publisher_entries_;
  std::vector<std::unique_ptr<google::protobuf::Message>> publisher_messages_;
  bool suppress_updates_ = false;
  QTreeWidgetItem* context_array_item_ = nullptr;
  QTreeWidgetItem* context_element_item_ = nullptr;
  google::protobuf::Message* context_message_ = nullptr;
};

}  // namespace publish_panel
}  // namespace autoviz
