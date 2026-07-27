/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/tf_tree_panel.hpp"

#include <QRegularExpression>
#include <QTreeWidget>
#include <QVBoxLayout>

#include "autoviz/transform/buffer.hpp"

namespace autoviz {

TfTreePanel::TfTreePanel(autoviz::transform::Buffer* tf_buffer,
                         QWidget* parent)
    : QWidget(parent), tf_buffer_(tf_buffer) {
  setupUi();
  refresh();
}

void TfTreePanel::setupUi() {
  auto* layout = new QVBoxLayout(this);
  tree_ = new QTreeWidget(this);
  tree_->setHeaderLabels({tr("Frame"), tr("Parent")});
  layout->addWidget(tree_);
}

void TfTreePanel::refresh() {
  tree_->clear();
  if (tf_buffer_ == nullptr) {
    return;
  }
  const std::string frames_text = tf_buffer_->allFramesAsString();
  const QRegularExpression pattern(
      QStringLiteral(R"(Frame (\S+) exists with parent (\S+)\.)"));
  QRegularExpressionMatchIterator it = pattern.globalMatch(
      QString::fromStdString(frames_text));
  while (it.hasNext()) {
    const QRegularExpressionMatch match = it.next();
    auto* item = new QTreeWidgetItem(tree_);
    item->setText(0, match.captured(1));
    item->setText(1, match.captured(2));
  }
  tree_->sortItems(0, Qt::AscendingOrder);
}

}  // namespace autoviz
