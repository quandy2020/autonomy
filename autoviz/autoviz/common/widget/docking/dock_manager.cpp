/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *****************************************************************************/

#include "autoviz/common/widget/docking/dock_manager.hpp"

#include <QDockWidget>
#include <QMainWindow>
#include <QVariant>
#include <QWidget>

namespace autoviz::widget {
namespace {

Qt::DockWidgetArea ToQtDockArea(DockArea area) {
  switch (area) {
    case DockArea::kLeft:
      return Qt::LeftDockWidgetArea;
    case DockArea::kRight:
      return Qt::RightDockWidgetArea;
    case DockArea::kTop:
      return Qt::TopDockWidgetArea;
    case DockArea::kBottom:
      return Qt::BottomDockWidgetArea;
  }
  return Qt::LeftDockWidgetArea;
}

}  // namespace

DockManager::DockManager(QMainWindow* host) : host_(host) {}

QDockWidget* DockManager::AddPanel(const QString& id, const QString& title,
                                   QWidget* content, DockArea area,
                                   const DockPanelOptions& options) {
  if (host_ == nullptr || content == nullptr || id.isEmpty()) {
    return nullptr;
  }

  const auto existing = panels_.find(id);
  if (existing != panels_.end()) {
    if (!existing.value().isNull()) {
      return nullptr;
    }
    panels_.erase(existing);
  }

  auto* dock = new QDockWidget(title, host_);
  dock->setObjectName(id);
  dock->setProperty("autovizRole", QStringLiteral("dockPanel"));
  dock->setWidget(content);

  QDockWidget::DockWidgetFeatures features;
  if (options.closable) {
    features |= QDockWidget::DockWidgetClosable;
  }
  if (options.movable) {
    features |= QDockWidget::DockWidgetMovable;
  }
  if (options.floatable) {
    features |= QDockWidget::DockWidgetFloatable;
  }
  dock->setFeatures(features);

  host_->addDockWidget(ToQtDockArea(area), dock);
  panels_.insert(id, dock);
  return dock;
}

bool DockManager::RemovePanel(const QString& id) {
  const auto it = panels_.find(id);
  if (it == panels_.end()) {
    return false;
  }
  QDockWidget* dock = it.value();
  panels_.erase(it);
  if (dock != nullptr) {
    if (host_ != nullptr) {
      host_->removeDockWidget(dock);
    }
    dock->deleteLater();
  }
  return true;
}

QDockWidget* DockManager::Panel(const QString& id) const {
  return panels_.value(id).data();
}

void DockManager::SetPanelVisible(const QString& id, bool visible) {
  if (QDockWidget* dock = Panel(id); dock != nullptr) {
    dock->setVisible(visible);
  }
}

QByteArray DockManager::SaveLayout(int version) const {
  return host_ == nullptr ? QByteArray() : host_->saveState(version);
}

bool DockManager::RestoreLayout(const QByteArray& state, int version) {
  return host_ != nullptr && !state.isEmpty() &&
         host_->restoreState(state, version);
}

}  // namespace autoviz::widget
