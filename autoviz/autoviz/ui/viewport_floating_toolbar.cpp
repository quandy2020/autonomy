/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/viewport_floating_toolbar.hpp"

#include <QFont>
#include <QFrame>
#include <QHBoxLayout>
#include <QRegion>
#include <QResizeEvent>
#include <QShowEvent>
#include <QSizePolicy>
#include <QToolButton>
#include <QVBoxLayout>

#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/panel_settings_styles.hpp"

namespace autoviz {

namespace {

QFrame* MakeToolGroup(QWidget* parent) {
  auto* group = new QFrame(parent);
  group->setObjectName(QStringLiteral("AutovizViewportToolGroup"));
  group->setStyleSheet(QStringLiteral(
      "QFrame#AutovizViewportToolGroup {"
      "  background: palette(base);"
      "  border: 1px solid palette(mid);"
      "  border-radius: 8px;"
      "}"
      "QFrame#AutovizViewportToolGroup QToolButton {"
      "  border: none;"
      "  margin: 0;"
      "  padding: 4px;"
      "  background: transparent;"
      "  border-radius: 6px;"
      "}"
      "QFrame#AutovizViewportToolGroup QToolButton:hover {"
      "  background: palette(midlight);"
      "}"
      "QFrame#AutovizViewportToolGroup QToolButton:checked {"
      "  background: palette(highlight);"
      "  color: palette(highlighted-text);"
      "}"));
  return group;
}

}  // namespace

ViewportFloatingToolbar::ViewportFloatingToolbar(QWidget* parent)
    : QWidget(parent) {
  setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  setAttribute(Qt::WA_TransparentForMouseEvents, false);
  // Sibling of QOpenGLWidget / native Ogre view: without this the GL surface
  // paints over the toolbar (common on macOS Retina).
  setAttribute(Qt::WA_AlwaysStackOnTop, true);
  auto* outer = new QVBoxLayout(this);
  outer->setContentsMargins(0, 0, 0, 0);
  outer->setSpacing(8);

  {
    auto* group = MakeToolGroup(this);
    auto* layout = new QVBoxLayout(group);
    layout->setContentsMargins(2, 2, 2, 2);
    layout->setSpacing(0);
    inspect_button_ = MakeToolButton(
        IconLoader::panelTitleIcon(QStringLiteral("viewport.inspect")),
        tr("Inspect Object"), true);
    layout->addWidget(inspect_button_);
    connect(inspect_button_, &QToolButton::clicked, this, [this]() {
      if (callbacks_.on_inspect) {
        callbacks_.on_inspect();
      }
    });
    outer->addWidget(group);
  }

  {
    auto* group = MakeToolGroup(this);
    auto* layout = new QVBoxLayout(group);
    layout->setContentsMargins(2, 2, 2, 2);
    layout->setSpacing(0);
    camera_2d_button_ = MakeTextToolButton(
        QStringLiteral("3D"), tr("Switch To 2D Camera"), true);
    measure_button_ = MakeToolButton(
        IconLoader::panelTitleIcon(QStringLiteral("viewport.measure")),
        tr("Measure Distance"), true);
    layout->addWidget(camera_2d_button_);
    layout->addWidget(measure_button_);
    connect(camera_2d_button_, &QToolButton::clicked, this, [this]() {
      if (callbacks_.on_toggle_2d_camera) {
        callbacks_.on_toggle_2d_camera();
      }
    });
    connect(measure_button_, &QToolButton::clicked, this, [this]() {
      if (callbacks_.on_measure) {
        callbacks_.on_measure();
      }
    });
    outer->addWidget(group);
  }

  {
    auto* group = MakeToolGroup(this);
    auto* layout = new QVBoxLayout(group);
    layout->setContentsMargins(2, 2, 2, 2);
    layout->setSpacing(0);
    recenter_button_ = MakeToolButton(
        IconLoader::panelTitleIcon(QStringLiteral("viewport.recenter_frame")),
        tr("Re-center On frame"));
    layout->addWidget(recenter_button_);
    connect(recenter_button_, &QToolButton::clicked, this, [this]() {
      if (callbacks_.on_recenter_frame) {
        callbacks_.on_recenter_frame();
      }
    });
    outer->addWidget(group);
  }

  adjustSize();
  updateClickThroughMask();
}

void ViewportFloatingToolbar::showEvent(QShowEvent* event) {
  QWidget::showEvent(event);
  adjustSize();
  updateClickThroughMask();
}

void ViewportFloatingToolbar::resizeEvent(QResizeEvent* event) {
  QWidget::resizeEvent(event);
  updateClickThroughMask();
}

void ViewportFloatingToolbar::updateClickThroughMask() {
  QRegion region;
  const QList<QFrame*> groups = findChildren<QFrame*>(
      QStringLiteral("AutovizViewportToolGroup"));
  for (QFrame* group : groups) {
    if (group != nullptr && group->isVisible()) {
      region += group->geometry();
    }
  }
  if (region.isEmpty()) {
    clearMask();
  } else {
    setMask(region);
  }
}

void ViewportFloatingToolbar::setCallbacks(
    ViewportFloatingToolbarCallbacks callbacks) {
  callbacks_ = std::move(callbacks);
}

void ViewportFloatingToolbar::setInspectChecked(bool checked) {
  if (inspect_button_ != nullptr) {
    inspect_button_->blockSignals(true);
    inspect_button_->setChecked(checked);
    inspect_button_->blockSignals(false);
  }
}

void ViewportFloatingToolbar::setMeasureChecked(bool checked) {
  if (measure_button_ != nullptr) {
    measure_button_->blockSignals(true);
    measure_button_->setChecked(checked);
    measure_button_->blockSignals(false);
  }
}

void ViewportFloatingToolbar::set2dCameraChecked(bool checked) {
  if (camera_2d_button_ != nullptr) {
    camera_2d_button_->blockSignals(true);
    camera_2d_button_->setChecked(checked);
    camera_2d_button_->setText(checked ? QStringLiteral("2D")
                                       : QStringLiteral("3D"));
    camera_2d_button_->setToolTip(checked ? tr("Switch To 3D Camera")
                                          : tr("Switch To 2D Camera"));
    camera_2d_button_->blockSignals(false);
  }
}

void ViewportFloatingToolbar::setRecenterToolTip(const QString& tip) {
  if (recenter_button_ != nullptr) {
    recenter_button_->setToolTip(tip);
  }
}

QToolButton* ViewportFloatingToolbar::MakeToolButton(const QIcon& icon,
                                                     const QString& tip,
                                                     bool checkable) {
  auto* button = new QToolButton(this);
  button->setIcon(icon);
  button->setIconSize(QSize(18, 18));
  button->setAutoRaise(true);
  button->setToolTip(tip);
  button->setCheckable(checkable);
  button->setFixedSize(QSize(32, 32));
  return button;
}

QToolButton* ViewportFloatingToolbar::MakeTextToolButton(const QString& text,
                                                         const QString& tip,
                                                         bool checkable) {
  auto* button = new QToolButton(this);
  button->setText(text);
  button->setToolButtonStyle(Qt::ToolButtonTextOnly);
  QFont font = button->font();
  font.setBold(true);
  font.setPixelSize(11);
  button->setFont(font);
  button->setAutoRaise(true);
  button->setToolTip(tip);
  button->setCheckable(checkable);
  button->setFixedSize(QSize(32, 32));
  return button;
}

}  // namespace autoviz
