/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QDockWidget>
#include <QLabel>

class QToolButton;
class QMainWindow;

namespace autoviz {

/** RViz-style dock panel with custom title bar and collapsed state. */
class PanelDockWidget : public QDockWidget {
  Q_OBJECT

 public:
  explicit PanelDockWidget(const QString& name, QWidget* parent = nullptr);

  void setContentWidget(QWidget* child);
  void setCollapsed(bool collapsed);
  bool isCollapsed() const { return collapsed_; }
  void setPanelIcon(const QIcon& icon);

  /** Lock dock height to title bar + fixed content (e.g. bottom Time bar). */
  void setFixedContentHeight(int height);
  void enforceFixedHeight();

  void overrideVisibility(bool hidden);

 protected:
  void closeEvent(QCloseEvent* event) override;
  void setVisible(bool visible) override;
  void showEvent(QShowEvent* event) override;
  void resizeEvent(QResizeEvent* event) override;
  bool eventFilter(QObject* watched, QEvent* event) override;

 private slots:
  void onChildDestroyed(QObject* child);

 signals:
  void closed();

 private:
  void applyFixedContentHeight();
  void installFixedHeightHooks();
  int fixedDockHeight() const;
  QMainWindow* mainWindow() const;

  bool collapsed_ = false;
  bool requested_visibility_ = true;
  bool forced_hidden_ = false;
  int fixed_content_height_ = 0;
  bool fixed_height_hooks_installed_ = false;
  bool height_enforcing_ = false;
  QWidget* title_bar_ = nullptr;
  QLabel* icon_label_ = nullptr;
  QLabel* title_label_ = nullptr;
};

}  // namespace autoviz
