/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QLayout>
#include <QList>
#include <QStyle>

class QLayoutItem;
class QRect;
class QSize;
class QWidget;

namespace autoviz {
namespace widget {

/**
 * @brief A compact wrapping layout for chips, badges, and tool actions.
 *
 * FlowLayout owns every QLayoutItem added to it and follows the same ownership
 * rules as Qt's built-in layouts. Spacing defaults are taken from the active
 * QStyle, so the layout automatically tracks the application theme.
 */
class FlowLayout final : public QLayout {
 public:
  explicit FlowLayout(QWidget* parent = nullptr, int margin = -1,
                      int horizontal_spacing = -1, int vertical_spacing = -1);
  explicit FlowLayout(int margin, int horizontal_spacing = -1,
                      int vertical_spacing = -1);
  ~FlowLayout() override;

  void addItem(QLayoutItem* item) override;
  Qt::Orientations expandingDirections() const override;
  bool hasHeightForWidth() const override;
  int heightForWidth(int width) const override;
  int count() const override;
  QLayoutItem* itemAt(int index) const override;
  QSize minimumSize() const override;
  void setGeometry(const QRect& rect) override;
  QSize sizeHint() const override;
  QLayoutItem* takeAt(int index) override;

  int horizontalSpacing() const;
  int verticalSpacing() const;

 private:
  int DoLayout(const QRect& rect, bool test_only) const;
  int SmartSpacing(QStyle::PixelMetric metric) const;

  QList<QLayoutItem*> items_;
  int horizontal_spacing_ = -1;
  int vertical_spacing_ = -1;
};

}  // namespace widget
}  // namespace autoviz
