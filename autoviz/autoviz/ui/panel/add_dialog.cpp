/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel/add_dialog.hpp"
#include "autoviz/ui/panel/catalog.hpp"

#include <QAbstractItemView>
#include <QDialogButtonBox>
#include <QFont>
#include <QHBoxLayout>
#include <QListWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QPalette>
#include <QPushButton>
#include <QStyledItemDelegate>
#include <QStyleOptionViewItem>
#include <QVBoxLayout>

#include "autoviz/ui/theme/glass.hpp"
#include "autoviz/ui/app/icon_loader.hpp"
#include "autoviz/ui/theme/style.hpp"

namespace autoviz {
namespace {

constexpr int kCardInset = 12;
constexpr int kCardRadius = 18;

class GlassWell : public QWidget {
 public:
  explicit GlassWell(QWidget* parent = nullptr) : QWidget(parent) {
    setAttribute(Qt::WA_TranslucentBackground, true);
    setAutoFillBackground(false);
  }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    const QRectF card = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    QPainterPath path;
    path.addRoundedRect(card, 14.0, 14.0);
    painter.fillPath(path, QColor(255, 255, 255, 72));
    QLinearGradient sheen(card.topLeft(),
                          QPointF(card.left(), card.top() + 48.0));
    sheen.setColorAt(0.0, QColor(255, 255, 255, 110));
    sheen.setColorAt(1.0, QColor(255, 255, 255, 0));
    painter.fillPath(path, sheen);
    painter.setPen(QPen(QColor(255, 255, 255, 160), 1.0));
    painter.drawPath(path);
    painter.setPen(QPen(QColor(186, 230, 253, 90), 1.0));
    painter.drawRoundedRect(card.adjusted(1.0, 1.0, -1.0, -1.0), 13.0, 13.0);
  }
};

class ContinuousListDelegate : public QStyledItemDelegate {
 public:
  using QStyledItemDelegate::QStyledItemDelegate;

  void paint(QPainter* painter, const QStyleOptionViewItem& option,
             const QModelIndex& index) const override {
    QStyleOptionViewItem opt(option);
    initStyleOption(&opt, index);
    const bool selected = opt.state.testFlag(QStyle::State_Selected);
    const bool hovered = opt.state.testFlag(QStyle::State_MouseOver);

    QRect row = opt.rect;
    if (const auto* view = qobject_cast<const QAbstractItemView*>(opt.widget)) {
      row = QRect(4, opt.rect.y(), view->viewport()->width() - 8,
                  opt.rect.height());
    }
    if (selected || hovered) {
      painter->save();
      painter->setRenderHint(QPainter::Antialiasing, true);
      QPainterPath path;
      path.addRoundedRect(QRectF(row).adjusted(0.5, 1.0, -0.5, -1.0), 10.0, 10.0);
      painter->fillPath(path, selected ? QColor(20, 184, 166, 78)
                                       : QColor(165, 243, 252, 56));
      painter->restore();
    }

    opt.state.setFlag(QStyle::State_Selected, false);
    opt.state.setFlag(QStyle::State_MouseOver, false);
    opt.backgroundBrush = Qt::NoBrush;
    if (selected) {
      opt.palette.setColor(QPalette::Text, QColor(0x0F, 0x76, 0x6E));
      opt.palette.setColor(QPalette::WindowText, QColor(0x0F, 0x76, 0x6E));
      opt.font.setWeight(QFont::DemiBold);
    }
    QStyledItemDelegate::paint(painter, opt, index);
  }
};

}  // namespace

AddPanelDialog::AddPanelDialog(const QStringList& available_panels,
                               QWidget* parent)
    : QDialog(parent) {
  setObjectName(QStringLiteral("AddPanelDialog"));
  setWindowTitle(tr("Add Panel"));
  setModal(true);
  setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint);
  setAttribute(Qt::WA_TranslucentBackground, true);
  setAttribute(Qt::WA_StyledBackground, false);
  setAutoFillBackground(false);
  setMinimumSize(380, 480);
  resize(420, 560);

  auto* root = new QVBoxLayout(this);
  root->setContentsMargins(kCardInset + 6, kCardInset + 6, kCardInset + 6,
                           kCardInset + 6);
  root->setSpacing(12);

  auto* well = new GlassWell(this);
  auto* well_layout = new QVBoxLayout(well);
  well_layout->setContentsMargins(6, 6, 6, 6);
  well_layout->setSpacing(0);

  list_ = new QListWidget(well);
  list_->setIconSize(QSize(22, 22));
  list_->setSpacing(2);
  list_->setUniformItemSizes(true);
  list_->setAttribute(Qt::WA_MacShowFocusRect, false);
  list_->setAttribute(Qt::WA_TranslucentBackground, true);
  list_->setAutoFillBackground(false);
  if (list_->viewport() != nullptr) {
    list_->viewport()->setAttribute(Qt::WA_TranslucentBackground, true);
    list_->viewport()->setAutoFillBackground(false);
  }
  list_->setStyleSheet(
      style::sheet(QStringLiteral("add_panel")));
  QPalette pal = list_->palette();
  pal.setBrush(QPalette::Highlight, Qt::transparent);
  pal.setBrush(QPalette::HighlightedText, QColor(0x0F, 0x76, 0x6E));
  pal.setBrush(QPalette::Inactive, QPalette::Highlight, Qt::transparent);
  list_->setPalette(pal);
  list_->setItemDelegate(new ContinuousListDelegate(list_));
  well_layout->addWidget(list_, 1);

  auto* action_row = new QWidget(this);
  action_row->setAttribute(Qt::WA_TranslucentBackground, true);
  auto* action_layout = new QHBoxLayout(action_row);
  action_layout->setContentsMargins(2, 0, 2, 0);
  action_layout->setSpacing(10);

  button_box_ = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                     Qt::Horizontal, action_row);
  if (QPushButton* ok = button_box_->button(QDialogButtonBox::Ok)) {
    ok->setText(QStringLiteral("Confirm"));
    ok->setCursor(Qt::PointingHandCursor);
    ok->setStyleSheet(
        style::sheet(QStringLiteral("add_panel")));
    ok->setDefault(true);
  }
  if (QPushButton* cancel = button_box_->button(QDialogButtonBox::Cancel)) {
    cancel->setText(QStringLiteral("Cancel"));
    cancel->setCursor(Qt::PointingHandCursor);
    cancel->setStyleSheet(
        style::sheet(QStringLiteral("add_panel")));
  }
  action_layout->addStretch(1);
  action_layout->addWidget(button_box_, 0, Qt::AlignRight);

  root->addWidget(well, 1);
  root->addWidget(action_row);

  connect(button_box_, &QDialogButtonBox::accepted, this, &QDialog::accept);
  connect(button_box_, &QDialogButtonBox::rejected, this, &QDialog::reject);
  connect(list_, &QListWidget::itemDoubleClicked, this, &QDialog::accept);
  connect(list_, &QListWidget::itemSelectionChanged, this, [this]() {
    if (QPushButton* ok = button_box_->button(QDialogButtonBox::Ok)) {
      ok->setEnabled(list_->currentItem() != nullptr);
    }
  });

  populate(available_panels);
}

QSize AddPanelDialog::sizeHint() const { return {420, 560}; }

void AddPanelDialog::paintEvent(QPaintEvent* /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QRectF outer = QRectF(rect()).adjusted(kCardInset, kCardInset, -kCardInset,
                                               -kCardInset);
  QPainterPath card;
  card.addRoundedRect(outer, kCardRadius, kCardRadius);

  for (int i = 8; i >= 1; --i) {
    QPainterPath glow;
    glow.addRoundedRect(outer.adjusted(-i * 0.7, -i * 0.35, i * 0.7, i * 0.9),
                        kCardRadius + i * 0.4, kCardRadius + i * 0.4);
    painter.fillPath(glow, QColor(15, 23, 42, 4 + (8 - i)));
  }

  painter.fillPath(card, QColor(252, 253, 253, 228));
  QLinearGradient wash(outer.topLeft(), outer.bottomRight());
  wash.setColorAt(0.0, QColor(255, 255, 255, 70));
  wash.setColorAt(0.45, QColor(245, 252, 250, 28));
  wash.setColorAt(1.0, QColor(241, 245, 249, 40));
  painter.fillPath(card, wash);

  QLinearGradient sheen(outer.topLeft(),
                        QPointF(outer.left(), outer.top() + outer.height() * 0.38));
  sheen.setColorAt(0.0, QColor(255, 255, 255, 150));
  sheen.setColorAt(1.0, QColor(255, 255, 255, 0));
  painter.fillPath(card, sheen);

  painter.setPen(QPen(QColor(255, 255, 255, 200), 1.2));
  painter.drawPath(card);
  painter.setPen(QPen(QColor(203, 213, 225, 110), 1.0));
  painter.drawRoundedRect(outer.adjusted(1.0, 1.0, -1.0, -1.0), kCardRadius - 1.0,
                          kCardRadius - 1.0);
}

void AddPanelDialog::mousePressEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    QWidget* hit = childAt(event->pos());
    if (hit == nullptr || hit == this) {
      dragging_ = true;
      drag_offset_ = event->globalPosition().toPoint() - frameGeometry().topLeft();
      event->accept();
      return;
    }
  }
  QDialog::mousePressEvent(event);
}

void AddPanelDialog::mouseMoveEvent(QMouseEvent* event) {
  if (dragging_ && (event->buttons() & Qt::LeftButton)) {
    move(event->globalPosition().toPoint() - drag_offset_);
    event->accept();
    return;
  }
  QDialog::mouseMoveEvent(event);
}

void AddPanelDialog::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::LeftButton) {
    dragging_ = false;
  }
  QDialog::mouseReleaseEvent(event);
}

void AddPanelDialog::populate(const QStringList& available_panels) {
  list_->clear();
  for (const auto& entry : PanelCatalog()) {
    if (!entry.isImplemented()) {
      continue;
    }
    if (!available_panels.contains(QLatin1String(entry.object_name))) {
      continue;
    }
    const QString label = tr(entry.label);
    auto* item = new QListWidgetItem(
        IconLoader::panelIcon(QString::fromLatin1(entry.icon_id)), label);
    item->setData(Qt::UserRole, QString::fromLatin1(entry.object_name));
    item->setToolTip(tr(entry.description));
    list_->addItem(item);
  }
  if (list_->count() > 0) {
    list_->setCurrentRow(0);
  }
  if (QPushButton* ok = button_box_->button(QDialogButtonBox::Ok)) {
    ok->setEnabled(list_->currentItem() != nullptr);
  }
}

QString AddPanelDialog::selectedPanelObjectName() const {
  const QListWidgetItem* item = list_->currentItem();
  if (item == nullptr) {
    return {};
  }
  return item->data(Qt::UserRole).toString();
}

}  // namespace autoviz
