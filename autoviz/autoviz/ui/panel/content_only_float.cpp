/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/panel/content_only_float.hpp"

#include <QApplication>
#include <QEvent>
#include <QFont>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPointer>
#include <QRect>
#include <QSizeGrip>
#include <QTimer>
#include <QToolButton>
#include <QWidget>

#include "autoviz/ui/panel/dock.hpp"

namespace autoviz {
namespace {

constexpr char kPropContentOnly[] = "contentOnlyFloat";
constexpr char kControllerName[] = "contentOnlyFloatController";
constexpr int kTopDragPx = 28;
constexpr int kEdgePx = 6;
constexpr int kHoverHideMs = 1200;

enum class ResizeEdge : quint8 {
  kNone = 0,
  kLeft = 1,
  kRight = 2,
  kTop = 4,
  kBottom = 8,
  kTopLeft = kTop | kLeft,
  kTopRight = kTop | kRight,
  kBottomLeft = kBottom | kLeft,
  kBottomRight = kBottom | kRight,
};

ResizeEdge HitTestEdge(const QWidget* host, const QPoint& local) {
  if (host == nullptr) {
    return ResizeEdge::kNone;
  }
  const int w = host->width();
  const int h = host->height();
  quint8 edge = 0;
  if (local.x() <= kEdgePx) {
    edge |= static_cast<quint8>(ResizeEdge::kLeft);
  }
  if (local.x() >= w - kEdgePx) {
    edge |= static_cast<quint8>(ResizeEdge::kRight);
  }
  if (local.y() <= kEdgePx) {
    edge |= static_cast<quint8>(ResizeEdge::kTop);
  }
  if (local.y() >= h - kEdgePx) {
    edge |= static_cast<quint8>(ResizeEdge::kBottom);
  }
  return static_cast<ResizeEdge>(edge);
}

Qt::CursorShape CursorForEdge(ResizeEdge edge) {
  switch (edge) {
    case ResizeEdge::kLeft:
    case ResizeEdge::kRight:
      return Qt::SizeHorCursor;
    case ResizeEdge::kTop:
    case ResizeEdge::kBottom:
      return Qt::SizeVerCursor;
    case ResizeEdge::kTopLeft:
    case ResizeEdge::kBottomRight:
      return Qt::SizeFDiagCursor;
    case ResizeEdge::kTopRight:
    case ResizeEdge::kBottomLeft:
      return Qt::SizeBDiagCursor;
    default:
      return Qt::ArrowCursor;
  }
}

class ContentOnlyHoverBar : public QWidget {
 public:
  ContentOnlyHoverBar(QWidget* parent, std::function<void()> on_show_chrome,
                      std::function<void()> on_dock,
                      std::function<void()> on_close)
      : QWidget(parent),
        on_show_chrome_(std::move(on_show_chrome)),
        on_dock_(std::move(on_dock)),
        on_close_(std::move(on_close)) {
    setObjectName(QStringLiteral("contentOnlyHoverBar"));
    // Opaque dark plate (same idea as image HUD) — translucent light CSS over
    // the frame looked soft / blurred on HiDPI and washed out on bright pixels.
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setAttribute(Qt::WA_StyledBackground, false);
    setAutoFillBackground(false);
    setFixedHeight(32);

    QFont bar_font = font();
    bar_font.setStyleStrategy(
        static_cast<QFont::StyleStrategy>(QFont::PreferDefault |
                                         QFont::NoSubpixelAntialias));
    setFont(bar_font);

    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(10, 0, 8, 0);
    layout->setSpacing(6);

    auto style_label = [this](QLabel* label) {
      label->setAttribute(Qt::WA_TranslucentBackground, true);
      label->setStyleSheet(QStringLiteral("color: #F5F5F5; background: transparent;"));
      label->setFont(font());
    };
    auto style_button = [this](QToolButton* button) {
      button->setAutoRaise(true);
      button->setCursor(Qt::PointingHandCursor);
      button->setFont(font());
      button->setStyleSheet(
          QStringLiteral("QToolButton {"
                         "  color: #F5F5F5;"
                         "  background: transparent;"
                         "  border: none;"
                         "  padding: 4px 8px;"
                         "}"
                         "QToolButton:hover {"
                         "  background: rgba(255,255,255,28);"
                         "  border-radius: 4px;"
                         "}"
                         "QToolButton:pressed {"
                         "  background: rgba(255,255,255,48);"
                         "}"));
    };

    title_ = new QLabel(this);
    title_->setText(QObject::tr("Image"));
    style_label(title_);
    layout->addWidget(title_, 1);

    auto* show_chrome = new QToolButton(this);
    show_chrome->setText(QObject::tr("Show chrome"));
    style_button(show_chrome);
    layout->addWidget(show_chrome);
    QObject::connect(show_chrome, &QToolButton::clicked, this, [this]() {
      if (on_show_chrome_) {
        on_show_chrome_();
      }
    });

    auto* dock_btn = new QToolButton(this);
    dock_btn->setText(QObject::tr("Dock"));
    style_button(dock_btn);
    layout->addWidget(dock_btn);
    QObject::connect(dock_btn, &QToolButton::clicked, this, [this]() {
      if (on_dock_) {
        on_dock_();
      }
    });

    auto* close_btn = new QToolButton(this);
    close_btn->setText(QObject::tr("Close"));
    style_button(close_btn);
    layout->addWidget(close_btn);
    QObject::connect(close_btn, &QToolButton::clicked, this, [this]() {
      if (on_close_) {
        on_close_();
      }
    });

    hide();
  }

  void setTitle(const QString& title) {
    if (title_ != nullptr) {
      title_->setText(title.isEmpty() ? QObject::tr("Image") : title);
    }
  }

 protected:
  void paintEvent(QPaintEvent* /*event*/) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setRenderHint(QPainter::TextAntialiasing, true);
    // Near-opaque dark fill — avoids alpha-blend blur over the live image.
    painter.fillRect(rect(), QColor(12, 14, 18, 236));
    painter.setPen(QColor(255, 255, 255, 36));
    painter.drawLine(0, height() - 1, width(), height() - 1);
  }

 private:
  QLabel* title_ = nullptr;
  std::function<void()> on_show_chrome_;
  std::function<void()> on_dock_;
  std::function<void()> on_close_;
};

class ContentOnlyFloatController : public QObject {
 public:
  ContentOnlyFloatController(PanelDockWidget* dock,
                             std::function<void()> on_show_chrome,
                             std::function<void()> on_dock)
      : QObject(dock),
        dock_(dock),
        on_show_chrome_(std::move(on_show_chrome)),
        on_dock_(std::move(on_dock)) {
    setObjectName(QString::fromLatin1(kControllerName));
    hide_timer_.setSingleShot(true);
    QObject::connect(&hide_timer_, &QTimer::timeout, this, [this]() {
      if (bar_ != nullptr && !bar_hovered_ && !dragging_ &&
          resize_edge_ == ResizeEdge::kNone) {
        bar_->hide();
      }
    });

    // Prefer the float-shell wrap so the hover bar sits in a Qt margin above
    // native GL (3D View). Fall back to the dock for Image / non-shell floats.
    chrome_host_ = ResolveChromeHost(dock);
    if (chrome_host_ == nullptr) {
      chrome_host_ = dock;
    }

    bar_ = new ContentOnlyHoverBar(
        chrome_host_,
        [this]() {
          if (on_show_chrome_) {
            on_show_chrome_();
          }
        },
        [this]() {
          if (on_dock_) {
            on_dock_();
          }
        },
        [this]() {
          if (dock_ != nullptr) {
            dock_->close();
          }
        });
    bar_->setParent(chrome_host_);
    bar_->raise();
    bar_->installEventFilter(this);

    installTracking(chrome_host_);
    if (dock_ != nullptr && dock_ != chrome_host_) {
      installTracking(dock_);
    }
    placeBar();
  }

  static QWidget* ResolveChromeHost(PanelDockWidget* dock) {
    if (dock == nullptr) {
      return nullptr;
    }
    // Match FrameLayout float-shell wrap object name.
    QWidget* parent = dock->parentWidget();
    while (parent != nullptr) {
      if (parent->objectName() == QLatin1String("intentionalFloatShellWrap")) {
        return parent;
      }
      parent = parent->parentWidget();
    }
    return nullptr;
  }

  ~ContentOnlyFloatController() override {
    uninstallTracking();
    // Hover bar is parented to the dock (QObject controller cannot own a
    // QWidget visually) — delete it explicitly so disable/redock does not leave
    // a floating overlay behind.
    if (bar_ != nullptr) {
      bar_->hide();
      delete bar_;
      bar_ = nullptr;
    }
  }

  void installTracking(QWidget* widget) {
    if (widget == nullptr) {
      return;
    }
    widget->setMouseTracking(true);
    widget->installEventFilter(this);
    tracked_.append(widget);
    const auto children =
        widget->findChildren<QWidget*>(QString(), Qt::FindDirectChildrenOnly);
    for (QWidget* child : children) {
      if (child == bar_) {
        continue;
      }
      installTracking(child);
    }
  }

  void uninstallTracking() {
    for (const QPointer<QWidget>& widget : tracked_) {
      if (widget != nullptr) {
        widget->removeEventFilter(this);
      }
    }
    tracked_.clear();
    if (bar_ != nullptr) {
      bar_->removeEventFilter(this);
    }
    if (dock_ != nullptr) {
      dock_->removeEventFilter(this);
      if (QWidget* content = dock_->widget()) {
        content->removeEventFilter(this);
      }
    }
  }

  void setTitle(const QString& title) {
    if (bar_ != nullptr) {
      bar_->setTitle(title);
    }
  }

  bool eventFilter(QObject* watched, QEvent* event) override {
    if (dock_ == nullptr) {
      return QObject::eventFilter(watched, event);
    }

    if (watched == bar_) {
      if (event->type() == QEvent::Enter) {
        bar_hovered_ = true;
        hide_timer_.stop();
        showBar();
      } else if (event->type() == QEvent::Leave) {
        bar_hovered_ = false;
        hide_timer_.start(kHoverHideMs);
      }
      return QObject::eventFilter(watched, event);
    }

    switch (event->type()) {
      case QEvent::Resize:
      case QEvent::Show:
        placeBar();
        break;
      case QEvent::KeyPress: {
        auto* key = static_cast<QKeyEvent*>(event);
        if (key->key() == Qt::Key_Escape && !key->modifiers()) {
          if (on_show_chrome_) {
            on_show_chrome_();
            return true;
          }
        }
        break;
      }
      case QEvent::MouseMove: {
        auto* mouse = static_cast<QMouseEvent*>(event);
        QWidget* host = chrome_host_ != nullptr ? chrome_host_.data() : dock_.data();
        const QPoint local =
            host != nullptr
                ? host->mapFromGlobal(mouse->globalPosition().toPoint())
                : mouse->position().toPoint();
        if (dragging_) {
          if (QWidget* top = dock_->window()) {
            top->move(mouse->globalPosition().toPoint() - drag_offset_);
          }
          return true;
        }
        if (resize_edge_ != ResizeEdge::kNone) {
          applyResize(mouse->globalPosition().toPoint());
          return true;
        }
        updateHoverChrome(local);
        const ResizeEdge edge = HitTestEdge(host, local);
        if (edge != ResizeEdge::kNone) {
          host->setCursor(CursorForEdge(edge));
        } else if (local.y() <= kTopDragPx) {
          host->setCursor(Qt::SizeAllCursor);
        } else {
          host->unsetCursor();
        }
        break;
      }
      case QEvent::MouseButtonPress: {
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (mouse->button() != Qt::LeftButton) {
          break;
        }
        QWidget* host = chrome_host_ != nullptr ? chrome_host_.data() : dock_.data();
        const QPoint local =
            host != nullptr
                ? host->mapFromGlobal(mouse->globalPosition().toPoint())
                : mouse->position().toPoint();
        // Clicks on hover bar controls should pass through normally.
        if (bar_ != nullptr && bar_->isVisible() &&
            bar_->geometry().contains(local)) {
          break;
        }
        QWidget* top = dock_->window() != nullptr ? dock_->window() : dock_.data();
        const ResizeEdge edge = HitTestEdge(host, local);
        if (edge != ResizeEdge::kNone) {
          resize_edge_ = edge;
          resize_origin_global_ = mouse->globalPosition().toPoint();
          resize_origin_geom_ = top != nullptr ? top->geometry() : dock_->geometry();
          return true;
        }
        if (local.y() <= kTopDragPx) {
          dragging_ = true;
          drag_offset_ =
              mouse->globalPosition().toPoint() -
              (top != nullptr ? top->pos() : dock_->pos());
          showBar();
          return true;
        }
        break;
      }
      case QEvent::MouseButtonRelease: {
        auto* mouse = static_cast<QMouseEvent*>(event);
        if (mouse->button() == Qt::LeftButton) {
          dragging_ = false;
          resize_edge_ = ResizeEdge::kNone;
        }
        break;
      }
      case QEvent::Leave:
        if (!dragging_ && resize_edge_ == ResizeEdge::kNone) {
          hide_timer_.start(kHoverHideMs);
          if (chrome_host_ != nullptr) {
            chrome_host_->unsetCursor();
          }
          if (dock_ != nullptr) {
            dock_->unsetCursor();
          }
        }
        break;
      default:
        break;
    }
    return QObject::eventFilter(watched, event);
  }

 private:
  void placeBar() {
    QWidget* host = chrome_host_ != nullptr ? chrome_host_.data() : dock_.data();
    if (host == nullptr || bar_ == nullptr) {
      return;
    }
    // Integer device-pixel-aligned geometry avoids HiDPI subpixel blur.
    const qreal dpr = qMax<qreal>(1.0, host->devicePixelRatioF());
    const int w = qMax(1, static_cast<int>(qRound(host->width() * dpr) / dpr));
    const int h = bar_->height();
    bar_->setGeometry(0, 0, w, h);
    bar_->raise();
  }

  void showBar() {
    if (bar_ == nullptr) {
      return;
    }
    placeBar();
    bar_->show();
    bar_->raise();
    hide_timer_.stop();
  }

  void updateHoverChrome(const QPoint& local) {
    if (local.y() <= kTopDragPx + 4) {
      showBar();
      hide_timer_.start(kHoverHideMs);
    }
  }

  void applyResize(const QPoint& global) {
    if (dock_ == nullptr) {
      return;
    }
    QWidget* top = dock_->window() != nullptr ? dock_->window() : dock_.data();
    if (top == nullptr) {
      return;
    }
    QRect geom = resize_origin_geom_;
    const QPoint delta = global - resize_origin_global_;
    const quint8 edge = static_cast<quint8>(resize_edge_);
    if (edge & static_cast<quint8>(ResizeEdge::kLeft)) {
      geom.setLeft(resize_origin_geom_.left() + delta.x());
    }
    if (edge & static_cast<quint8>(ResizeEdge::kRight)) {
      geom.setRight(resize_origin_geom_.right() + delta.x());
    }
    if (edge & static_cast<quint8>(ResizeEdge::kTop)) {
      geom.setTop(resize_origin_geom_.top() + delta.y());
    }
    if (edge & static_cast<quint8>(ResizeEdge::kBottom)) {
      geom.setBottom(resize_origin_geom_.bottom() + delta.y());
    }
    constexpr int kMinW = 320;
    constexpr int kMinH = 240;
    if (geom.width() < kMinW) {
      if (edge & static_cast<quint8>(ResizeEdge::kLeft)) {
        geom.setLeft(geom.right() - kMinW + 1);
      } else {
        geom.setWidth(kMinW);
      }
    }
    if (geom.height() < kMinH) {
      if (edge & static_cast<quint8>(ResizeEdge::kTop)) {
        geom.setTop(geom.bottom() - kMinH + 1);
      } else {
        geom.setHeight(kMinH);
      }
    }
    top->setGeometry(geom);
  }

  QPointer<PanelDockWidget> dock_;
  QPointer<QWidget> chrome_host_;
  ContentOnlyHoverBar* bar_ = nullptr;
  QList<QPointer<QWidget>> tracked_;
  QTimer hide_timer_;
  bool bar_hovered_ = false;
  bool dragging_ = false;
  QPoint drag_offset_;
  ResizeEdge resize_edge_ = ResizeEdge::kNone;
  QPoint resize_origin_global_;
  QRect resize_origin_geom_;
  std::function<void()> on_show_chrome_;
  std::function<void()> on_dock_;
};

/** Destroy any prior controller. Anonymous class has no Q_OBJECT, so look up by
 *  objectName as QObject* (typed findChild would always return nullptr). */
void DestroyContentOnlyController(PanelDockWidget* dock) {
  if (dock == nullptr) {
    return;
  }
  if (QObject* existing = dock->findChild<QObject*>(
          QString::fromLatin1(kControllerName), Qt::FindDirectChildrenOnly)) {
    // Destructor uninstalls event filters from tracked widgets.
    delete existing;
  }
}

}  // namespace

bool IsContentOnlyFloat(const PanelDockWidget* dock) {
  return dock != nullptr && dock->property(kPropContentOnly).toBool();
}

void SetContentOnlyFloatChrome(PanelDockWidget* dock, bool enabled,
                               std::function<void()> on_show_chrome,
                               std::function<void()> on_dock,
                               const QString& title) {
  if (dock == nullptr) {
    return;
  }

  DestroyContentOnlyController(dock);
  dock->setProperty(kPropContentOnly, enabled);

  if (!enabled) {
    dock->setTitleBarVisible(true);
    dock->unsetCursor();
    // Restore intentional-float grip if still floating freely.
    if (dock->property("intentionalFloat").toBool()) {
      if (dock->findChild<QSizeGrip*>(
              QStringLiteral("intentionalFloatGrip")) == nullptr) {
        // Grip is re-created by setIntentionalFloatChrome; caller handles that.
      }
    }
    return;
  }

  dock->setTitleBarVisible(false);
  // Hide size grip while chrome-less (edge hit-zones replace it).
  if (auto* grip = dock->findChild<QSizeGrip*>(
          QStringLiteral("intentionalFloatGrip"))) {
    grip->hide();
  }

  auto* controller =
      new ContentOnlyFloatController(dock, std::move(on_show_chrome),
                                     std::move(on_dock));
  controller->setTitle(title);
}

}  // namespace autoviz
