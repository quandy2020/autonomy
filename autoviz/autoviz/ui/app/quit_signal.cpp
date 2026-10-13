/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/app/quit_signal.hpp"

#include <csignal>

#include <QTimer>
#include <QWidget>

#include "autolink/init.hpp"
#include "autolink/state.hpp"

namespace autoviz {
namespace {

void MarkShuttingDown(int /*sig*/) { ::autolink::OnShutdown(SIGINT); }

}  // namespace

void InstallQuitSignalHandler(QWidget* main_window) {
  if (main_window == nullptr) {
    return;
  }

  // Force SIGINT/SIGTERM → Autolink shutdown. Autolink::Init restores any
  // non-default handler; bash background jobs (&) inherit SIG_IGN for SIGINT,
  // so Ctrl+C / kill -INT would otherwise be ignored and never tear down Qt.
  std::signal(SIGINT, MarkShuttingDown);
  std::signal(SIGTERM, MarkShuttingDown);

  // Poll Autolink shutdown state on the GUI thread and close the main window.
  // That runs VisualizationFrame::closeEvent (sync GLX teardown) instead of
  // letting Qt tear down WA_PaintOnScreen widgets via closeAllWindows.
  auto* timer = new QTimer(main_window);
  timer->setInterval(100);
  QObject::connect(timer, &QTimer::timeout, main_window,
                   [main_window, timer]() {
                     if (!::autolink::IsShutdown()) {
                       return;
                     }
                     timer->stop();
                     main_window->close();
                   });
  timer->start();
}

}  // namespace autoviz
