/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/app/quit_signal.hpp"

#include <atomic>
#include <csignal>

#include <QTimer>
#include <QWidget>
#include <glog/logging.h>

#include "autolink/init.hpp"
#include "autolink/state.hpp"

namespace autoviz {
namespace {

std::atomic<bool> g_user_quit_signal{false};

void MarkShuttingDown(int /*sig*/) {
  g_user_quit_signal.store(true, std::memory_order_release);
  ::autolink::OnShutdown(SIGINT);
}

}  // namespace

void InstallQuitSignalHandler(QWidget* main_window) {
  if (main_window == nullptr) {
    return;
  }

  // Force SIGINT/SIGTERM → Autolink shutdown. Autolink::Init restores any
  // non-default handler; bash background jobs (&) inherit SIG_IGN for SIGINT,
  // so Ctrl+C / kill -INT would otherwise be ignored and never tear down Qt.
  g_user_quit_signal.store(false, std::memory_order_release);
  std::signal(SIGINT, MarkShuttingDown);
  std::signal(SIGTERM, MarkShuttingDown);

  // Poll only our signal flag (not bare Autolink IsShutdown). Spurious
  // STATE_SHUTTING_DOWN from shared teardown races was closing Autoviz ~20s
  // after RenderSystem ready under Docker stress (exit code 0, no SEGV).
  auto* timer = new QTimer(main_window);
  timer->setInterval(100);
  QObject::connect(timer, &QTimer::timeout, main_window,
                   [main_window, timer]() {
                     if (!g_user_quit_signal.load(std::memory_order_acquire)) {
                       return;
                     }
                     if (!::autolink::IsShutdown()) {
                       return;
                     }
                     timer->stop();
                     LOG(INFO) << "Quit signal: closing main window";
                     main_window->close();
                   });
  timer->start();
}

bool UserRequestedQuit() {
  return g_user_quit_signal.load(std::memory_order_acquire);
}

}  // namespace autoviz
