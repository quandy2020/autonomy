/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file mosaic_restore_test.cpp
 * @brief Offscreen regression tests for MainPanelHost mosaic (split tree)
 *        restore — the session "MainPanelState" path.
 */

#include <QApplication>
#include <QDockWidget>
#include <QString>
#include <QWidget>

#include <cstdio>
#include <functional>

#include "autoviz/ui/panel_host.hpp"

namespace {

int g_failures = 0;

void Check(bool condition, const char* what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_failures;
  }
}

QDockWidget* MakeDock(const char* name) {
  auto* dock = new QDockWidget(QString::fromLatin1(name));
  dock->setObjectName(QString::fromLatin1(name));
  dock->setWidget(new QWidget());
  return dock;
}

QByteArray Mosaic(const char* json) {
  return QByteArray("mosaic:v1:") + QByteArray(json);
}

using Resolver = std::function<QDockWidget*(const QString&)>;

void TestTwoLeafSplit() {
  autoviz::MainPanelHost host;
  QDockWidget* viewport = MakeDock("ViewportDock");
  QDockWidget* plot = MakeDock("PlotDock");
  const Resolver resolve = [&](const QString& name) -> QDockWidget* {
    if (name == QLatin1String("ViewportDock")) return viewport;
    if (name == QLatin1String("PlotDock")) return plot;
    return nullptr;
  };
  const bool restored = host.restoreMosaicLayout(
      Mosaic(R"({"root":{"type":"split","orientation":"horizontal",)"
             R"("sizes":[480,480],"children":[)"
             R"({"type":"leaf","name":"ViewportDock"},)"
             R"({"type":"leaf","name":"PlotDock"}]}})"),
      resolve);
  Check(restored, "two-leaf mosaic restores");
  Check(host.hostedPanels().size() == 2,
        "two-leaf mosaic hosts exactly two panes");
  Check(host.hostsPanel(viewport) && host.hostsPanel(plot),
        "both two-leaf docks are owned by the mosaic host");
}

void TestNestedSplit() {
  autoviz::MainPanelHost host;
  QDockWidget* teleop = MakeDock("TeleopDock");
  QDockWidget* map = MakeDock("MapDock");
  QDockWidget* service = MakeDock("ServiceDock");
  const Resolver resolve = [&](const QString& name) -> QDockWidget* {
    if (name == QLatin1String("TeleopDock")) return teleop;
    if (name == QLatin1String("MapDock")) return map;
    if (name == QLatin1String("ServiceDock")) return service;
    return nullptr;
  };
  const bool restored = host.restoreMosaicLayout(
      Mosaic(R"({"root":{"type":"split","orientation":"horizontal",)"
             R"("sizes":[480,480],"children":[)"
             R"({"type":"split","orientation":"vertical","sizes":[240,240],)"
             R"("children":[{"type":"leaf","name":"TeleopDock"},)"
             R"({"type":"leaf","name":"MapDock"}]},)"
             R"({"type":"leaf","name":"ServiceDock"}]}})"),
      resolve);
  Check(restored, "nested mosaic restores");
  Check(host.hostedPanels().size() == 3,
        "nested mosaic hosts all three panes");
}

void TestUnknownLeafDoesNotDropKnownOnes() {
  autoviz::MainPanelHost host;
  QDockWidget* viewport = MakeDock("ViewportDock");
  const Resolver resolve = [&](const QString& name) -> QDockWidget* {
    if (name == QLatin1String("ViewportDock")) return viewport;
    return nullptr;  // e.g. plugin uninstalled
  };
  const bool restored = host.restoreMosaicLayout(
      Mosaic(R"({"root":{"type":"split","orientation":"horizontal",)"
             R"("children":[{"type":"leaf","name":"ViewportDock"},)"
             R"({"type":"leaf","name":"MissingDock"}]}})"),
      resolve);
  Check(restored, "mosaic with one unresolved leaf still restores");
  Check(host.hostedPanels().size() == 2,
        "unresolved leaf keeps a placeholder pane");
  bool placeholder_found = false;
  for (QDockWidget* dock : host.hostedPanels()) {
    if (dock == nullptr || !dock->property("mosaicPlaceholder").toBool()) {
      continue;
    }
    if (dock->property("mosaicMissingName").toString() ==
        QLatin1String("MissingDock")) {
      placeholder_found = true;
    }
  }
  Check(placeholder_found, "placeholder records the missing panel type");

  const QByteArray saved = host.saveMosaicLayout();
  Check(saved.contains("MissingDock"),
        "placeholder persists the original leaf name");
  Check(!saved.contains("MosaicPlaceholder"),
        "placeholder object name is not persisted");
}

void TestInvalidPayload() {
  autoviz::MainPanelHost host;
  Check(!host.restoreMosaicLayout(QByteArray("not-a-mosaic"), Resolver()),
        "non-mosaic payload is rejected");
  Check(!host.restoreMosaicLayout(
            QByteArray("mosaic:v1:{\"root\":{}}"),
            [](const QString&) { return static_cast<QDockWidget*>(nullptr); }),
        "empty root is rejected");
}

}  // namespace

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  TestTwoLeafSplit();
  TestNestedSplit();
  TestUnknownLeafDoesNotDropKnownOnes();
  TestInvalidPayload();
  if (g_failures != 0) {
    std::fprintf(stderr, "%d mosaic test(s) failed\n", g_failures);
    return 1;
  }
  std::printf("mosaic restore tests passed\n");
  return 0;
}

