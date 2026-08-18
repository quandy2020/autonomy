/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <memory>

#include <QApplication>
#include <QCommandLineParser>
#include <QFile>
#include <QFileInfo>
#include <QGuiApplication>
#include <QLibraryInfo>
#include <QLocale>
#include <QTimer>

#include "autoviz/ui/app_preferences.hpp"
#include "autoviz/ui/app_theme.hpp"
#include "autoviz/ui/app_translation.hpp"
#include <glog/logging.h>

#ifdef AUTOVIZ_USE_QML_DRONE
#include <QtPlugin>
Q_IMPORT_PLUGIN(Autoviz_Vehicle3DPlugin)
#endif

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/platform/opengl_setup.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/splash_screen.hpp"
#include "autoviz/ui/visualization_frame.hpp"

namespace {

QString defaultConfigPath() {
  return QStringLiteral("config/default.autoviz");
}

}  // namespace

int main(int argc, char** argv) {
  autoviz::platform::configureOpenGLDefaults();
  QApplication app(argc, argv);
  app.setApplicationName(QStringLiteral("Autoviz"));
  app.setApplicationDisplayName(QStringLiteral("Aviz"));
  app.setOrganizationName(QStringLiteral("Autonomy"));
  QGuiApplication::setDesktopFileName(
      QStringLiteral("org.autonomy.autoviz"));
  const QIcon app_icon = autoviz::IconLoader::applicationIcon();
  if (!app_icon.isNull()) {
    app.setWindowIcon(app_icon);
  }

  const autoviz::AppUiPreferences ui_preferences = autoviz::LoadAppUiPreferences();
  autoviz::InstallAppTranslations(app, ui_preferences.language_code);
  autoviz::ApplyAppTheme(app);

  QCommandLineParser parser;
  parser.setApplicationDescription(
      QStringLiteral("Autonomy 3D visualizer"));
  parser.addHelpOption();
  QCommandLineOption config_option(
      QStringList{QStringLiteral("config"), QStringLiteral("c")},
      QStringLiteral("Load session config (.autoviz)"),
      QStringLiteral("file"));
  parser.addOption(config_option);
  QCommandLineOption splash_option(
      QStringList{QStringLiteral("splash-screen"), QStringLiteral("s")},
      QStringLiteral("Splash screen image (empty path disables)"),
      QStringLiteral("splash_path"));
  parser.addOption(splash_option);
  parser.addPositionalArgument(
      QStringLiteral("files"),
      QStringLiteral(
          "Optional session config (.autoviz) and/or Autolink record "
          "(.record/.bag/.mcap) to open and play"),
      QStringLiteral("[files...]"));
  parser.process(app);

  QString splash_path;
  bool show_splash = true;
  if (parser.isSet(splash_option)) {
    splash_path = parser.value(splash_option);
    show_splash = !splash_path.isEmpty();
  } else {
    splash_path = QStringLiteral(":/autoviz/images/splash.png");
  }

  std::unique_ptr<autoviz::SplashScreen> splash;
  if (show_splash) {
    splash = autoviz::SplashScreen::create(splash_path);
    if (splash != nullptr) {
      if (!app_icon.isNull()) {
        splash->setWindowIcon(app_icon);
      }
      splash->show();
      splash->showStatusFor(QStringLiteral("Initializing"));
    }
  }

  auto manager = std::make_shared<autoviz::common::VisualizationManager>();
  if (splash != nullptr) {
    splash->showStatusFor(QStringLiteral("Initializing"));
  }
  if (!manager->initialize(argv[0])) {
    LOG(ERROR) << "Failed to start Autoviz.";
    return 1;
  }

  QString config_path = parser.value(config_option);
  QStringList record_paths;
  const QStringList positional = parser.positionalArguments();
  for (const QString& argument : positional) {
    const QString suffix = QFileInfo(argument).suffix().toLower();
    if (suffix == QLatin1String("autoviz") || suffix == QLatin1String("yaml") ||
        suffix == QLatin1String("rviz")) {
      if (config_path.isEmpty()) {
        config_path = argument;
      }
      continue;
    }
    record_paths.push_back(argument);
  }
  if (config_path.isEmpty()) {
    config_path = defaultConfigPath();
  }

  if (splash != nullptr) {
    splash->showStatusFor(QStringLiteral("Loading UI"));
  }
  autoviz::VisualizationFrame frame(manager);
  if (splash != nullptr) {
    splash->showStatusFor(QStringLiteral("Loading config"));
  }
  if (QFile::exists(config_path)) {
    frame.loadConfig(config_path);
  }

  if (splash != nullptr) {
    splash->finish();
    splash.reset();
  }

  frame.applyStartupWindowState();

  if (!record_paths.isEmpty()) {
    const QString record_path = record_paths.front();
    QTimer::singleShot(0, &frame, [record_path, &frame]() {
      frame.openRecordFile(record_path);
    });
  }

  const int code = app.exec();
  manager->shutdown();
  return code;
}
