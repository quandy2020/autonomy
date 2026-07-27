/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <memory>

#include <QApplication>
#include <QCommandLineParser>
#include <QFile>
#include <QGuiApplication>
#include <QLibraryInfo>
#include <QLocale>
#include <QTranslator>
#include <glog/logging.h>

#ifdef AUTOVIZ_USE_QML_DRONE
#include <QtPlugin>
Q_IMPORT_PLUGIN(Autoviz_Vehicle3DPlugin)
#endif

#include "autoviz/common/visualization_manager.hpp"
#include "autoviz/ui/icon_loader.hpp"
#include "autoviz/ui/splash_screen.hpp"
#include "autoviz/ui/visualization_frame.hpp"

namespace {

QString defaultConfigPath() {
  return QStringLiteral("config/default.autoviz");
}

void installTranslations(QApplication& app) {
  static QTranslator qt_base_translator;
  static QTranslator qt_translator;
  static QTranslator autoviz_translator;

  const QString locale = QLocale::system().name();
  const QString lang = locale.section(QLatin1Char('_'), 0, 0);

  const QString translations_path =
      QLibraryInfo::path(QLibraryInfo::TranslationsPath);
  if (qt_base_translator.load(QStringLiteral("qtbase_") + locale,
                              translations_path)) {
    app.installTranslator(&qt_base_translator);
  }
  if (qt_translator.load(QStringLiteral("qt_") + locale, translations_path)) {
    app.installTranslator(&qt_translator);
  }

  const auto try_load_aviz = [&](const QString& suffix) {
    return autoviz_translator.load(QStringLiteral("autoviz_") + suffix,
                                QStringLiteral(":/i18n"));
  };
  if (try_load_aviz(locale) || try_load_aviz(lang)) {
    app.installTranslator(&autoviz_translator);
  }
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

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
  installTranslations(app);

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

  frame.show();
  const int code = app.exec();
  manager->shutdown();
  return code;
}
