/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/ui/app_translation.hpp"

#include <QApplication>
#include <QLibraryInfo>
#include <QLocale>
#include <QTranslator>

namespace autoviz {
namespace {

QTranslator* g_qt_base_translator = nullptr;
QTranslator* g_qt_translator = nullptr;
QTranslator* g_autoviz_translator = nullptr;

void RemoveInstalledTranslators(QApplication& app) {
  if (g_autoviz_translator != nullptr) {
    app.removeTranslator(g_autoviz_translator);
  }
  if (g_qt_translator != nullptr) {
    app.removeTranslator(g_qt_translator);
  }
  if (g_qt_base_translator != nullptr) {
    app.removeTranslator(g_qt_base_translator);
  }
}

}  // namespace

QString EffectiveLocaleName(const QString& language_code) {
  if (language_code.trimmed().isEmpty() ||
      language_code == QLatin1String("system")) {
    return QLocale::system().name();
  }
  return language_code.trimmed();
}

bool InstallAppTranslations(QApplication& app, const QString& language_code) {
  static QTranslator qt_base_storage;
  static QTranslator qt_storage;
  static QTranslator autoviz_storage;

  RemoveInstalledTranslators(app);

  g_qt_base_translator = &qt_base_storage;
  g_qt_translator = &qt_storage;
  g_autoviz_translator = &autoviz_storage;

  const QString locale = EffectiveLocaleName(language_code);
  const QString lang = locale.section(QLatin1Char('_'), 0, 0);
  const QString translations_path =
      QLibraryInfo::path(QLibraryInfo::TranslationsPath);

  if (g_qt_base_translator->load(QStringLiteral("qtbase_") + locale,
                                 translations_path)) {
    app.installTranslator(g_qt_base_translator);
  }
  if (g_qt_translator->load(QStringLiteral("qt_") + locale, translations_path)) {
    app.installTranslator(g_qt_translator);
  }

  const auto try_load_aviz = [&](const QString& suffix) {
    return g_autoviz_translator->load(QStringLiteral("autoviz_") + suffix,
                                      QStringLiteral(":/i18n"));
  };

  if (lang == QLatin1String("en")) {
    return true;
  }
  return try_load_aviz(locale) || try_load_aviz(lang);
}

}  // namespace autoviz
