/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QString>

class QApplication;

namespace autoviz {

/** Returns the locale suffix used for translation loading. */
QString EffectiveLocaleName(const QString& language_code);

/** Install Qt and Autoviz translators. Empty language_code follows system locale. */
bool InstallAppTranslations(QApplication& app, const QString& language_code = {});

}  // namespace autoviz
