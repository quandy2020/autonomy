/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file translation.hpp
 * @brief Qt and Autoviz translator installation helpers.
 *
 * Resolves an effective locale from preferences (or the system) and installs
 * both Qt base and Autoviz @c .qm translators on the application.
 *
 * @see AppUiPreferences::language_code
 * @see FrameSession::applyUiPreferences()
 * @see EffectiveLocaleName()
 */

#pragma once

#include <QString>

class QApplication;

namespace autoviz {

/**
 * @brief Returns the locale suffix used for translation loading.
 *
 * Empty @p language_code follows the system locale. Normalizes aliases
 * (e.g. @c zh → @c zh_CN) as implemented.
 *
 * @param language_code Preference language code, or empty for system.
 * @return Locale name suitable for @c QTranslator::load() suffixes.
 */
QString EffectiveLocaleName(const QString& language_code);

/**
 * @brief Install Qt and Autoviz translators.
 *
 * Empty @p language_code follows the system locale. Replaces previously
 * installed Autoviz translators when called again (e.g. after settings
 * change).
 *
 * @param app Application instance.
 * @param language_code Preference language code, or empty for system.
 * @return @c true if at least the expected translators were installed (or
 *         English/no-op path succeeded).
 *
 * @see EffectiveLocaleName()
 * @see SaveAppUiPreferences()
 */
bool InstallAppTranslations(QApplication& app,
                            const QString& language_code = {});

}  // namespace autoviz
