/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

class QWidget;

namespace autoviz {

/**
 * @brief When Autolink reports shutdown (Ctrl+C / SIGTERM), close @p main_window.
 *
 * Autolink's SIGINT handler only flips process state. A short GUI-thread timer
 * polls that state and calls @c QWidget::close() so
 * @c VisualizationFrame::closeEvent can sync-destroy Ogre/GLX safely.
 */
void InstallQuitSignalHandler(QWidget* main_window);

/**
 * @brief True after SIGINT/SIGTERM was delivered to Autoviz (not bare Autolink
 *        state flips).
 */
bool UserRequestedQuit();

}  // namespace autoviz
