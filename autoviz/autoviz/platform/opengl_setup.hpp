/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QSurfaceFormat>

namespace autoviz {
namespace platform {

// Must run before constructing QApplication / QGuiApplication.
void configureOpenGLDefaults();

bool usesSoftwareOpenGL();
QSurfaceFormat defaultSurfaceFormat();

}  // namespace platform
}  // namespace autoviz
