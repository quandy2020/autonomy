/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file opengl_setup.hpp
 * @brief Process-wide OpenGL / Qt surface defaults for Autoviz rendering.
 *
 * Must run before constructing @c QApplication / @c QGuiApplication so Qt
 * picks the intended GL profile (and software fallback when needed).
 *
 * @see defaultSurfaceFormat
 * @see usesSoftwareOpenGL
 */

#pragma once

#include <QSurfaceFormat>

namespace autoviz {
namespace platform {

/**
 * @brief Configures Qt OpenGL defaults for the process.
 *
 * Sets the default @c QSurfaceFormat (and related Qt attributes) before the
 * application object exists. Safe / intended to call exactly once at startup.
 *
 * @warning Must run before constructing @c QApplication / @c QGuiApplication.
 *
 * @see defaultSurfaceFormat()
 */
void configureOpenGLDefaults();

/**
 * @brief Whether Autoviz is using a software OpenGL implementation.
 *
 * Useful for disabling GPU depth picking and adjusting render quality.
 *
 * @return @c true when software GL was selected / detected.
 *
 * @see common::ToolContext::gpu_picking_enabled
 */
bool usesSoftwareOpenGL();

/**
 * @brief Returns the @c QSurfaceFormat applied as the process default.
 *
 * @return Copy of the configured surface format (profile, samples, depth, …).
 *
 * @see configureOpenGLDefaults()
 */
QSurfaceFormat defaultSurfaceFormat();

}  // namespace platform
}  // namespace autoviz
