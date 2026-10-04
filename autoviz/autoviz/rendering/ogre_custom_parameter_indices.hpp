/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 * Shader custom parameter indices — aligned with rviz_rendering ogre_media GLSL.
 *****************************************************************************/

/**
 * @file ogre_custom_parameter_indices.hpp
 * @brief Ogre custom-parameter slot indices for rviz_rendering GLSL materials.
 *
 * These macros match the @c custom_parameter indices expected by Autoviz /
 * rviz @c ogre_media point-cloud and pick shaders. Pass values via
 * @c Ogre::Renderable::setCustomParameter using these indices.
 *
 * @see OgrePointCloud
 * @see OgrePointCloudRenderable
 */

#pragma once

/**
 * @def AUTOVIZ_OGRE_SIZE_PARAMETER
 * @brief Custom parameter index for point / billboard size (width/height/depth).
 */
#define AUTOVIZ_OGRE_SIZE_PARAMETER 0

/**
 * @def AUTOVIZ_OGRE_ALPHA_PARAMETER
 * @brief Custom parameter index for global alpha multiplier.
 */
#define AUTOVIZ_OGRE_ALPHA_PARAMETER 1

/**
 * @def AUTOVIZ_OGRE_PICK_COLOR_PARAMETER
 * @brief Custom parameter index for GPU pick-pass encoded color.
 */
#define AUTOVIZ_OGRE_PICK_COLOR_PARAMETER 2

/**
 * @def AUTOVIZ_OGRE_NORMAL_PARAMETER
 * @brief Custom parameter index for common surface normal / facing direction.
 */
#define AUTOVIZ_OGRE_NORMAL_PARAMETER 3

/**
 * @def AUTOVIZ_OGRE_UP_PARAMETER
 * @brief Custom parameter index for common up vector (tiles / squares).
 */
#define AUTOVIZ_OGRE_UP_PARAMETER 4

/**
 * @def AUTOVIZ_OGRE_HIGHLIGHT_PARAMETER
 * @brief Custom parameter index for selection highlight tint.
 */
#define AUTOVIZ_OGRE_HIGHLIGHT_PARAMETER 5

/**
 * @def AUTOVIZ_OGRE_AUTO_SIZE_PARAMETER
 * @brief Custom parameter index enabling distance-based auto sizing.
 */
#define AUTOVIZ_OGRE_AUTO_SIZE_PARAMETER 6

