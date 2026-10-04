/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file map_plan.hpp
 * @brief Mission geometry for the map panel: waypoints, geofence, rally points.
 *
 * Generators cover a lawnmower survey inside a fence, a corridor along a
 * polyline, and a circular structure scan. Plans move as GeoJSON files.
 */

#pragma once

#include <QByteArray>
#include <QString>
#include <QVector>

#include "autoviz/ui/map/map_types.hpp"

namespace autoviz {
namespace map {

/**
 * @brief Lawn-mower waypoints that stay inside @p fence.
 *
 * @param fence Polygon vertices, at least 3.
 * @param spacing_m Distance between adjacent passes, in meters.
 * @return Empty when the fence is too small to scan.
 */
QVector<MapPlanPoint> SurveyLawnmower(const QVector<MapPlanPoint>& fence,
                                      double spacing_m);

/**
 * @brief Offset passes along a centerline.
 *
 * @param centerline At least two waypoints.
 * @param width_m Full corridor width.
 * @return Centerline plus a return pass offset to one side.
 */
QVector<MapPlanPoint> CorridorScan(const QVector<MapPlanPoint>& centerline,
                                   double width_m);

/**
 * @brief Closed ring of waypoints around a center.
 *
 * @param latitude Center latitude.
 * @param longitude Center longitude.
 * @param radius_m Ring radius in meters.
 * @param count Number of vertices, at least 3.
 */
QVector<MapPlanPoint> StructureScan(double latitude, double longitude,
                                    double radius_m, int count);

/**
 * @brief Serializes the three plan layers to a GeoJSON feature collection.
 */
QByteArray ExportPlanGeoJson(const QVector<MapPlanPoint>& waypoints,
                             const QVector<MapPlanPoint>& geofence,
                             const QVector<MapPlanPoint>& rally_points);

/**
 * @brief Reads plan layers from a GeoJSON feature collection.
 *
 * @param bytes File contents.
 * @param waypoints Output mission line. Cleared on success.
 * @param geofence Output polygon.
 * @param rally_points Output rally markers.
 * @param error Human-readable failure. May be null.
 * @return @c false when the document has no plan features.
 */
bool ImportPlanGeoJson(const QByteArray& bytes, QVector<MapPlanPoint>* waypoints,
                       QVector<MapPlanPoint>* geofence,
                       QVector<MapPlanPoint>* rally_points, QString* error);

}  // namespace map
}  // namespace autoviz
