/*
 * Copyright 2026 The Openbot Authors
 *
 * Schema ID strings for StreamEnvelope.schema. Canonical message definitions
 * live in autonomy/orbisview/proto/render.proto (package autonomy.orbisview.render).
 * Frontend panels consume JSON payloads tagged with these IDs.
 */

#pragma once

namespace autonomy {
namespace orbisview {
namespace rendering {

/** Schema IDs used in StreamEnvelope.schema */
inline constexpr char kSchemaPose[] = "orbisview.render.Pose2D";
inline constexpr char kSchemaPath[] = "orbisview.render.Path2D";
inline constexpr char kSchemaOccupancyGrid[] = "orbisview.render.OccupancyGrid";
inline constexpr char kSchemaFootprint[] = "orbisview.render.RobotFootprint";
inline constexpr char kSchemaTfTree[] = "orbisview.render.TfTree";
inline constexpr char kSchemaLaserScan[] = "orbisview.render.LaserScan";
inline constexpr char kSchemaImage[] = "orbisview.render.Image";
inline constexpr char kSchemaPointCloud2[] = "orbisview.render.PointCloud2";
inline constexpr char kSchemaDepthImage[] = "orbisview.render.DepthImage";
inline constexpr char kSchemaExploration[] = "orbisview.render.Exploration";
inline constexpr char kSchemaNavigation[] = "orbisview.render.Navigation";
inline constexpr char kSchemaMapping[] = "orbisview.render.Mapping";
inline constexpr char kSchemaTwist2D[] = "orbisview.render.Twist2D";
inline constexpr char kSchemaChassis[] = "orbisview.render.ChassisState";
inline constexpr char kSchemaObstacles[] = "orbisview.render.ObstacleArray";
inline constexpr char kSchemaWorld[] = "orbisview.render.WorldState";
inline constexpr char kSchemaRoute[] = "orbisview.render.RoutePath";
inline constexpr char kSchemaVectorMap[] = "orbisview.render.VectorMap";
inline constexpr char kSchemaPrediction[] = "orbisview.render.PredictionObstacles";
inline constexpr char kSchemaPlanningDebug[] = "orbisview.render.PlanningDebug";
inline constexpr char kSchemaHmiStatus[] = "orbisview.render.HmiStatus";
inline constexpr char kSchemaComponents[] = "orbisview.render.ComponentsStatus";

}  // namespace rendering
}  // namespace orbisview
}  // namespace autonomy
