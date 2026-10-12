//
// Created by rgrandia on 24.04.20.
//

#pragma once

/**
 * @file foot_phase.hpp
 * @brief Linear inequality constraint A_p * p_world + b >=  0.
 */


#include "bolza/switched_model/core/switched_model.hpp"
#include "bolza/switched_model/foot_planner/swing_spline3d.hpp"
#include "bolza/switched_model/terrain/convex_terrain.hpp"
#include "bolza/switched_model/terrain/terrain_model.hpp"
#include "bolza/switched_model/terrain/terrain_plane.hpp"

namespace switched_model {

/**
 * Linear inequality constraint A_p * p_world + b >=  0
 */
struct FootTangentialConstraintMatrix {
  Eigen::Matrix<scalar_t, -1, 3> A;
  vector_t b;
};

/**
 * @brief Tangential Constraints From Convex Terrain.
 * @param [in] stanceTerrain: Stance Terrain.
 * @param [in] margin: Margin.
 * @return Result value.
 */
FootTangentialConstraintMatrix TangentialConstraintsFromConvexTerrain(const ConvexTerrain& stanceTerrain, scalar_t margin);

/**
 * Base class for a planned foot phase : Stance or Swing.
 */
class FootPhase {
 public:
  /**
   * @brief Constructs a FootPhase.
   */
  FootPhase() = default;
  /**
   * @brief Destructor.
   */
  virtual ~FootPhase() = default;
  /**
   * @brief Constructs a FootPhase.
   */
  FootPhase(const FootPhase&) = delete;
  FootPhase& operator=(const FootPhase&) = delete;

  /**
   * @brief Returns the contact flag for this phase. Stance phase: True, Swing phase: false.
   * @return True on success.
   */
  virtual bool ContactFlag() const = 0;

  /**
   * @brief Returns the unit vector pointing in the normal direction.
   * @param [in] time: Time.
   * @return Result value.
   */
  virtual vector3_t NormalDirectionInWorldFrame(scalar_t time) const = 0;

  /**
   * @brief Nominal foothold location (upcoming for swinglegs).
   * @return Result value.
   */
  virtual vector3_t NominalFootholdLocation() const = 0;

  /**
   * @brief Convex terrain that constrains the foot placement. (null for swinglegs).
   * @return Pointer; may be null.
   */
  virtual const ConvexTerrain* NominalFootholdConstraint() const { return nullptr; };

  /**
   * @brief Foot reference position in world frame.
   * @param [in] time: Time.
   * @return Result value.
   */
  virtual vector3_t GetPositionInWorld(scalar_t time) const = 0;

  /**
   * @brief Foot reference velocity in world frame.
   * @param [in] time: Time.
   * @return Result value.
   */
  virtual vector3_t GetVelocityInWorld(scalar_t time) const = 0;

  /**
   * @brief Foot reference acceleration in world frame.
   * @param [in] time: Time.
   * @return Result value.
   */
  virtual vector3_t GetAccelerationInWorld(scalar_t time) const = 0;

  /**
   * @brief Returns the position inequality constraints formulated in the tangential direction.
   * @return Pointer; may be null.
   */
  virtual const FootTangentialConstraintMatrix* GetFootTangentialConstraintInWorldFrame() const { return nullptr; };

  /**
   * @brief Get Minimum Foot Clearance.
   * @param [in] time: Time.
   * @return Result value.
   */
  virtual scalar_t GetMinimumFootClearance(scalar_t time) const { return 0.0; };
};

/**
 * Encodes a planned stance phase on a terrain plane.
 * The normal constraint makes the foot converge to the terrain plane when positionGain > 0.0
 */
class StancePhase final : public FootPhase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * @brief Constructs a StancePhase.
   * @param [in] stanceTerrain: Stance Terrain.
   * @param [in] terrainMargin: Terrain Margin.
   */
  explicit StancePhase(ConvexTerrain stanceTerrain, scalar_t terrainMargin = 0.0);
  /**
   * @brief Destructor.
   */
  ~StancePhase() override = default;

  /**
   * @brief Contact Flag.
   * @return True on success.
   */
  bool ContactFlag() const override { return true; };
  /**
   * @brief Normal Direction In World Frame.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t NormalDirectionInWorldFrame(scalar_t time) const override;
  /**
   * @brief Nominal Foothold Location.
   * @return Result value.
   */
  vector3_t NominalFootholdLocation() const override;
  /**
   * @brief Nominal Foothold Constraint.
   * @return Pointer; may be null.
   */
  const ConvexTerrain* NominalFootholdConstraint() const override;
  /**
   * @brief Get Position In World.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t GetPositionInWorld(scalar_t time) const override;
  /**
   * @brief Get Velocity In World.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t GetVelocityInWorld(scalar_t time) const override;
  /**
   * @brief Get Acceleration In World.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t GetAccelerationInWorld(scalar_t time) const override;
  /**
   * @brief Get Foot Tangential Constraint In World Frame.
   * @return Pointer; may be null.
   */
  const FootTangentialConstraintMatrix* GetFootTangentialConstraintInWorldFrame() const override;

 private:
  ConvexTerrain stanceTerrain_;
  const vector3_t nominalFootholdLocation_;
  const vector3_t surfaceNormalInWorldFrame_;
  const FootTangentialConstraintMatrix footTangentialConstraint_;
};

/**
 * Encodes a swing trajectory between two terrain planes.
 * A spline based swing motion is designed in both liftoff and target plane.
 */
class SwingPhase final : public FootPhase {
 public:
  struct SwingEvent {
    scalar_t time;
    scalar_t velocity;
    const TerrainPlane* terrainPlane;
  };

  struct SwingProfile {
    struct Node {
      /// Time progress in the swing phase in [0, 1]
      scalar_t phase = 0.5;
      /// Swing height in normal direction
      scalar_t swingHeight = 0.1;
      /// Velocity in normal direction
      scalar_t normalVelocity = 0.0;
      /// Swing progress in tangential direction in [0, 1]
      scalar_t tangentialProgress = 0.5;
      /// Tangantial velocity as a factor of the average velocity. The tangential velocity will be: velocityFactor * swingdistance / dt
      scalar_t tangentialVelocityFactor = 1.0;
    };

    /// Height / velocity profile
    std::vector<Node> nodes;
    /// Desired SDF clearance at the middle of the swing phase.
    scalar_t sdfMidswingMargin = 0.0;
    /// Desired SDF clearance at liftoff and touchdown. Slight negative margin allows a bit of ground penetration
    scalar_t sdfStartEndMargin = -0.02;
    /// Limits the amount of additional swing height from terrain adaptation
    scalar_t maxSwingHeightAdaptation = 0.3;
  };

  /**
   * @brief Construct a swing phase:.
   *
   * Creates a 3D swing reference motion
   *
   * Creates a 1D clearance profile for SDF based obstacle avoidance.
   *
   * object exists. Will extract SDF and obstacle information from the terrain.
   * @param [in] liftOff: Information about the liftoff event.
   * @param [in] touchDown: Information about the touchdown event.
   * @param [in] swingProfile: Swing Profile.
   * @param [in] terrainModel: (optional) Pointer to the terrain model. Terrain model should be kept alive externall as long as the swingphase
   */
  SwingPhase(SwingEvent liftOff, SwingEvent touchDown, const SwingProfile& swingProfile, const TerrainModel* terrainModel = nullptr);
  /**
   * @brief Destructor.
   */
  ~SwingPhase() override = default;

  /**
   * @brief Contact Flag.
   * @return True on success.
   */
  bool ContactFlag() const override { return false; };
  /**
   * @brief Normal Direction In World Frame.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t NormalDirectionInWorldFrame(scalar_t time) const override;
  /**
   * @brief Nominal Foothold Location.
   * @return Result value.
   */
  vector3_t NominalFootholdLocation() const override;
  /**
   * @brief Get Position In World.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t GetPositionInWorld(scalar_t time) const override;
  /**
   * @brief Get Velocity In World.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t GetVelocityInWorld(scalar_t time) const override;
  /**
   * @brief Get Acceleration In World.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t GetAccelerationInWorld(scalar_t time) const override;
  /**
   * @brief Get Minimum Foot Clearance.
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t GetMinimumFootClearance(scalar_t time) const override;

 private:
  /**
   * @brief Set Full Swing.
   * @param [in] swingProfile: Swing Profile.
   * @param [in] terrainModel: Terrain Model.
   */
  void SetFullSwing(const SwingProfile& swingProfile, const TerrainModel* terrainModel);
  /**
   * @brief Set Halve Swing.
   * @param [in] swingProfile: Swing Profile.
   * @param [in] terrainModel: Terrain Model.
   */
  void SetHalveSwing(const SwingProfile& swingProfile, const TerrainModel* terrainModel);

  /**
   * @brief Get Scaling.
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t GetScaling(scalar_t time) const;

  SwingEvent liftOff_;
  SwingEvent touchDown_;

  std::unique_ptr<SwingSpline3d> motion_;
  std::unique_ptr<QuinticSwing> terrainClearanceMotion_;
};

/**
 * Encodes a swing trajectory derived directly from the reference motion
 */
class ExternalSwingPhase final : public FootPhase {
 public:
  /**
   * @brief Constructs a ExternalSwingPhase.
   * @param [in] timeTrajectory: Time Trajectory.
   * @param [in] positionTrajectory: Position Trajectory.
   * @param [in] velocityTrajectory: Velocity Trajectory.
   */
  ExternalSwingPhase(std::vector<scalar_t> timeTrajectory, std::vector<vector3_t> positionTrajectory,
                     std::vector<vector3_t> velocityTrajectory);
  /**
   * @brief Destructor.
   */
  ~ExternalSwingPhase() override = default;

  /**
   * @brief Contact Flag.
   * @return True on success.
   */
  bool ContactFlag() const override { return false; };
  /**
   * @brief Normal Direction In World Frame.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t NormalDirectionInWorldFrame(scalar_t time) const override { return {0.0, 0.0, 1.0}; }
  /**
   * @brief Nominal Foothold Location.
   * @return Result value.
   */
  vector3_t NominalFootholdLocation() const override { return positionTrajectory_.back(); }
  /**
   * @brief Get Position In World.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t GetPositionInWorld(scalar_t time) const override;
  /**
   * @brief Get Velocity In World.
   * @param [in] time: Time.
   * @return Result value.
   */
  vector3_t GetVelocityInWorld(scalar_t time) const override;
  /**
   * @brief Get Acceleration In World.
   * @return Result value.
   */
  vector3_t GetAccelerationInWorld(scalar_t time) const override { return vector3_t::Zero(); }
  /**
   * @brief Get Minimum Foot Clearance.
   * @param [in] time: Time.
   * @return Result value.
   */
  scalar_t GetMinimumFootClearance(scalar_t time) const override { return 0.0; }

 private:
  std::vector<scalar_t> timeTrajectory_;
  std::vector<vector3_t> positionTrajectory_;
  std::vector<vector3_t> velocityTrajectory_;
};

}  // namespace switched_model
