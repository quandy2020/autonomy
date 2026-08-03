/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <gtest/gtest.h>

#ifdef AUTOVIZ_USE_OGRE

#include <array>

#include <Ogre.h>

#include "autoviz/rendering/objects/ogre_arrow.hpp"
#include "autoviz/rendering/objects/ogre_billboard_line.hpp"
#include "autoviz/rendering/objects/ogre_covariance_visual.hpp"
#include "autoviz/rendering/objects/ogre_line.hpp"
#include "autoviz/rendering/objects/ogre_mesh_shape.hpp"
#include "autoviz/rendering/objects/ogre_point_cloud.hpp"
#include "autoviz/rendering/objects/ogre_screw_visual.hpp"
#include "autoviz/rendering/objects/ogre_shape.hpp"
#include "autoviz/rendering/objects/ogre_wrench_visual.hpp"
#include "tests/ogre_testing_environment.hpp"

class SceneTestFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    env_ = std::make_shared<autoviz::rendering::test::OgreTestingEnvironment>();
    env_->setUpOgreTestEnvironment();
    scene_manager_ = Ogre::Root::getSingletonPtr()->createSceneManager();
    root_node_ = scene_manager_->getRootSceneNode();
  }

  void TearDown() override {
    if (scene_manager_ != nullptr) {
      Ogre::Root::getSingletonPtr()->destroySceneManager(scene_manager_);
      scene_manager_ = nullptr;
    }
  }

  std::shared_ptr<autoviz::rendering::test::OgreTestingEnvironment> env_;
  Ogre::SceneManager* scene_manager_ = nullptr;
  Ogre::SceneNode* root_node_ = nullptr;
};

TEST_F(SceneTestFixture, line_setPoints_sets_bounds) {
  auto* line = new autoviz::rendering::OgreLine(scene_manager_, root_node_);
  line->setPoints(Ogre::Vector3(-5, -5, -5), Ogre::Vector3(3, 3, 3));
  auto* line_node = dynamic_cast<Ogre::SceneNode*>(root_node_->getChild(0));
  ASSERT_NE(nullptr, line_node);
  const Ogre::AxisAlignedBox aabb = line_node->getAttachedObject(0)->getBoundingBox();
  EXPECT_NEAR(-5.f, aabb.getMinimum().x, 1e-3f);
  EXPECT_NEAR(3.f, aabb.getMaximum().x, 1e-3f);
  delete line;
}

TEST_F(SceneTestFixture, cube_entity_is_created) {
  auto* shape = new autoviz::rendering::OgreShape(autoviz::rendering::OgreShape::kCube,
                                               scene_manager_, root_node_);
  ASSERT_NE(nullptr, shape->entity());
  delete shape;
}

TEST_F(SceneTestFixture, arrow_has_shaft_and_head) {
  autoviz::rendering::OgreArrow arrow(scene_manager_, root_node_);
  ASSERT_NE(nullptr, arrow.shaft());
  ASSERT_NE(nullptr, arrow.head());
}

TEST_F(SceneTestFixture, wrench_visual_attaches_to_scene) {
  autoviz::rendering::OgreWrenchVisual wrench(scene_manager_, root_node_);
  wrench.setWrench(Ogre::Vector3(1, 0, 0), Ogre::Vector3(0, 0, 1));
  wrench.setVisible(true);
  EXPECT_GT(root_node_->numChildren(), 0u);
}

TEST_F(SceneTestFixture, screw_visual_attaches_to_scene) {
  autoviz::rendering::OgreScrewVisual screw(scene_manager_, root_node_);
  screw.setLinearScale(1.f);
  screw.setAngularScale(1.f);
  screw.setWidth(0.05f);
  screw.setScrew(Ogre::Vector3(1, 0, 0), Ogre::Vector3(0, 0, 1));
  screw.setVisible(true);
  EXPECT_GT(root_node_->numChildren(), 0u);
}

TEST_F(SceneTestFixture, mesh_shape_builds_triangle_entity) {
  autoviz::rendering::OgreMeshShape mesh(scene_manager_, root_node_);
  mesh.beginTriangles();
  mesh.addVertex(Ogre::Vector3(0, 0, 0), Ogre::Vector3::UNIT_Z);
  mesh.addVertex(Ogre::Vector3(1, 0, 0), Ogre::Vector3::UNIT_Z);
  mesh.addVertex(Ogre::Vector3(0, 1, 0), Ogre::Vector3::UNIT_Z);
  mesh.endTriangles();
  ASSERT_NE(nullptr, mesh.entity());
}

TEST_F(SceneTestFixture, covariance_visual_accepts_pose_covariance) {
  autoviz::rendering::OgreCovarianceVisual covariance(scene_manager_, root_node_);
  std::array<double, 36> cov{};
  cov[0] = 0.1;
  cov[7] = 0.1;
  cov[14] = 0.1;
  cov[21] = 0.01;
  cov[28] = 0.01;
  cov[35] = 0.01;
  covariance.setCovariance(Ogre::Quaternion::IDENTITY, cov);
  EXPECT_NE(nullptr, covariance.getPositionSceneNode());
}

class PointCloudTestFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    env_ = std::make_shared<autoviz::rendering::test::OgreTestingEnvironment>();
    env_->setUpOgreTestEnvironment();
  }

  std::shared_ptr<autoviz::rendering::test::OgreTestingEnvironment> env_;
};

TEST_F(PointCloudTestFixture, addPoints_creates_renderable) {
  auto cloud = std::make_shared<autoviz::rendering::OgrePointCloud>();
  std::vector<autoviz::rendering::OgrePointCloud::Point> points(1);
  points[*0].mutable_position() = Ogre::Vector3(1.f, 2.f, 3.f);
  points[*0].mutable_color() = Ogre::ColourValue::White;
  cloud->addPoints(points.begin(), points.end());
  EXPECT_FALSE(cloud->getRenderables().empty());
}

#endif
