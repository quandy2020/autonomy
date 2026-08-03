/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include <gtest/gtest.h>

#ifdef AUTOVIZ_USE_OGRE

#include <Ogre.h>

#include "autoviz/rendering/mesh_resource.hpp"
#include "autoviz/rendering/ogre_mesh_loader.hpp"
#include "tests/ogre_testing_environment.hpp"

namespace {

void registerTestPackages() {
  auto& resolver = autoviz::rendering::MeshResourceResolver::instance();
  resolver.clearPackagePaths();
  resolver.addPackageSharePath("autoviz_tests", AUTOVIZ_TEST_MESHES_DIR);
  resolver.addPackageSharePath("autoviz_ogre", AUTOVIZ_OGRE_MEDIA_DIR);
}

void assertVector3Near(const Ogre::Vector3& actual, const Ogre::Vector3& expected,
                       float tol = 1e-4f) {
  ASSERT_NEAR(actual.x(), expected.x(), tol);
  ASSERT_NEAR(actual.y(), expected.y(), tol);
  ASSERT_NEAR(actual.z(), expected.z(), tol);
}

void assertBoundingBoxNear(const Ogre::AxisAlignedBox& actual,
                           const Ogre::AxisAlignedBox& expected,
                           float tol = 1e-4f) {
  assertVector3Near(actual.getMaximum(), expected.getMaximum(), tol);
  assertVector3Near(actual.getMinimum(), expected.getMinimum(), tol);
}

bool loadMeshOk(const std::string& uri) {
  const Ogre::MeshPtr mesh =
      autoviz::rendering::OgreMeshLoader::loadMeshFromResource(uri);
  return mesh && !mesh.isNull();
}

}  // namespace

class MeshLoaderTestFixture : public ::testing::Test {
 protected:
  void SetUp() override {
    registerTestPackages();
    testing_environment_ =
        std::make_shared<autoviz::rendering::test::OgreTestingEnvironment>();
    testing_environment_->setUpOgreTestEnvironment();
    autoviz::rendering::OgreMeshLoader::ensurePrimitiveMeshes();
  }

  std::shared_ptr<autoviz::rendering::test::OgreTestingEnvironment>
      testing_environment_;
};

TEST_F(MeshLoaderTestFixture, throws_for_missing_files) {
  EXPECT_THROW(
      autoviz::rendering::OgreMeshLoader::loadMeshFromResource(
          "package://autoviz_tests/MISSING.stl"),
      autoviz::rendering::MeshResourceError);
}

TEST_F(MeshLoaderTestFixture, can_load_ogre_mesh_files) {
  const std::string mesh_path = "package://autoviz_ogre/models/rviz_sphere.mesh";
  const Ogre::MeshPtr mesh =
      autoviz::rendering::OgreMeshLoader::loadMeshFromResource(mesh_path);
  ASSERT_TRUE(mesh);
  EXPECT_TRUE(mesh->isManuallyLoaded());
  EXPECT_EQ(mesh_path, mesh->getName());
}

TEST_F(MeshLoaderTestFixture, can_load_stl_files) {
  const std::string mesh_path = "package://autoviz_tests/test_meshes/F2.stl";
  const Ogre::MeshPtr mesh =
      autoviz::rendering::OgreMeshLoader::loadMeshFromResource(mesh_path);
  ASSERT_TRUE(mesh);
  size_t vertex_count = 0;
  for (unsigned short i = 0; i < mesh->getNumSubMeshes(); ++i) {
    vertex_count += mesh->getSubMesh(i)->vertexData->vertexCount;
  }
  EXPECT_TRUE(mesh->isManuallyLoaded());
  EXPECT_EQ(mesh_path, mesh->getName());
  EXPECT_NEAR(34.920441f, mesh->getBoundingSphereRadius(), 0.01f);
  EXPECT_EQ(35532u, vertex_count);
}

TEST_F(MeshLoaderTestFixture, loading_invalid_short_stl_files_fail) {
  EXPECT_FALSE(loadMeshOk("package://autoviz_tests/test_meshes/invalid_short.stl"));
}

TEST_F(MeshLoaderTestFixture, loading_invalid_stl_files_fail) {
  EXPECT_FALSE(loadMeshOk("package://autoviz_tests/test_meshes/invalid.stl"));
}

TEST_F(MeshLoaderTestFixture, loading_invalid_ascii_stl_file) {
  EXPECT_FALSE(loadMeshOk("package://autoviz_tests/test_meshes/invalid_ascii.stl"));
}

TEST_F(MeshLoaderTestFixture, loading_invalid_stl_files_should_fail) {
  EXPECT_FALSE(
      loadMeshOk("package://autoviz_tests/test_meshes/16bit_vs_32bit_should_fail.stl"));
}

TEST_F(MeshLoaderTestFixture, loading_almost_invalid_stl_files_should_fail) {
  EXPECT_FALSE(loadMeshOk("package://autoviz_tests/test_meshes/invalid_extra.stl"));
}

TEST_F(MeshLoaderTestFixture, loading_stl_mesh_twice_should_not_fail) {
  const std::string mesh_path = "package://autoviz_tests/test_meshes/F2.stl";
  EXPECT_TRUE(loadMeshOk(mesh_path));
  EXPECT_TRUE(loadMeshOk(mesh_path));
}

TEST_F(MeshLoaderTestFixture, can_load_assimp_mesh_files) {
  const std::string mesh_path = "package://autoviz_tests/test_meshes/pr2-base.dae";
  const Ogre::MeshPtr mesh =
      autoviz::rendering::OgreMeshLoader::loadMeshFromResource(mesh_path);
  ASSERT_TRUE(mesh);
  const size_t expected_vertex_number = 3600;
  EXPECT_EQ(expected_vertex_number, mesh->getSubMesh(0)->vertexData->vertexCount);
  const Ogre::AxisAlignedBox expected_bounding_box(
      Ogre::Vector3(-0.342375f, -0.340043f, -0.00656401f),
      Ogre::Vector3(0.340425f, 0.340043f, 0.662748f));
  EXPECT_TRUE(mesh->isManuallyLoaded());
  EXPECT_EQ(mesh_path, mesh->getName());
  EXPECT_NEAR(0.754754f, mesh->getBoundingSphereRadius(), 0.01f);
  assertBoundingBoxNear(expected_bounding_box, mesh->getBounds(), 0.01f);
}

TEST_F(MeshLoaderTestFixture, assimp_loader_reads_size_correctly) {
  const std::string mesh_path = "package://autoviz_tests/test_meshes/pr2-base_large.dae";
  const Ogre::MeshPtr mesh =
      autoviz::rendering::OgreMeshLoader::loadMeshFromResource(mesh_path);
  ASSERT_TRUE(mesh);
  const float expected_bounding_radius = 6 * 0.754754f;
  const Ogre::AxisAlignedBox expected_bounding_box(
      Ogre::Vector3(6 * -0.342375f, 6 * -0.340043f, 6 * -0.00656401f),
      Ogre::Vector3(6 * 0.340425f, 6 * 0.340043f, 6 * 0.662748f));
  EXPECT_TRUE(mesh->isManuallyLoaded());
  EXPECT_EQ(mesh_path, mesh->getName());
  EXPECT_NEAR(expected_bounding_radius, mesh->getBoundingSphereRadius(), 0.05f);
  assertBoundingBoxNear(expected_bounding_box, mesh->getBounds(), 0.05f);
}

TEST_F(MeshLoaderTestFixture, loading_solidworks_binary_stl) {
  EXPECT_TRUE(loadMeshOk("package://autoviz_tests/test_meshes/solidworks.stl"));
}

TEST_F(MeshLoaderTestFixture, assimp_loader_reads_per_vertex_colors) {
  const std::string mesh_path = "package://autoviz_tests/test_meshes/colored_box.glb";
  const Ogre::MeshPtr mesh =
      autoviz::rendering::OgreMeshLoader::loadMeshFromResource(mesh_path);
  ASSERT_TRUE(mesh);
  ASSERT_GT(mesh->getNumSubMeshes(), 0u);
  const auto* declaration = mesh->getSubMesh(0)->vertexData->vertexDeclaration;
  EXPECT_NE(nullptr, declaration->findElementBySemantic(Ogre::VES_DIFFUSE));
}

TEST_F(MeshLoaderTestFixture, meshes_without_vertex_colors_have_no_diffuse_element) {
  const std::string mesh_path = "package://autoviz_tests/test_meshes/F2.stl";
  const Ogre::MeshPtr mesh =
      autoviz::rendering::OgreMeshLoader::loadMeshFromResource(mesh_path);
  ASSERT_TRUE(mesh);
  const auto* declaration = mesh->getSubMesh(0)->vertexData->vertexDeclaration;
  EXPECT_EQ(nullptr, declaration->findElementBySemantic(Ogre::VES_DIFFUSE));
}

#endif
