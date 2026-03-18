/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>

#include <string>

#include "autonomy/transform/urdf/model.hpp"

int g_argc;
char** g_argv;

namespace autonomy {
namespace transform {
namespace {

class TestParser : public testing::Test {
 public:
  bool checkModel(autonomy::transform::Model& robot) {
    // get root link
    ::urdf::LinkConstSharedPtr root_link = robot.getRoot();
    if (!root_link) {
      fprintf(stderr, "no root link %s\n", robot.getName().c_str());
      return false;
    }

    // go through entire tree
    return this->traverse_tree(root_link);
  }

 protected:
  /// constructor
  // num_links starts at 1 because traverse_tree doesn't count the root node
  TestParser() : num_joints(0), num_links(1) {}

  /// Destructor
  ~TestParser() {}

  bool traverse_tree(::urdf::LinkConstSharedPtr link, int level = 0) {
    fprintf(stderr, "Traversing tree at level %d, link size %lu\n", level, link->child_links.size());
    level += 2;
    bool retval = true;
    for (auto child = link->child_links.begin(); child != link->child_links.end(); child++) {
      ++num_links;
      if (*child && (*child)->parent_joint) {
        ++num_joints;
        // check rpy
        double roll, pitch, yaw;
        (*child)->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);

        if (std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw)) {
          fprintf(stderr, "getRPY() returned nan!\n");
          return false;
        }
        // recurse down the tree
        retval &= this->traverse_tree(*child, level);
      } else {
        fprintf(stderr, "root link: %s has a null child!\n", link->name.c_str());
        return false;
      }
    }
    // no more children
    return retval;
  }

  size_t num_joints;
  size_t num_links;
};

TEST_F(TestParser, test) {
  ASSERT_GE(g_argc, 3);
  std::string folder = std::string(g_argv[1]) + "/test/";
  fprintf(stderr, "Folder %s\n", folder.c_str());
  std::string file = std::string(g_argv[2]);
  bool expect_success = (file.substr(0, 5) != "fail_");
  autonomy::transform::Model robot;
  fprintf(stderr, "Parsing file %s, expecting %d\n", (folder + file).c_str(), expect_success);
  if (!expect_success) {
    ASSERT_FALSE(robot.initFile(folder + file));
    return;
  }

  ASSERT_EQ(g_argc, 7);
  std::string robot_name = std::string(g_argv[3]);
  std::string root_name = std::string(g_argv[4]);
  size_t expected_num_joints = atoi(g_argv[5]);
  size_t expected_num_links = atoi(g_argv[6]);

  ASSERT_TRUE(robot.initFile(folder + file));

  EXPECT_EQ(robot.getName(), robot_name);
  ::urdf::LinkConstSharedPtr root = robot.getRoot();
  ASSERT_TRUE(static_cast<bool>(root));
  EXPECT_EQ(root->name, root_name);

  ASSERT_TRUE(checkModel(robot));
  EXPECT_EQ(num_joints, expected_num_joints);
  EXPECT_EQ(num_links, expected_num_links);
  EXPECT_EQ(robot.joints_.size(), expected_num_joints);
  EXPECT_EQ(robot.links_.size(), expected_num_links);

  // test reading from parameter server
  // Note: initParam method is not implemented in this version
  // ASSERT_TRUE(robot.initParam("robot_description"));
  // ASSERT_FALSE(robot.initParam("robot_description_wim"));
  SUCCEED();
}

}  // namespace
}  // namespace transform
}  // namespace autonomy

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}
