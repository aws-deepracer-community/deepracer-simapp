/*
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include <gtest/gtest.h>
#include <aws_common/fs_utils/wordexp_ros.h>
#include <stdlib.h>

class WordExpRosTestFixture : public ::testing::Test
{
protected:
  void SetUp() override {
    env_home = getenv("HOME");
    env_ros_home = getenv("ROS_HOME");
  }

  void TearDown() override {
    wordfree(&wordexp_standard);
    wordfree(&wordexp_ros_expansion);
    // Restore environment variables
    if (NULL != env_home) {
      setenv("HOME", env_home, 1);
    }
    if (NULL != env_ros_home) {
      setenv("ROS_HOME", env_ros_home, 1);
    }
  }
  wordexp_t wordexp_standard, wordexp_ros_expansion;
  const char *env_home, *env_ros_home;
};

/*  Test Scenarios

    HOME  | ROS_HOME | result:
 1. unset |  unset   | same as wordexp (home directory of pwd entry)
 2. set   |  unset   | same as wordexp ($HOME)
 3. set   |  set     | same as wordexp ($HOME)
 4. unset |  set     | ROS_HOME
**/

TEST_F(WordExpRosTestFixture, TestHomeUnsetRosHomeUnset)
{
  // Scenario 1
  unsetenv("HOME");
  unsetenv("ROS_HOME");
  EXPECT_EQ(wordexp_ros("~", &wordexp_ros_expansion, 0), wordexp("~", &wordexp_standard, 0));
  EXPECT_EQ(wordexp_ros_expansion.we_wordc, wordexp_standard.we_wordc);
  EXPECT_STREQ(wordexp_ros_expansion.we_wordv[0], wordexp_standard.we_wordv[0]);
}

TEST_F(WordExpRosTestFixture, TestHomeSetRosHomeUnset)
{
  // Scenario 2
  setenv("HOME", "/home/test2", true);
  unsetenv("ROS_HOME");
  EXPECT_EQ(wordexp_ros("~", &wordexp_ros_expansion, 0), wordexp("~", &wordexp_standard, 0));
  EXPECT_EQ(wordexp_ros_expansion.we_wordc, wordexp_standard.we_wordc);
  EXPECT_STREQ(wordexp_ros_expansion.we_wordv[0], wordexp_standard.we_wordv[0]);
  EXPECT_STREQ(wordexp_ros_expansion.we_wordv[0], "/home/test2");
}

TEST_F(WordExpRosTestFixture, TestHomeSetRosHomeSet)
{
  // Scenario 3
  setenv("HOME", "/home/test3", true);
  setenv("ROS_HOME", "/home/roshome3", true);
  EXPECT_EQ(wordexp_ros("~", &wordexp_ros_expansion, 0), wordexp("~", &wordexp_standard, 0));
  EXPECT_EQ(wordexp_ros_expansion.we_wordc, wordexp_standard.we_wordc);
  EXPECT_STREQ(wordexp_ros_expansion.we_wordv[0], wordexp_standard.we_wordv[0]);
  EXPECT_STREQ(wordexp_ros_expansion.we_wordv[0], "/home/test3");
}

TEST_F(WordExpRosTestFixture, TestHomeUnsetRosHomeSet)
{
  // Scenario 4
  unsetenv("HOME");
  setenv("ROS_HOME", "/home/roshome4", true);
  EXPECT_EQ(wordexp_ros("~", &wordexp_ros_expansion, 0), wordexp("~", &wordexp_standard, 0));
  EXPECT_STRNE(wordexp_ros_expansion.we_wordv[0], wordexp_standard.we_wordv[0]);
  EXPECT_STREQ(wordexp_ros_expansion.we_wordv[0], "/home/roshome4");
  // Verify that HOME remains unchanged
  EXPECT_EQ(getenv("HOME"), nullptr);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
