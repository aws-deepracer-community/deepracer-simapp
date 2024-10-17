/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

/**
 * NOTE:
 *  To exectue this test, you need roscore process to be running.
 *  This is because, the test uses a subscriber to recieve messages from /rosout.
 */

#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <gtest/gtest.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <unistd.h>

#include <iostream>

using namespace Aws::Utils::Logging;

typedef std::shared_ptr<AWSLogSystem> aws_logger_t;

std::atomic<int> message_count;

void CreateTestLog(aws_logger_t logger, const char * tag, const char * message,
                   Aws::Utils::Logging::LogLevel log_level)
{
  logger->Log(log_level, tag, message);
  message_count++;
}

TEST(AWSROSLoggerTest, TestLogMethod)
{
  aws_logger_t logger = std::make_shared<AWSROSLogger>(Aws::Utils::Logging::LogLevel::Error);

  // Log via AWSROSLogger logger class
  CreateTestLog(logger, "test_tag_1", "test_log_message_1", Aws::Utils::Logging::LogLevel::Error);
  usleep(3000000);  // sleep for some time before sending second log.
  CreateTestLog(logger, "test_tag_2", "test_log_message_2", Aws::Utils::Logging::LogLevel::Fatal);
}

void RosoutLoggerCallback(const rosgraph_msgs::Log & published_log)
{
  std::string expected_msg_1 = "[test_tag_1] test_log_message_1";
  std::string expected_msg_2 = "[test_tag_2] test_log_message_2";

  EXPECT_EQ(published_log.msg.compare(expected_msg_1) == 0 ||
              published_log.msg.compare(expected_msg_2) == 0,
            true);
  EXPECT_EQ(message_count > 0, true);

  message_count--;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_ros_logger");

  ros::NodeHandle n;

  // subscrive to /rosout where all the ROS_*** logs will be emitted.
  // The callback extracts the recieved log message verifies the contents.
  ros::Subscriber sub = n.subscribe("/rosout", 1000, RosoutLoggerCallback);
  int ret = RUN_ALL_TESTS();

  // spin till there are no events in events queue OR we have recieved all the expected logs.
  while (ros::ok()) {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
    if (message_count <= 0) {
      break;
    }
  }

  return ret;
}
