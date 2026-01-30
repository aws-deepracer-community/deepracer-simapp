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

#include <unistd.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <cstdio>

using namespace Aws::Utils::Logging;

using aws_logger_t = std::shared_ptr<AWSLogSystem>;

TEST(AWSROSLoggerTest, testStderr) {
    auto dummy_node = rclcpp::Node::make_shared("my_node_name");
    aws_logger_t logger = std::make_shared<AWSROSLogger>(Aws::Utils::Logging::LogLevel::Error, dummy_node);

    /* Dup stderr */
    int stderr_bk = dup(fileno(stderr));
    int pipe_fd[2];
    EXPECT_EQ(0, pipe2(pipe_fd, 0));
    EXPECT_NE(0, dup2(pipe_fd[1], fileno(stderr)));

    logger->Log(Aws::Utils::Logging::LogLevel::Error, "test", "msg");
    fflush(stderr);
    char stderr_buffer[BUFSIZ] = {0};
    read(pipe_fd[0], stderr_buffer, sizeof(stderr_buffer) - 1);

    /* Restore stderr */
    close(pipe_fd[1]);
    dup2(stderr_bk, fileno(stderr));

    ASSERT_TRUE(strstr(stderr_buffer, "[ERROR]") != nullptr && 
    strstr(stderr_buffer, "[my_node_name]") != nullptr && 
    strstr(stderr_buffer, "[test] msg") != nullptr);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
