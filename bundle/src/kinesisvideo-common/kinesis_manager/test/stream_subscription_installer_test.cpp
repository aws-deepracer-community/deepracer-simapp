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

#include <kinesis_manager/stream_subscription_installer.h>
#include <gtest/gtest.h>

using namespace Aws::Kinesis;

class TestStreamSubscriptionInstaller : public StreamSubscriptionInstaller
{
public:
  void Uninstall(const std::string & topic_name) override {}
  void PutInstaller(KinesisStreamInputType input_type, const SubscriberSetupFn& setup_fun) 
  {
    installers_.insert({input_type, setup_fun});
  }
};

TEST(StreamSubscriptionInstallerSuite, streamSubscriptionInstallerTest)
{
  TestStreamSubscriptionInstaller test_subject;

  KinesisStreamInputType input_type;
  StreamSubscriptionDescriptor descriptor{input_type, std::string("topic_name"),
    std::string("stream_name"), 10,
    std::string("rekognition_topic_name"), std::string("rekognition_data_stream")
  };

  StreamSubscriptionDescriptor incomplete_descriptor = descriptor;
  incomplete_descriptor.topic_name = "";
  auto status = test_subject.Install(incomplete_descriptor);
  EXPECT_EQ(KINESIS_MANAGER_STATUS_INVALID_INPUT, status);

  incomplete_descriptor = descriptor;
  incomplete_descriptor.stream_name = "";
  status = test_subject.Install(incomplete_descriptor);
  EXPECT_EQ(KINESIS_MANAGER_STATUS_INVALID_INPUT, status);

  status = test_subject.Install(descriptor);
  EXPECT_EQ(KINESIS_MANAGER_STATUS_SUBSCRIPTION_INSTALLER_NOT_FOUND, status);

  bool setup_called = false;
  SubscriberSetupFn setup_fun = [&setup_called](const StreamSubscriptionDescriptor & descriptor) { 
    setup_called = true;
    return true; 
  };
  test_subject.PutInstaller(descriptor.input_type, setup_fun);
  status = test_subject.Install(descriptor);
  EXPECT_EQ(KINESIS_MANAGER_STATUS_SUCCESS, status);
  EXPECT_TRUE(setup_called);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
