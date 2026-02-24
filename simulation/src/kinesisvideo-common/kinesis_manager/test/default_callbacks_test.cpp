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

#include <kinesis_manager/default_callbacks.h>
#include <gtest/gtest.h>

using namespace Aws::Kinesis;

TEST(ClientCallbackProviderSuite, defaultClientCallbackProviderTest)
{
  Aws::Kinesis::DefaultClientCallbackProvider test_subject;
  UINT64 custom_handle;
  UINT64 remaining_bytes;

  EXPECT_EQ(test_subject.storageOverflowPressure(custom_handle, remaining_bytes), 
    test_subject.getStorageOverflowPressureCallback()(custom_handle, remaining_bytes));
  EXPECT_EQ(STATUS_SUCCESS, 
    test_subject.storageOverflowPressure(custom_handle, remaining_bytes));
}

TEST(ClientCallbackProviderSuite, defaultStreamCallbackProviderTest)
{
  DefaultStreamCallbackProvider test_subject;
  UINT64 custom_data;
  STREAM_HANDLE stream_handle;
  UPLOAD_HANDLE upload_handle;
  UINT64 last_buffering_ack;
  UINT64 errored_timecode;
  STATUS status_code;
  UINT64 dropped_frame_timecode;

  EXPECT_EQ(STATUS_SUCCESS,
    test_subject.getStreamConnectionStaleCallback()(custom_data, stream_handle, last_buffering_ack));
  EXPECT_EQ(STATUS_SUCCESS,
    test_subject.getStreamErrorReportCallback()(custom_data, stream_handle, upload_handle, errored_timecode, status_code));
  EXPECT_EQ(STATUS_SUCCESS,
    test_subject.getDroppedFrameReportCallback()(custom_data, stream_handle, dropped_frame_timecode));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}