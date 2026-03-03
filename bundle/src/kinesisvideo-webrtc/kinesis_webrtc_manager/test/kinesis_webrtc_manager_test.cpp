/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <aws/core/Aws.h>
#include <kinesis_webrtc_manager/common.h>
#include <kinesis_webrtc_manager/kinesis_webrtc_client_interface.h>
#define protected public
#define private   public
#include <kinesis_webrtc_manager/kinesis_webrtc_manager.h>
#undef protected
#undef private

using namespace Aws::Kinesis;
using namespace testing;
using ::testing::Return;

class MockKinesisWebRtcClientInterface : public KinesisWebRtcClientInterface
{
public:
  MockKinesisWebRtcClientInterface() = default;
  MOCK_CONST_METHOD5(CreateSignalingClientSync,
    STATUS(PSignalingClientInfo p_signaling_client_info,
          PChannelInfo p_channel_info,
          PSignalingClientCallbacks p_signaling_client_callbacks,
          PAwsCredentialProvider p_aws_credential_provider,
          PSIGNALING_CLIENT_HANDLE p_signaling_client_handle)
  );

  MOCK_CONST_METHOD3(DataChannelOnMessage,
    STATUS(PRtcDataChannel p_rtc_data_channel, UINT64 custom_data, RtcOnMessage rtc_on_message));

  MOCK_CONST_METHOD3(DataChannelOnOpen,
    STATUS(PRtcDataChannel p_rtc_data_channel, UINT64 custom_data, RtcOnOpen rtc_on_open));

  MOCK_CONST_METHOD4(DataChannelSend,
    STATUS(PRtcDataChannel p_rtc_data_channel, BOOL is_binary, PBYTE data, UINT32 length));

  MOCK_CONST_METHOD1(FreeSignalingClient, STATUS(PSIGNALING_CLIENT_HANDLE p_signaling_client_handle));

  MOCK_CONST_METHOD0(InitKvsWebRtc, STATUS());

  MOCK_CONST_METHOD1(SignalingClientConnectSync, STATUS(SIGNALING_CLIENT_HANDLE signaling_client_handle));

  MOCK_CONST_METHOD2(WriteFrame, STATUS(PRtcRtpTransceiver p_rtc_rtp_transceiver, PFrame p_frame));
};

class KinesisWebRtcManagerTestFixture : public ::testing::Test
{
protected:
  std::shared_ptr<KinesisWebRtcManager> webrtc_manager_;
  std::vector<WebRtcStreamInfo> stream_infos_;
  std::shared_ptr<MockKinesisWebRtcClientInterface> mock_webrtc_client_interface_;
  const std::string test_signaling_channel_name_ = "test_signaling_channel";
  const std::string empty_signaling_channel_name_ = "";
  const std::string nonexistent_signaling_channel_name_ = "nonexistent_signaling_channel_name";
  const std::string test_custom_data_ = "test_custom_data";
  const VideoFormatType test_video_format_type_ = VideoFormatType::H264;
  const VideoFormatType invalid_video_format_type_ = static_cast<VideoFormatType>(-1);
  Frame * test_frame_;
  const std::string test_data_message_ = "test_data_message";
  SIGNALING_CLIENT_HANDLE test_signaling_client_handle_ = static_cast<UINT64>(0x1000);

  void SetUp() override
  {
    mock_webrtc_client_interface_ = std::make_shared<MockKinesisWebRtcClientInterface>();
    webrtc_manager_ = std::make_shared<KinesisWebRtcManager>(mock_webrtc_client_interface_);
    WebRtcStreamInfo webrtc_stream_info = {
      // Storing the address of the test_custom_data_ as WebRtcStreamInfo::CustomData
      reinterpret_cast<UINT64>(&test_custom_data_),
      test_signaling_channel_name_,
      test_video_format_type_,
      nullptr
    };
    stream_infos_.push_back(webrtc_stream_info);
  }
  
  void TearDown() override
  {
    webrtc_manager_.reset();
    mock_webrtc_client_interface_.reset();
  }
};

TEST_F(KinesisWebRtcManagerTestFixture, InitializeWebRtcSuccess)
{
  EXPECT_CALL(*mock_webrtc_client_interface_, InitKvsWebRtc())
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));

  EXPECT_CALL(*mock_webrtc_client_interface_, CreateSignalingClientSync(_,_,_,_,_))
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));

  EXPECT_CALL(*mock_webrtc_client_interface_, SignalingClientConnectSync(_))
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));
  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->InitializeWebRtc(stream_infos_);
  EXPECT_TRUE(KinesisWebRtcManagerStatusSucceeded(ret_status));

  // On successful iterations, expecting FreeSignalingClient on destruction
  EXPECT_CALL(*mock_webrtc_client_interface_, FreeSignalingClient(_))
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));
}

TEST_F(KinesisWebRtcManagerTestFixture, InitializeWebRtcEmptyStreamInfos)
{
  stream_infos_.clear();
  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->InitializeWebRtc(stream_infos_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));
}

TEST_F(KinesisWebRtcManagerTestFixture, InitializeWebRtcDoubleInitialization)
{
  EXPECT_CALL(*mock_webrtc_client_interface_, InitKvsWebRtc())
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));

  EXPECT_CALL(*mock_webrtc_client_interface_, CreateSignalingClientSync(_,_,_,_,_))
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));

  EXPECT_CALL(*mock_webrtc_client_interface_, SignalingClientConnectSync(_))
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));
  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->InitializeWebRtc(stream_infos_);
  EXPECT_TRUE(KinesisWebRtcManagerStatusSucceeded(ret_status));
  
  EXPECT_CALL(*mock_webrtc_client_interface_, InitKvsWebRtc())
    .Times(0);

  EXPECT_CALL(*mock_webrtc_client_interface_, CreateSignalingClientSync(_,_,_,_,_))
    .Times(0);

  EXPECT_CALL(*mock_webrtc_client_interface_, SignalingClientConnectSync(_))
    .Times(0);

  // should fail second initialization
  ret_status = webrtc_manager_->InitializeWebRtc(stream_infos_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));

  // On successful iterations, expecting FreeSignalingClient on destruction
  EXPECT_CALL(*mock_webrtc_client_interface_, FreeSignalingClient(_))
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));
}

TEST_F(KinesisWebRtcManagerTestFixture, InitializeWebRtcEmptySignalingChannelName)
{
  stream_infos_.clear();
  WebRtcStreamInfo stream_info = {
    // Storing the address of the test_custom_data_ as WebRtcStreamInfo::CustomData
    reinterpret_cast<UINT64>(&test_custom_data_),
    empty_signaling_channel_name_,
    test_video_format_type_,
    nullptr
  };
  stream_infos_.push_back(stream_info);

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->InitializeWebRtc(stream_infos_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));
}

TEST_F(KinesisWebRtcManagerTestFixture, InitializeWebRtcDuplicateSignalingChannelName)
{
  webrtc_manager_->kvs_webrtc_configurations_.insert({test_signaling_channel_name_, nullptr}); 

  EXPECT_CALL(*mock_webrtc_client_interface_, InitKvsWebRtc())
    .Times(0);

  EXPECT_CALL(*mock_webrtc_client_interface_, CreateSignalingClientSync(_,_,_,_,_))
    .Times(0);

  EXPECT_CALL(*mock_webrtc_client_interface_, SignalingClientConnectSync(_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->InitializeWebRtc(stream_infos_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));
}

TEST_F(KinesisWebRtcManagerTestFixture, InitializeWebRtcInvalidVideoFormatType)
{
  stream_infos_.clear();
  WebRtcStreamInfo stream_info = {
    // Storing the address of the test_custom_data_ as WebRtcStreamInfo::CustomData
    reinterpret_cast<UINT64>(&test_custom_data_),
    test_signaling_channel_name_,
    invalid_video_format_type_,
    nullptr
  };
  stream_infos_.push_back(stream_info);

  EXPECT_CALL(*mock_webrtc_client_interface_, InitKvsWebRtc())
    .Times(0);

  EXPECT_CALL(*mock_webrtc_client_interface_, CreateSignalingClientSync(_,_,_,_,_))
    .Times(0);

  EXPECT_CALL(*mock_webrtc_client_interface_, SignalingClientConnectSync(_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->InitializeWebRtc(stream_infos_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));
}

TEST_F(KinesisWebRtcManagerTestFixture, InitializeWebRtcInitKvsWebRtcError)
{
  EXPECT_CALL(*mock_webrtc_client_interface_, InitKvsWebRtc())
    .Times(1)
    .WillOnce(Return(STATUS_NOT_FOUND));

  EXPECT_CALL(*mock_webrtc_client_interface_, CreateSignalingClientSync(_,_,_,_,_))
    .Times(0);

  EXPECT_CALL(*mock_webrtc_client_interface_, SignalingClientConnectSync(_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->InitializeWebRtc(stream_infos_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));
}

TEST_F(KinesisWebRtcManagerTestFixture, InitializeWebRtcCreateSignalingClientSyncError)
{
  EXPECT_CALL(*mock_webrtc_client_interface_, InitKvsWebRtc())
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));

  EXPECT_CALL(*mock_webrtc_client_interface_, CreateSignalingClientSync(_,_,_,_,_))
    .Times(1)
    .WillOnce(Return(STATUS_NOT_FOUND));

  EXPECT_CALL(*mock_webrtc_client_interface_, SignalingClientConnectSync(_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->InitializeWebRtc(stream_infos_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));
}

TEST_F(KinesisWebRtcManagerTestFixture, InitializeWebRtcSignalingClientConnectSyncError)
{
  EXPECT_CALL(*mock_webrtc_client_interface_, InitKvsWebRtc())
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));

  EXPECT_CALL(*mock_webrtc_client_interface_, CreateSignalingClientSync(_,_,_,_,_))
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));

  EXPECT_CALL(*mock_webrtc_client_interface_, SignalingClientConnectSync(_))
    .Times(1)
    .WillOnce(Return(STATUS_NOT_FOUND));

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->InitializeWebRtc(stream_infos_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));
}

TEST_F(KinesisWebRtcManagerTestFixture, PutFrameNonexistentSignalingChannel)
{
  EXPECT_CALL(*mock_webrtc_client_interface_, WriteFrame(_,_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->PutFrame(nonexistent_signaling_channel_name_, test_frame_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));
}

TEST_F(KinesisWebRtcManagerTestFixture, PutFrameMissingKvsWebRtcConfig)
{
  webrtc_manager_->kvs_webrtc_configurations_.insert({test_signaling_channel_name_, nullptr});
  
  EXPECT_CALL(*mock_webrtc_client_interface_, WriteFrame(_,_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->PutFrame(test_signaling_channel_name_, test_frame_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));
}

TEST_F(KinesisWebRtcManagerTestFixture, PutFrameNoStreamingSessions)
{
  KvsWebRtcConfiguration test_kvs_webrtc_configuration;
  test_kvs_webrtc_configuration.streamingSessionCount = 0;
  webrtc_manager_->kvs_webrtc_configurations_.insert({test_signaling_channel_name_, &test_kvs_webrtc_configuration});
  
  EXPECT_CALL(*mock_webrtc_client_interface_, WriteFrame(_,_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->PutFrame(test_signaling_channel_name_, test_frame_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));

  // Remove configuration manually
  webrtc_manager_->kvs_webrtc_configurations_.erase(test_signaling_channel_name_);
}

TEST_F(KinesisWebRtcManagerTestFixture, PutFrameUpdatingStreamingSessionList)
{
  KvsWebRtcConfiguration test_kvs_webrtc_configuration;
  test_kvs_webrtc_configuration.streamingSessionCount = 1;
  test_kvs_webrtc_configuration.updatingStreamingSessionList = true;
  webrtc_manager_->kvs_webrtc_configurations_.insert({test_signaling_channel_name_, &test_kvs_webrtc_configuration});
  
  EXPECT_CALL(*mock_webrtc_client_interface_, WriteFrame(_,_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->PutFrame(test_signaling_channel_name_, test_frame_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));

  // Remove configuration manually
  webrtc_manager_->kvs_webrtc_configurations_.erase(test_signaling_channel_name_);
}

TEST_F(KinesisWebRtcManagerTestFixture, PutFrameWriteFrameFailure)
{
  KvsWebRtcConfiguration test_kvs_webrtc_configuration;
  test_kvs_webrtc_configuration.streamingSessionCount = 1;
  test_kvs_webrtc_configuration.updatingStreamingSessionList = false;
  KvsWebRtcStreamingSession test_kvs_webrtc_streaming_session;
  test_kvs_webrtc_configuration.streamingSessionList[0] = &test_kvs_webrtc_streaming_session;
  test_kvs_webrtc_configuration.streamingSessionList[0]->pVideoRtcRtpTransceiver = nullptr;
  webrtc_manager_->kvs_webrtc_configurations_.insert({test_signaling_channel_name_, &test_kvs_webrtc_configuration});
  
  EXPECT_CALL(*mock_webrtc_client_interface_, WriteFrame(_,_))
    .Times(1)
    .WillOnce(Return(STATUS_NOT_FOUND));

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->PutFrame(test_signaling_channel_name_, test_frame_);
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));

  // Remove configuration manually
  webrtc_manager_->kvs_webrtc_configurations_.erase(test_signaling_channel_name_);
}

TEST_F(KinesisWebRtcManagerTestFixture, PutFrameSuccess)
{
  KvsWebRtcConfiguration test_kvs_webrtc_configuration;
  test_kvs_webrtc_configuration.streamingSessionCount = 1;
  test_kvs_webrtc_configuration.updatingStreamingSessionList = false;
  KvsWebRtcStreamingSession test_kvs_webrtc_streaming_session;
  test_kvs_webrtc_configuration.streamingSessionList[0] = &test_kvs_webrtc_streaming_session;
  test_kvs_webrtc_configuration.streamingSessionList[0]->pVideoRtcRtpTransceiver = nullptr;
  webrtc_manager_->kvs_webrtc_configurations_.insert({test_signaling_channel_name_, &test_kvs_webrtc_configuration});
  
  EXPECT_CALL(*mock_webrtc_client_interface_, WriteFrame(_,_))
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));

  KinesisWebRtcManagerStatus ret_status = webrtc_manager_->PutFrame(test_signaling_channel_name_, test_frame_);
  EXPECT_TRUE(KinesisWebRtcManagerStatusSucceeded(ret_status));

  // Remove configuration manually
  webrtc_manager_->kvs_webrtc_configurations_.erase(test_signaling_channel_name_);
}

TEST_F(KinesisWebRtcManagerTestFixture, SendDataMessageNonexistentSignalingChannel)
{
  EXPECT_CALL(*mock_webrtc_client_interface_, DataChannelSend(_,_,_,_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status
      = webrtc_manager_->SendDataMessage(test_signaling_channel_name_, true,
        reinterpret_cast<PBYTE>(const_cast<char *>(test_data_message_.c_str())), test_data_message_.length());
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));
}

TEST_F(KinesisWebRtcManagerTestFixture, SendDataMessageMissingKvsWebRtcConfig)
{
  webrtc_manager_->kvs_webrtc_configurations_.insert({test_signaling_channel_name_, nullptr});
  
  EXPECT_CALL(*mock_webrtc_client_interface_, DataChannelSend(_,_,_,_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status
      = webrtc_manager_->SendDataMessage(test_signaling_channel_name_, true,
        reinterpret_cast<PBYTE>(const_cast<char *>(test_data_message_.c_str())), test_data_message_.length());
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));
}

TEST_F(KinesisWebRtcManagerTestFixture, SendDataMessageNoStreamingSessions)
{
  KvsWebRtcConfiguration test_kvs_webrtc_configuration;
  test_kvs_webrtc_configuration.streamingSessionCount = 0;
  webrtc_manager_->kvs_webrtc_configurations_.insert({test_signaling_channel_name_, &test_kvs_webrtc_configuration});
  
  EXPECT_CALL(*mock_webrtc_client_interface_, DataChannelSend(_,_,_,_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status
      = webrtc_manager_->SendDataMessage(test_signaling_channel_name_, true,
        reinterpret_cast<PBYTE>(const_cast<char *>(test_data_message_.c_str())), test_data_message_.length());
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));

  // Remove configuration manually
  webrtc_manager_->kvs_webrtc_configurations_.erase(test_signaling_channel_name_);
}

TEST_F(KinesisWebRtcManagerTestFixture, SendDataMessageUpdatingStreamingSessionList)
{
  KvsWebRtcConfiguration test_kvs_webrtc_configuration;
  test_kvs_webrtc_configuration.streamingSessionCount = 1;
  test_kvs_webrtc_configuration.updatingStreamingSessionList = true;
  webrtc_manager_->kvs_webrtc_configurations_.insert({test_signaling_channel_name_, &test_kvs_webrtc_configuration});
  
  EXPECT_CALL(*mock_webrtc_client_interface_, DataChannelSend(_,_,_,_))
    .Times(0);

  KinesisWebRtcManagerStatus ret_status
      = webrtc_manager_->SendDataMessage(test_signaling_channel_name_, true,
        reinterpret_cast<PBYTE>(const_cast<char *>(test_data_message_.c_str())), test_data_message_.length());
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));

  // Remove configuration manually
  webrtc_manager_->kvs_webrtc_configurations_.erase(test_signaling_channel_name_);
}

TEST_F(KinesisWebRtcManagerTestFixture, SendDataMessageWriteFrameFailure)
{
  KvsWebRtcConfiguration test_kvs_webrtc_configuration;
  test_kvs_webrtc_configuration.streamingSessionCount = 1;
  test_kvs_webrtc_configuration.updatingStreamingSessionList = false;
  KvsWebRtcStreamingSession test_kvs_webrtc_streaming_session;
  test_kvs_webrtc_configuration.streamingSessionList[0] = &test_kvs_webrtc_streaming_session;
  test_kvs_webrtc_configuration.streamingSessionList[0]->pRtcDataChannel = nullptr;
  webrtc_manager_->kvs_webrtc_configurations_.insert({test_signaling_channel_name_, &test_kvs_webrtc_configuration});
  
  EXPECT_CALL(*mock_webrtc_client_interface_, DataChannelSend(_,_,_,_))
    .Times(1)
    .WillOnce(Return(STATUS_NOT_FOUND));

  KinesisWebRtcManagerStatus ret_status
      = webrtc_manager_->SendDataMessage(test_signaling_channel_name_, true,
        reinterpret_cast<PBYTE>(const_cast<char *>(test_data_message_.c_str())), test_data_message_.length());
  EXPECT_FALSE(KinesisWebRtcManagerStatusSucceeded(ret_status));

  // Remove configuration manually
  webrtc_manager_->kvs_webrtc_configurations_.erase(test_signaling_channel_name_);
}

TEST_F(KinesisWebRtcManagerTestFixture, SendDataMessageSuccess)
{
  KvsWebRtcConfiguration test_kvs_webrtc_configuration;
  test_kvs_webrtc_configuration.streamingSessionCount = 1;
  test_kvs_webrtc_configuration.updatingStreamingSessionList = false;
  KvsWebRtcStreamingSession test_kvs_webrtc_streaming_session;
  test_kvs_webrtc_configuration.streamingSessionList[0] = &test_kvs_webrtc_streaming_session;
  test_kvs_webrtc_configuration.streamingSessionList[0]->pRtcDataChannel = nullptr;
  webrtc_manager_->kvs_webrtc_configurations_.insert({test_signaling_channel_name_, &test_kvs_webrtc_configuration});
  
  EXPECT_CALL(*mock_webrtc_client_interface_, DataChannelSend(_,_,_,_))
    .Times(1)
    .WillOnce(Return(STATUS_SUCCESS));

    KinesisWebRtcManagerStatus ret_status
      = webrtc_manager_->SendDataMessage(test_signaling_channel_name_, true,
        reinterpret_cast<PBYTE>(const_cast<char *>(test_data_message_.c_str())), test_data_message_.length());
  EXPECT_TRUE(KinesisWebRtcManagerStatusSucceeded(ret_status));

  // Remove configuration manually
  webrtc_manager_->kvs_webrtc_configurations_.erase(test_signaling_channel_name_);
}

TEST_F(KinesisWebRtcManagerTestFixture, OnDataChannelOpenReceive)
{
  KvsWebRtcConfiguration kvs_webrtc_configuration;
  // Passing the address of test_custom_data_ as UINT64 CustomData
  webrtc_manager_->OnDataChannelOpenReceive(reinterpret_cast<UINT64>(&kvs_webrtc_configuration), nullptr);
}

TEST_F(KinesisWebRtcManagerTestFixture, OnDataChannelOpenSend)
{
  // Passing the address of test_custom_data_ as UINT64 CustomData
  webrtc_manager_->OnDataChannelOpenSend(reinterpret_cast<UINT64>(&test_custom_data_), nullptr);
}

TEST_F(KinesisWebRtcManagerTestFixture, DestructorFreeSignalingClientFailure)
{
  webrtc_manager_->InitializeWebRtc(stream_infos_);

  EXPECT_CALL(*mock_webrtc_client_interface_, FreeSignalingClient(_))
    .Times(1)
    .WillOnce(Return(STATUS_NOT_FOUND));
}

int main(int argc, char ** argv)
{
  Aws::SDKOptions options;
  Aws::InitAPI(options);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
