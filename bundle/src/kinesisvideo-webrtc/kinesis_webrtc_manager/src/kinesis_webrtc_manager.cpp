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
#include "kinesis_webrtc_manager/kinesis_webrtc_manager.h"

#include <aws/core/client/AWSError.h>
#include <aws/core/utils/logging/LogMacros.h>

// Wrapper function to execute clean up for webrtc peers using pthread
void * SessionCleanupWait(void *param) {
  PKvsWebRtcConfiguration config = (PKvsWebRtcConfiguration) param;
  STATUS ret = sessionCleanupWait(config);
  return NULL;
}

namespace Aws {
namespace Kinesis {

KinesisWebRtcManager::~KinesisWebRtcManager()
{
  STATUS status = STATUS_SUCCESS;
  // Join the cleanup thread
  status = pthread_join(cleanup_thread_id, NULL);
  if (status != 0) {
    AWS_LOG_ERROR(__func__, "Pthread join failed.");
  }
  for (const auto &entry : kvs_webrtc_configurations_) {
    PKvsWebRtcConfiguration p_kvs_webrtc_configuration = entry.second;

    if (p_kvs_webrtc_configuration != nullptr) {
      ATOMIC_STORE_BOOL(&p_kvs_webrtc_configuration->appTerminateFlag, TRUE);

      status = webrtc_client_->FreeSignalingClient(&p_kvs_webrtc_configuration->signalingClientHandle);
      if (status != STATUS_SUCCESS) {
        AWS_LOG_ERROR(__func__, "freeSignalingClient(): operation returned status code: 0x%08x", status);
      }

      status = freeKvsWebRtcConfiguration(&p_kvs_webrtc_configuration);
      if (status != STATUS_SUCCESS) {
        AWS_LOG_ERROR(__func__, "freeKvsWebRtcConfiguration(): operation returned status code: 0x%08x", status);
      }
    }
  }
}

KinesisWebRtcManagerStatus KinesisWebRtcManager::InitializeWebRtc(
  const std::vector<WebRtcStreamInfo> & stream_infos)
{
  if (kvs_webrtc_configurations_.size() > 0) {
    AWS_LOG_ERROR(__func__, "KinesisWebRtcManager has already been initialized.");
    return KinesisWebRtcManagerStatus::WEBRTC_ALREADY_INITIALIZED;
  }
  if (stream_infos.size() == 0) {
    AWS_LOG_WARN(__func__, "Invalid parameters. Empty WebRTC Stream Infos.");
    return KinesisWebRtcManagerStatus::INVALID_INPUT;
  }
  for (WebRtcStreamInfo webrtc_stream_info : stream_infos) {
    STATUS status = STATUS_SUCCESS;
    UINT64 custom_data = webrtc_stream_info.custom_data_;
    
    std::string signaling_channel_name = webrtc_stream_info.signaling_channel_name_;
    if (signaling_channel_name == "") {
      AWS_LOGSTREAM_ERROR(__func__, "Empty signaling channel name inputed.");
      return KinesisWebRtcManagerStatus::INVALID_INPUT;
    }
    if (kvs_webrtc_configurations_.find(signaling_channel_name) != kvs_webrtc_configurations_.end()) {
      AWS_LOGSTREAM_ERROR(__func__, "Duplicate signaling channel name was inputed: " << signaling_channel_name);
      return KinesisWebRtcManagerStatus::INVALID_INPUT;
    }

    RtcOnMessage on_data_channel_message_callback = webrtc_stream_info.on_data_channel_message_callback_;
    
    PKvsWebRtcConfiguration p_kvs_webrtc_configuration = nullptr;

    status = createKvsWebRtcConfiguration(const_cast<char *>(signaling_channel_name.c_str()),
                                          SIGNALING_CHANNEL_ROLE_TYPE_MASTER,
                                          TRUE,
                                          TRUE,
                                          &p_kvs_webrtc_configuration);
    if (status != STATUS_SUCCESS) {
      AWS_LOG_ERROR(__func__, "createKvsWebRtcConfiguration(): operation returned status code: 0x%08x", status);
      return KinesisWebRtcManagerStatus::CREATEKVSWEBRTCCONFIGURATION_FAILED;
    }
    
    // setting handlers that get spun when peer connection is established
    p_kvs_webrtc_configuration->onDataChannel = OnDataChannelOpenReceive;
    p_kvs_webrtc_configuration->onDataChannelMessageCallback = on_data_channel_message_callback;

    p_kvs_webrtc_configuration->onDataChannelOpenSend = OnDataChannelOpenSend;
    
    if (!IsValidVideoFormatType(webrtc_stream_info.video_format_type_)) {
      AWS_LOGSTREAM_ERROR(__func__, "Invalid video format type inputed: " << static_cast<int>(webrtc_stream_info.video_format_type_));
      return KinesisWebRtcManagerStatus::INVALID_VIDEO_FORMAT_TYPE;
    }
    p_kvs_webrtc_configuration->videoFormatType = webrtc_stream_info.video_format_type_;
    p_kvs_webrtc_configuration->customData = custom_data;

    status = webrtc_client_->InitKvsWebRtc();
    if (status != STATUS_SUCCESS) {
      AWS_LOG_ERROR(__func__, "initKvsWebRtc(): operation returned status code: 0x%08x", status);
      return KinesisWebRtcManagerStatus::INITKVSWEBRTC_FAILED;
    }

    strncpy(p_kvs_webrtc_configuration->clientInfo.clientId, signaling_channel_name.c_str(), MAX_SIGNALING_CLIENT_ID_LEN + 1);

    status = webrtc_client_->CreateSignalingClientSync(
              &p_kvs_webrtc_configuration->clientInfo,
              &p_kvs_webrtc_configuration->channelInfo,
              &p_kvs_webrtc_configuration->signalingClientCallbacks,
              p_kvs_webrtc_configuration->pCredentialProvider,
              &p_kvs_webrtc_configuration->signalingClientHandle);
    if (status != STATUS_SUCCESS) {
      AWS_LOG_ERROR(__func__, "createSignalingClientSync(): operation returned status code: 0x%08x", status);
      return KinesisWebRtcManagerStatus::CREATESIGNALINGCLIENTSYNC_FAILED;
    }

    status = webrtc_client_->SignalingClientConnectSync(p_kvs_webrtc_configuration->signalingClientHandle);
    if(status != STATUS_SUCCESS) {
      AWS_LOG_ERROR(__func__, "signalingClientConnectSync(): operation returned status code: 0x%08x", status);
      return KinesisWebRtcManagerStatus::SIGNALINGCLIENTCONNECTSYNC_FAILED;
    }

    kvs_webrtc_configurations_.insert({signaling_channel_name, p_kvs_webrtc_configuration});

    //Launching cleanup in a separate thread
    status = pthread_create(&cleanup_thread_id, NULL, SessionCleanupWait, (void *)p_kvs_webrtc_configuration);
    if(status != 0) {
      AWS_LOG_ERROR(__func__, "sessionCleanupWait(): Creating thread returned status code: 0x%08x", status);
      return KinesisWebRtcManagerStatus::SESSIONCLEANUPWAIT_FAILED;
    }
  }
  return KinesisWebRtcManagerStatus::SUCCESS;
}

KinesisWebRtcManagerStatus KinesisWebRtcManager::PutFrame(
  const std::string & signaling_channel_name,
  const Frame * frame) const
{
  if (kvs_webrtc_configurations_.find(signaling_channel_name) == kvs_webrtc_configurations_.end()) {
    return KinesisWebRtcManagerStatus::PUTFRAME_SIGNALING_CHANNEL_NOT_FOUND;
  }
  PKvsWebRtcConfiguration p_kvs_webrtc_configuration = kvs_webrtc_configurations_.at(signaling_channel_name);

  if (p_kvs_webrtc_configuration == nullptr) {
    AWS_LOGSTREAM_ERROR(__func__, "No KvsWebRtcConfiguration associated with the signaling channel name: " 
                                  << signaling_channel_name);
    return KinesisWebRtcManagerStatus::PUTFRAME_MISSING_KVS_WEBRTC_CONFIGURATION;
  }

  if (p_kvs_webrtc_configuration->streamingSessionCount == 0) {
    AWS_LOG_DEBUG(__func__, "No viewers connected to the WebRTC Peer Connection.");
    return KinesisWebRtcManagerStatus::PUTFRAME_NO_VIEWER;
  }
  
  if (ATOMIC_LOAD_BOOL(&p_kvs_webrtc_configuration->updatingStreamingSessionList)) {
    AWS_LOGSTREAM_INFO(__func__, "Streaming Session List being updated for signaling channel name: "
                                 << signaling_channel_name);
    return KinesisWebRtcManagerStatus::PUTFRAME_STREAMING_SESSION_BEING_UPDATED;
  }

  for (UINT32 i = 0; i < p_kvs_webrtc_configuration->streamingSessionCount; ++i) {
    STATUS status = webrtc_client_->WriteFrame(
          p_kvs_webrtc_configuration->streamingSessionList[i]->pVideoRtcRtpTransceiver,
          const_cast<Frame *>(frame));
    if (status != STATUS_SUCCESS) {
      AWS_LOGSTREAM_ERROR(__func__, "writeFrame() failed for signaling channel name: " << signaling_channel_name 
                                    << " PeerID: " << p_kvs_webrtc_configuration->streamingSessionList[i]->peerId);
      return KinesisWebRtcManagerStatus::PUTFRAME_FAILED;
    }
  }
  return KinesisWebRtcManagerStatus::SUCCESS;
}

KinesisWebRtcManagerStatus KinesisWebRtcManager::SendDataMessage(
  const std::string & signaling_channel_name,
  const bool is_binary,
  const PBYTE data,
  const UINT32 length) const
{
    if (kvs_webrtc_configurations_.find(signaling_channel_name) == kvs_webrtc_configurations_.end()) {
    return KinesisWebRtcManagerStatus::SENDDATAMESSAGE_SIGNALING_CHANNEL_NOT_FOUND;
  }
  PKvsWebRtcConfiguration p_kvs_webrtc_configuration = kvs_webrtc_configurations_.at(signaling_channel_name);

  if (p_kvs_webrtc_configuration == nullptr) {
    AWS_LOGSTREAM_ERROR(__func__, "No KvsWebRtcConfiguration associated with the signaling channel name: " 
                                  << signaling_channel_name);
    return KinesisWebRtcManagerStatus::SENDDATAMESSAGE_MISSING_KVS_WEBRTC_CONFIGURATION;
  }

  if (p_kvs_webrtc_configuration->streamingSessionCount == 0) {
    AWS_LOG_DEBUG(__func__, "No viewers connected to the WebRTC Peer Connection.");
    return KinesisWebRtcManagerStatus::SENDDATAMESSAGE_NO_VIEWER;
  }
  if (ATOMIC_LOAD_BOOL(&p_kvs_webrtc_configuration->updatingStreamingSessionList)) {
    AWS_LOGSTREAM_INFO(__func__, "Streaming Session List being updated for signaling channel name: "
                                 << signaling_channel_name);
    return KinesisWebRtcManagerStatus::SENDDATAMESSAGE_STREAMING_SESSION_BEING_UPDATED;
  }

  for (UINT32 i = 0; i < p_kvs_webrtc_configuration->streamingSessionCount; ++i) {
    STATUS status = webrtc_client_->DataChannelSend(
          p_kvs_webrtc_configuration->streamingSessionList[i]->pRtcDataChannel,
          is_binary,
          data,
          length);
    if (status != STATUS_SUCCESS) {
      AWS_LOGSTREAM_ERROR(__func__, "dataChannelSend() failed for signaling channel name: " << signaling_channel_name 
                                    << " PeerID: " << p_kvs_webrtc_configuration->streamingSessionList[i]->peerId);
      return KinesisWebRtcManagerStatus::SENDDATAMESSAGE_FAILED;
    }
  }
  return KinesisWebRtcManagerStatus::SUCCESS;
}

void KinesisWebRtcManager::OnDataChannelOpenReceive(
  UINT64 p_kvs_webrtc_config,
  PRtcDataChannel p_rtc_data_channel)
{
  AWS_LOGSTREAM_INFO(__func__, "New DataChannel has been opened to receive data: " << p_rtc_data_channel->name);

  PKvsWebRtcConfiguration p_kvs_webrtc_configuration = (PKvsWebRtcConfiguration) p_kvs_webrtc_config;
  UINT64 custom_data = p_kvs_webrtc_configuration->customData;
  // set up the data channel connection for the each peer that connects
  dataChannelOnMessage(p_rtc_data_channel, custom_data, p_kvs_webrtc_configuration->onDataChannelMessageCallback);
}

void KinesisWebRtcManager::OnDataChannelOpenSend(UINT64 custom_data, PRtcDataChannel p_rtc_data_channel)
{
  PKvsWebRtcStreamingSession p_kvs_webrtc_streaming_session = (PKvsWebRtcStreamingSession) custom_data;
  AWS_LOGSTREAM_INFO(__func__, "New DataChannel has been opened to send data: " << 
    p_kvs_webrtc_streaming_session->pRtcDataChannel->name);
}

}  // namespace Kinesis
}  // namespace Aws