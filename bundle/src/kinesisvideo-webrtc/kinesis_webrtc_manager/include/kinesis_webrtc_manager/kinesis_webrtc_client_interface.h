/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include <webrtc/com/amazonaws/kinesis/video/webrtcclient/Include.h>
#include <kinesis_webrtc_manager/common.h>

namespace Aws {
namespace Kinesis {

/**
 * This class acts as a facade between this package and the external AWS Kinesis Video Streams
 * with WebRTC in C SDK. These function calls can be found here:
 * https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c/blob/master/src/include/com/amazonaws/kinesis/video/webrtcclient/Include.h
 */
class KinesisWebRtcClientInterface
{
public:
  virtual ~KinesisWebRtcClientInterface() = default;
  /**
   * @brief Creates a Signaling client and returns a handle to it
   *
   * @param[in] PSignalingClientInfo Signaling client info
   * @param[in] PChannelInfo Signaling channel info to use/create a channel
   * @param[in] PSignalingClientCallbacks Signaling callbacks for event notifications
   * @param[in] PAwsCredentialProvider Credential provider for auth integration
   * @param[out] PSIGNALING_CLIENT_HANDLE Returned signaling client handle
   *
   * @return STATUS code of the execution. STATUS_SUCCESS on success
   */
  virtual STATUS CreateSignalingClientSync(
    PSignalingClientInfo p_signaling_client_info,
    PChannelInfo p_channel_info,
    PSignalingClientCallbacks p_signaling_client_callbacks,
    PAwsCredentialProvider p_aws_credential_provider,
    PSIGNALING_CLIENT_HANDLE p_signaling_client_handle) const = 0;

  /**
   * @brief Set a callback for data channel message
   *
   * @param[in] PRtcDataChannel Data channel struct created by createDataChannel()
   * @param[in] UINT64 User customData that will be passed along when RtcOnMessage is called
   * @param[in] RtcOnMessage User RtcOnMessage callback
   *
   * @return STATUS code of the execution. STATUS_SUCCESS on success
   */
  virtual STATUS DataChannelOnMessage(
    PRtcDataChannel p_rtc_data_channel,
    UINT64 custom_data,
    RtcOnMessage rtc_on_message) const = 0;

  /**
   * @brief Set a callback for data channel open
   *
   * @param[in] PRtcDataChannel Data channel struct created by createDataChannel()
   * @param[in] UINT64 User customData that will be passed along when RtcOnOpen is called
   * @param[in] RtcOnOpen User RtcOnOpen callback
   *
   * @return STATUS code of the execution. STATUS_SUCCESS on success
   */
  virtual STATUS DataChannelOnOpen(PRtcDataChannel p_rtc_data_channel, UINT64 custom_data, RtcOnOpen rtc_on_open)
    const = 0;
  
  /**
   * @brief Send data via the PRtcDataChannel
   *
   * Reference: https://www.w3.org/TR/webrtc/#dfn-send
   *
   * @param[in] PRtcDataChannel Configured and connected PRtcDataChannel
   * @param[in] BOOL Is message binary, if false will be delivered as a string
   * @param[in] PBYTE Data that you wish to send
   * @param[in] UINT32 Length of the PBYTE you wish to send
   *
   * @return STATUS code of the execution. STATUS_SUCCESS on success
   *
   */
  virtual STATUS DataChannelSend(PRtcDataChannel p_rtc_data_channel, BOOL is_binary, PBYTE data, UINT32 length)
    const = 0;

  /**
   * @brief Frees the Signaling client object
   *
   * NOTE: The call is idempotent.
   *
   * @param[in/out/opt] PSIGNALING_CLIENT_HANDLE Signaling client handle to free
   *
   * @return STATUS code of the execution. STATUS_SUCCESS on success
   */
  virtual STATUS FreeSignalingClient(PSIGNALING_CLIENT_HANDLE p_signaling_client_handle) const = 0;
  
  /**
   * @brief Initializes global state needed for all RtcPeerConnections. It must only be called once
   *
   * @return STATUS code of the execution. STATUS_SUCCESS on success
   */
  virtual STATUS InitKvsWebRtc() const = 0;
  
  /**
   * @brief Connects the signaling client to the socket in order to send/receive messages.
   *
   * NOTE: The call will succeed only when the signaling client is in a ready state.
   * @param SIGNALING_CLIENT_HANDLE
   *
   * @return STATUS function execution status. STATUS_SUCCESS on success
   */
  virtual STATUS SignalingClientConnectSync(SIGNALING_CLIENT_HANDLE signaling_client_handle) const = 0;

  /**
   * @brief Packetizes and sends media via the configuration specified by the RtcRtpTransceiver
   *
   * @param[in] PRtcRtpTransceiver Configured and connected RtcRtpTransceiver to send media
   * @param[in] PFrame Frame of media that will be sent
   *
   * @return STATUS code of the execution. STATUS_SUCCESS on success
   */
  virtual STATUS WriteFrame(PRtcRtpTransceiver p_rtc_rtp_transceiver, PFrame p_frame) const = 0;
protected:
  KinesisWebRtcClientInterface() = default;
};

class KinesisWebRtcClientImpl : public  KinesisWebRtcClientInterface
{
public:
  KinesisWebRtcClientImpl() = default;
  ~KinesisWebRtcClientImpl() = default;

  STATUS CreateSignalingClientSync(
    PSignalingClientInfo p_signaling_client_info,
    PChannelInfo p_channel_info,
    PSignalingClientCallbacks p_signaling_client_callbacks,
    PAwsCredentialProvider p_aws_credential_provider,
    PSIGNALING_CLIENT_HANDLE p_signaling_client_handle) const override;

  STATUS DataChannelOnMessage(
    PRtcDataChannel p_rtc_data_channel,
    UINT64 custom_data,
    RtcOnMessage rtc_on_message) const override;

  STATUS DataChannelOnOpen(PRtcDataChannel p_rtc_data_channel, UINT64 custom_data, RtcOnOpen rtc_on_open) const override;
  
  STATUS DataChannelSend(PRtcDataChannel p_rtc_data_channel, BOOL is_binary, PBYTE data, UINT32 length) const override;

  STATUS FreeSignalingClient(PSIGNALING_CLIENT_HANDLE p_signaling_client_handle) const override;

  STATUS InitKvsWebRtc() const override;

  STATUS SignalingClientConnectSync(SIGNALING_CLIENT_HANDLE signaling_client_handle) const override;

  STATUS WriteFrame(PRtcRtpTransceiver p_rtc_rtp_transceiver, PFrame p_frame) const override;

};

}  // namespace Kinesis
}  // namespace Aws