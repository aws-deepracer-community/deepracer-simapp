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
#include "kinesis_webrtc_manager/kinesis_webrtc_client_interface.h"

namespace Aws {
namespace Kinesis {

STATUS KinesisWebRtcClientImpl::CreateSignalingClientSync(
    PSignalingClientInfo p_signaling_client_info,
    PChannelInfo p_channel_info,
    PSignalingClientCallbacks p_signaling_client_callbacks,
    PAwsCredentialProvider p_aws_credential_provider,
    PSIGNALING_CLIENT_HANDLE p_signaling_client_handle) const
{
  return createSignalingClientSync(p_signaling_client_info,
                                   p_channel_info,
                                   p_signaling_client_callbacks,
                                   p_aws_credential_provider,
                                   p_signaling_client_handle);
}

STATUS KinesisWebRtcClientImpl::DataChannelOnMessage(
    PRtcDataChannel p_rtc_data_channel,
    UINT64 custom_data,
    RtcOnMessage rtc_on_message) const
{
  return dataChannelOnMessage(p_rtc_data_channel, custom_data, rtc_on_message);
}

STATUS KinesisWebRtcClientImpl::DataChannelOnOpen(PRtcDataChannel p_rtc_data_channel, UINT64 custom_data, RtcOnOpen rtc_on_open) const
{
  return dataChannelOnOpen(p_rtc_data_channel, custom_data, rtc_on_open);
}

STATUS KinesisWebRtcClientImpl::DataChannelSend(PRtcDataChannel p_rtc_data_channel, BOOL is_binary, PBYTE data, UINT32 length) const
{
  return dataChannelSend(p_rtc_data_channel, is_binary, data, length);
}

STATUS KinesisWebRtcClientImpl::FreeSignalingClient(PSIGNALING_CLIENT_HANDLE p_signaling_client_handle) const
{
  return freeSignalingClient(p_signaling_client_handle);
}

STATUS KinesisWebRtcClientImpl::InitKvsWebRtc() const
{
  return initKvsWebRtc();
}

STATUS KinesisWebRtcClientImpl::SignalingClientConnectSync(SIGNALING_CLIENT_HANDLE signaling_client_handle) const
{
  return signalingClientConnectSync(signaling_client_handle);
}

STATUS KinesisWebRtcClientImpl::WriteFrame(PRtcRtpTransceiver p_rtc_rtp_transceiver, PFrame p_frame) const
{
  return writeFrame(p_rtc_rtp_transceiver, (PFrame) p_frame);
}

}  // namespace Kinesis
}  // namespace Aws