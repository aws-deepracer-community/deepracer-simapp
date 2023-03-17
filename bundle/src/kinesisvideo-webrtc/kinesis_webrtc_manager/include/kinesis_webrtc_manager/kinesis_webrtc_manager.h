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
#pragma once
#include <memory>
#include <unordered_map>
#include <vector>
#include <pthread.h>

#include "kinesis_webrtc_manager/common.h"
#include "kinesis_webrtc_manager/kinesis_webrtc_client_interface.h"
#include "kinesis_webrtc_manager/kinesis_webrtc_utils.h"

namespace Aws {
namespace Kinesis {

class KinesisWebRtcManagerInterface
{
public:
  /**
   * Initializes the WebRTC connection as Master using the stream_infos vector.
   * Creates 1 signaling channel per WebRtcStreamInfo struct.
   * @param stream_infos
   * @return KinesisWebRtcManagerStatus
   */
  virtual KinesisWebRtcManagerStatus InitializeWebRtc(
    const std::vector<WebRtcStreamInfo> & stream_infos) = 0;

  /**
   * Transmits a single frame through the WebRTC connection associated with
   * the signaling channel name.
   * @note The WebRTC connection must be initialized before this and the signaling channel name
   * must exist.
   * @param signaling_channel_name
   * @param frame
   * @return KinesisWebRtcManagerStatus
   */
  virtual KinesisWebRtcManagerStatus PutFrame(
    const std::string & signaling_channel_name,
    const Frame * frame) const = 0;

  /**
   * Transmits a single data message through the WebRTC connection associated with
   * the signaling channel name.
   * If is_binary is false, the message will be treated as a String.
   * @note The WebRTC connection must be initialized before this and the signaling channel name
   * must exist.
   * @param signaling_channel_name
   * @param is_binary
   * @param data
   * @param length
   * @return KinesisWebRtcManagerStatus
   */
  virtual KinesisWebRtcManagerStatus SendDataMessage(
    const std::string & signaling_channel_name,
    const bool is_binary,
    const PBYTE data,
    const UINT32 length) const = 0;
protected:
  KinesisWebRtcManagerInterface() = default;
  virtual ~KinesisWebRtcManagerInterface() = default;
  std::unordered_map<std::string, KvsWebRtcConfiguration *> kvs_webrtc_configurations_;
};

class KinesisWebRtcManager : public KinesisWebRtcManagerInterface
{
public:
  KinesisWebRtcManager(
    std::shared_ptr<KinesisWebRtcClientInterface> webrtc_client_interface =
      std::make_shared<KinesisWebRtcClientImpl>()
  ) : webrtc_client_(webrtc_client_interface) {}

  virtual ~KinesisWebRtcManager();

  KinesisWebRtcManagerStatus InitializeWebRtc(
    const std::vector<WebRtcStreamInfo> & stream_infos) override;

  KinesisWebRtcManagerStatus PutFrame(
    const std::string & signaling_channel_name,
    const Frame * frame) const override;

  KinesisWebRtcManagerStatus SendDataMessage(
    const std::string & signaling_channel_name,
    const bool is_binary,
    const PBYTE data,
    const UINT32 length) const override;
private:
  static void OnDataChannelOpenReceive(UINT64 custom_data, PRtcDataChannel p_rtc_data_channel);
  static void OnDataChannelOpenSend(UINT64 custom_data, PRtcDataChannel p_rtc_data_channel);
  std::shared_ptr<KinesisWebRtcClientInterface> webrtc_client_;
  pthread_t cleanup_thread_id;
};

}  // namespace Kinesis
}  // namespace Aws