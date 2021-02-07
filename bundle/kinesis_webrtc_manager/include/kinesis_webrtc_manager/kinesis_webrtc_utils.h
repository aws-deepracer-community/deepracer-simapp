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

/* 
 * This file is forked from 
 * https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c/blob/aa9628d189719e6fe2f709347c5a62d93a3ff98d/samples/Samples.h
 * 
 * Changes:
 *   Most names with "sample" have the "sample" prefix or replaced with "Kvs".
 *   Removed the audio specific functions and fields such as pAudioRtcRtpTransceiver.
 * 
 * Removed various methods:
 *   sigintHandler() - ROS has its own SIGINT handler
 *   viewerMessageReceived() - this package only supports running as the Master in the WebRTC connection
 *   sampleFrameHandler() - unnecessary because the Master does not receive frames from the Viewer
 *   lookForSslCert() - only necessary on older linux distros
 */
#pragma once

#include <stdbool.h>
#include <webrtc/com/amazonaws/kinesis/video/webrtcclient/Include.h>

#define DEFAULT_MAX_CONCURRENT_STREAMING_SESSION                                10
#define ASYNC_ICE_CONFIG_INFO_WAIT_TIMEOUT (3 * HUNDREDS_OF_NANOS_IN_A_SECOND)
#define ICE_CONFIG_INFO_POLL_PERIOD        (20 * HUNDREDS_OF_NANOS_IN_A_MILLISECOND)

namespace Aws {
namespace Kinesis {
    enum class VideoFormatType;
} // namespace Kinesis
} // namespace Aws

typedef enum {
    SAMPLE_STREAMING_VIDEO_ONLY,
    SAMPLE_STREAMING_AUDIO_VIDEO,
} KvsWebRtcStreamingMediaType;

typedef struct __KvsWebRtcStreamingSession KvsWebRtcStreamingSession;
typedef struct __KvsWebRtcStreamingSession* PKvsWebRtcStreamingSession;
typedef struct __KvsWebRtcConfiguration KvsWebRtcConfiguration;
typedef struct __KvsWebRtcConfiguration* PKvsWebRtcConfiguration;

struct __KvsWebRtcConfiguration {
    volatile ATOMIC_BOOL appTerminateFlag;
    volatile ATOMIC_BOOL interrupted;
    volatile ATOMIC_BOOL mediaThreadStarted;
    volatile ATOMIC_BOOL updatingStreamingSessionList;
    volatile ATOMIC_BOOL recreateSignalingClient;
    volatile SIZE_T streamingSessionListReadingThreadCount;
    ChannelInfo channelInfo;
    PAwsCredentialProvider pCredentialProvider;
    SIGNALING_CLIENT_HANDLE signalingClientHandle;
    RtcOnDataChannel onDataChannel;
    RtcOnMessage onDataChannelMessageCallback;

    Aws::Kinesis::VideoFormatType videoFormatType;
    RtcOnOpen onDataChannelOpenSend;

    MUTEX configurationObjLock;
    CVAR cvar;
    BOOL trickleIce;
    BOOL useTurn;
    PKvsWebRtcStreamingSession streamingSessionList[DEFAULT_MAX_CONCURRENT_STREAMING_SESSION];
    UINT32 streamingSessionCount;
    UINT32 iceUriCount;
    SignalingClientCallbacks signalingClientCallbacks;
    SignalingClientInfo clientInfo;

    UINT64 customData;
};

typedef VOID (*StreamSessionShutdownCallback)(UINT64, PKvsWebRtcStreamingSession);

struct __KvsWebRtcStreamingSession {
    volatile ATOMIC_BOOL terminateFlag;
    volatile ATOMIC_BOOL candidateGatheringDone;
    volatile ATOMIC_BOOL peerIdReceived;
    volatile ATOMIC_BOOL sdpOfferAnswerExchanged;
    PRtcPeerConnection pPeerConnection;
    PRtcRtpTransceiver pVideoRtcRtpTransceiver;
    PRtcDataChannel pRtcDataChannel;
    RtcSessionDescriptionInit answerSessionDescriptionInit;
    PKvsWebRtcConfiguration pKvsWebRtcConfiguration;
    CHAR peerId[MAX_SIGNALING_CLIENT_ID_LEN + 1];
    TID receiveAudioVideoSenderTid;

    // this is called when the KvsWebRtcStreamingSession is being freed
    StreamSessionShutdownCallback shutdownCallback;
    UINT64 shutdownCallbackCustomData;
};

STATUS getAwsCredentials(PAwsCredentialProvider pAwsCredentialProvider, PAwsCredentials* ppAwsCredentials);
STATUS createAwsCredentialProvider(PAwsCredentialProvider* ppCredentialProvider);
STATUS freeAwsCredentialProvider(PAwsCredentialProvider* ppCredentialProvider);
STATUS createKvsWebRtcConfiguration(PCHAR, SIGNALING_CHANNEL_ROLE_TYPE, BOOL, BOOL, PKvsWebRtcConfiguration*);
STATUS freeKvsWebRtcConfiguration(PKvsWebRtcConfiguration*);
STATUS signalingClientStateChanged(UINT64, SIGNALING_CLIENT_STATE);
STATUS signalingClientError(UINT64 customData, STATUS status, PCHAR msg, UINT32 msgLen);
STATUS masterMessageReceived(UINT64, PReceivedSignalingMessage);
STATUS handleAnswer(PKvsWebRtcConfiguration, PKvsWebRtcStreamingSession, PSignalingMessage);
STATUS handleOffer(PKvsWebRtcConfiguration, PKvsWebRtcStreamingSession, PSignalingMessage);
STATUS handleRemoteCandidate(PKvsWebRtcStreamingSession, PSignalingMessage);
STATUS initializePeerConnection(PKvsWebRtcConfiguration, PRtcPeerConnection*);
STATUS createKvsWebRtcStreamingSession(PKvsWebRtcConfiguration, PCHAR, BOOL, PKvsWebRtcStreamingSession*);
STATUS freeKvsWebRtcStreamingSession(PKvsWebRtcStreamingSession*);
STATUS streamingSessionOnShutdown(PKvsWebRtcStreamingSession, UINT64, StreamSessionShutdownCallback);
STATUS respondWithAnswer(PKvsWebRtcStreamingSession);
STATUS resetKvsWebRtcConfigurationState(PKvsWebRtcConfiguration);
VOID bandwidthEstimationHandler(UINT64 customData, DOUBLE maxiumBitrate);
VOID onConnectionStateChange(UINT64, RTC_PEER_CONNECTION_STATE);
STATUS sessionCleanupWait(PKvsWebRtcConfiguration);
STATUS awaitGetIceConfigInfoCount(SIGNALING_CLIENT_HANDLE, PUINT32);
