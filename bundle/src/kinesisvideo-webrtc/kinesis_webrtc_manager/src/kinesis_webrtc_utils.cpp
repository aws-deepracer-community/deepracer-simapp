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
 * https://github.com/awslabs/amazon-kinesis-video-streams-webrtc-sdk-c/blob/aa9628d189719e6fe2f709347c5a62d93a3ff98d/samples/Common.c
 * 
 * Changes:
 * Most names with "sample" have the "sample" prefix or replaced with "Kvs".
 * Removed the audio specific functions and fields such as pAudioRtcRtpTransceiver.
 * Removed lookForSslCert() which is only necessary on older linux distros.
 * Changed pStaticCredentialProvider to pKvsWebRtcCredentialProvider which dyanmically retrieves AWS credentials using
 * the Aws::Auth::AWSCredentialProviderChain and updates each time credentials are retrieves rather than a static credential.
 */

#define LOG_CLASS "WebRtcKvsWebRtcs"
#include "kinesis_webrtc_manager/kinesis_webrtc_utils.h"

#include <memory>

#include <aws/core/auth/AWSCredentialsProviderChain.h>
#include <aws/core/client/AWSError.h>
#include <aws/core/utils/logging/LogMacros.h>

#include "kinesis_webrtc_manager/common.h"

VOID onConnectionStateChange(UINT64 customData, RTC_PEER_CONNECTION_STATE newState)
{
    PKvsWebRtcStreamingSession p_kvs_webrtc_streaming_session = (PKvsWebRtcStreamingSession) customData;
    AWS_LOGSTREAM_INFO(__func__, "New connection state " << newState);

    if (newState == RTC_PEER_CONNECTION_STATE_FAILED ||
        newState == RTC_PEER_CONNECTION_STATE_CLOSED ||
        newState == RTC_PEER_CONNECTION_STATE_DISCONNECTED) {
        ATOMIC_STORE_BOOL(&p_kvs_webrtc_streaming_session->terminateFlag, TRUE);
        CVAR_BROADCAST(p_kvs_webrtc_streaming_session->pKvsWebRtcConfiguration->cvar);
    }
}

STATUS signalingClientStateChanged(UINT64 customData, SIGNALING_CLIENT_STATE state)
{
    UNUSED_PARAM(customData);
    STATUS retStatus = STATUS_SUCCESS;
    PCHAR pStateStr;

    signalingClientGetStateString(state, &pStateStr);

    AWS_LOGSTREAM_DEBUG(__func__, "Signaling client state changed to " << state << " - " << pStateStr);

    // Return success to continue
    return retStatus;
}

STATUS signalingClientError(UINT64 customData, STATUS status, PCHAR msg, UINT32 msgLen)
{
    PKvsWebRtcConfiguration pKvsWebRtcConfiguration = (PKvsWebRtcConfiguration) customData;

    AWS_LOG_WARN(__func__, "Signaling client generated an error 0x%08x - '%.*s'", status, msgLen, msg);

    // We will force re-create the signaling client on the following errors
    if (status == STATUS_SIGNALING_ICE_CONFIG_REFRESH_FAILED || status == STATUS_SIGNALING_RECONNECT_FAILED) {
        ATOMIC_STORE_BOOL(&pKvsWebRtcConfiguration->recreateSignalingClient, TRUE);
        CVAR_BROADCAST(pKvsWebRtcConfiguration->cvar);
    }

    return STATUS_SUCCESS;
}

STATUS masterMessageReceived(UINT64 customData, PReceivedSignalingMessage pReceivedSignalingMessage)
{
    STATUS retStatus = STATUS_SUCCESS;
    PKvsWebRtcConfiguration pKvsWebRtcConfiguration = (PKvsWebRtcConfiguration) customData;
    PKvsWebRtcStreamingSession pKvsWebRtcStreamingSession = NULL;
    UINT32 i;
    BOOL locked = FALSE;
    ATOMIC_BOOL expected = FALSE;

    CHK(pKvsWebRtcConfiguration != NULL, STATUS_NULL_ARG);

    MUTEX_LOCK(pKvsWebRtcConfiguration->configurationObjLock);
    locked = TRUE;

    // ice candidate message and offer message can come at any order. Therefore, if we see a new peerId, then create
    // a new KvsWebRtcStreamingSession, which in turn creates a new peerConnection
    for (i = 0; i < pKvsWebRtcConfiguration->streamingSessionCount && pKvsWebRtcStreamingSession == NULL; ++i) {
        if (0 == STRCMP(pReceivedSignalingMessage->signalingMessage.peerClientId, pKvsWebRtcConfiguration->streamingSessionList[i]->peerId)) {
            pKvsWebRtcStreamingSession = pKvsWebRtcConfiguration->streamingSessionList[i];
        }
    }

    if (pKvsWebRtcStreamingSession == NULL) {
        CHK_WARN(pKvsWebRtcConfiguration->streamingSessionCount < ARRAY_SIZE(pKvsWebRtcConfiguration->streamingSessionList),
                 retStatus, "Dropping signalling message from peer %s since maximum simultaneous streaming session of %u is reached",
                 pReceivedSignalingMessage->signalingMessage.peerClientId, ARRAY_SIZE(pKvsWebRtcConfiguration->streamingSessionList));

        AWS_LOGSTREAM_INFO(__func__, "Creating new streaming session for peer " << pReceivedSignalingMessage->signalingMessage.peerClientId);
        CHK_STATUS(createKvsWebRtcStreamingSession(pKvsWebRtcConfiguration,
                                                       pReceivedSignalingMessage->signalingMessage.peerClientId,
                                                       TRUE,
                                                       &pKvsWebRtcStreamingSession));
        pKvsWebRtcConfiguration->streamingSessionList[pKvsWebRtcConfiguration->streamingSessionCount++] = pKvsWebRtcStreamingSession;
    }

    switch (pReceivedSignalingMessage->signalingMessage.messageType) {
        case SIGNALING_MESSAGE_TYPE_OFFER:
            if (ATOMIC_COMPARE_EXCHANGE_BOOL(&pKvsWebRtcStreamingSession->sdpOfferAnswerExchanged, &expected, TRUE)) {
            CHK_STATUS(handleOffer(pKvsWebRtcConfiguration,
                                   pKvsWebRtcStreamingSession,
                                   &pReceivedSignalingMessage->signalingMessage));
        }
            break;
        case SIGNALING_MESSAGE_TYPE_ICE_CANDIDATE:
            CHK_STATUS(handleRemoteCandidate(pKvsWebRtcStreamingSession, &pReceivedSignalingMessage->signalingMessage));
            break;
        case SIGNALING_MESSAGE_TYPE_ANSWER:
            AWS_LOG_ERROR(__func__, "Unexpected message SIGNALING_MESSAGE_TYPE_ANSWER");
            break;
        default:
            AWS_LOGSTREAM_WARN(__func__, "Unknown message type " << pReceivedSignalingMessage->signalingMessage.messageType);
    }

CleanUp:

    if (locked) {
        MUTEX_UNLOCK(pKvsWebRtcConfiguration->configurationObjLock);
    }

    // Return success to continue
    return retStatus;
}

STATUS handleAnswer(PKvsWebRtcConfiguration pKvsWebRtcConfiguration, PKvsWebRtcStreamingSession pKvsWebRtcStreamingSession,
                    PSignalingMessage pSignalingMessage)
{
    UNUSED_PARAM(pKvsWebRtcConfiguration);
    STATUS retStatus = STATUS_SUCCESS;
    RtcSessionDescriptionInit answerSessionDescriptionInit;

    MEMSET(&answerSessionDescriptionInit, 0x00, SIZEOF(RtcSessionDescriptionInit));

    CHK_STATUS(deserializeSessionDescriptionInit(pSignalingMessage->payload, pSignalingMessage->payloadLen, &answerSessionDescriptionInit));
    CHK_STATUS(setRemoteDescription(pKvsWebRtcStreamingSession->pPeerConnection, &answerSessionDescriptionInit));

CleanUp:

    CHK_LOG_ERR(retStatus);

    return retStatus;
}

STATUS handleOffer(PKvsWebRtcConfiguration pKvsWebRtcConfiguration, PKvsWebRtcStreamingSession pKvsWebRtcStreamingSession,
                   PSignalingMessage pSignalingMessage)
{
    STATUS retStatus = STATUS_SUCCESS;
    RtcSessionDescriptionInit offerSessionDescriptionInit;
    BOOL locked = FALSE;
    NullableBool canTrickle;

    CHK(pKvsWebRtcConfiguration != NULL && pSignalingMessage != NULL, STATUS_NULL_ARG);

    MEMSET(&offerSessionDescriptionInit, 0x00, SIZEOF(RtcSessionDescriptionInit));
    MEMSET(&pKvsWebRtcStreamingSession->answerSessionDescriptionInit, 0x00, SIZEOF(RtcSessionDescriptionInit));

    CHK_STATUS(deserializeSessionDescriptionInit(pSignalingMessage->payload, pSignalingMessage->payloadLen, &offerSessionDescriptionInit));
    CHK_STATUS(setRemoteDescription(pKvsWebRtcStreamingSession->pPeerConnection, &offerSessionDescriptionInit));
    canTrickle = canTrickleIceCandidates(pKvsWebRtcStreamingSession->pPeerConnection);
    CHECK(!NULLABLE_CHECK_EMPTY(canTrickle));
    pKvsWebRtcConfiguration->trickleIce = canTrickle.value;
    CHK_STATUS(createAnswer(pKvsWebRtcStreamingSession->pPeerConnection, &pKvsWebRtcStreamingSession->answerSessionDescriptionInit));
    CHK_STATUS(setLocalDescription(pKvsWebRtcStreamingSession->pPeerConnection, &pKvsWebRtcStreamingSession->answerSessionDescriptionInit));

    if (!pKvsWebRtcConfiguration->trickleIce) {
        MUTEX_LOCK(pKvsWebRtcConfiguration->configurationObjLock);
        locked = TRUE;
        while (!ATOMIC_LOAD_BOOL(&pKvsWebRtcStreamingSession->candidateGatheringDone)) {
            CHK_WARN(!ATOMIC_LOAD_BOOL(&pKvsWebRtcStreamingSession->terminateFlag), STATUS_INTERNAL_ERROR, "application terminated and candidate gathering still not done");
            CVAR_WAIT(pKvsWebRtcConfiguration->cvar, pKvsWebRtcConfiguration->configurationObjLock, 500 * HUNDREDS_OF_NANOS_IN_A_MILLISECOND);
        }

        MUTEX_UNLOCK(pKvsWebRtcConfiguration->configurationObjLock);
        locked = FALSE;

        AWS_LOG_DEBUG(__func__, "Candidate collection done for non trickle ice");
        // get the latest local description once candidate gathering is done
        CHK_STATUS(peerConnectionGetCurrentLocalDescription(pKvsWebRtcStreamingSession->pPeerConnection,
                                                            &pKvsWebRtcStreamingSession->answerSessionDescriptionInit));
    }

    CHK_STATUS(respondWithAnswer(pKvsWebRtcStreamingSession));
    if (!ATOMIC_LOAD_BOOL(&pKvsWebRtcConfiguration->mediaThreadStarted)) {
        ATOMIC_STORE_BOOL(&pKvsWebRtcConfiguration->mediaThreadStarted, TRUE);
    }

CleanUp:

    CHK_LOG_ERR(retStatus);

    if (locked) {
        MUTEX_UNLOCK(pKvsWebRtcConfiguration->configurationObjLock);
    }

    return retStatus;
}

STATUS respondWithAnswer(PKvsWebRtcStreamingSession pKvsWebRtcStreamingSession)
{
    STATUS retStatus = STATUS_SUCCESS;
    SignalingMessage message;
    UINT32 buffLen = 0;

    CHK_STATUS(serializeSessionDescriptionInit(&pKvsWebRtcStreamingSession->answerSessionDescriptionInit, NULL, &buffLen));
    CHK_STATUS(serializeSessionDescriptionInit(&pKvsWebRtcStreamingSession->answerSessionDescriptionInit, message.payload, &buffLen));

    message.version = SIGNALING_MESSAGE_CURRENT_VERSION;
    message.messageType = SIGNALING_MESSAGE_TYPE_ANSWER;
    STRCPY(message.peerClientId, pKvsWebRtcStreamingSession->peerId);
    message.payloadLen = (UINT32) STRLEN(message.payload);
    message.correlationId[0] = '\0';

    retStatus = signalingClientSendMessageSync(pKvsWebRtcStreamingSession->pKvsWebRtcConfiguration->signalingClientHandle, &message);

CleanUp:

    CHK_LOG_ERR(retStatus);
    return retStatus;
}

VOID onIceCandidateHandler(UINT64 customData, PCHAR candidateJson)
{
    STATUS retStatus = STATUS_SUCCESS;
    PKvsWebRtcStreamingSession pKvsWebRtcStreamingSession = (PKvsWebRtcStreamingSession) customData;
    SignalingMessage message;

    CHK(pKvsWebRtcStreamingSession != NULL, STATUS_NULL_ARG);

    if (candidateJson == NULL) {
        AWS_LOG_DEBUG(__func__, "ice candidate gathering finished");
        ATOMIC_STORE_BOOL(&pKvsWebRtcStreamingSession->candidateGatheringDone, TRUE);
        CVAR_BROADCAST(pKvsWebRtcStreamingSession->pKvsWebRtcConfiguration->cvar);

    } else if (pKvsWebRtcStreamingSession->pKvsWebRtcConfiguration->trickleIce &&
               ATOMIC_LOAD_BOOL(&pKvsWebRtcStreamingSession->peerIdReceived)) {
        message.version = SIGNALING_MESSAGE_CURRENT_VERSION;
        message.messageType = SIGNALING_MESSAGE_TYPE_ICE_CANDIDATE;
        STRCPY(message.peerClientId, pKvsWebRtcStreamingSession->peerId);
        message.payloadLen = (UINT32) STRLEN(candidateJson);
        STRCPY(message.payload, candidateJson);
        message.correlationId[0] = '\0';
        CHK_STATUS(signalingClientSendMessageSync(pKvsWebRtcStreamingSession->pKvsWebRtcConfiguration->signalingClientHandle, &message));
    }

CleanUp:

    CHK_LOG_ERR(retStatus);
}

STATUS initializePeerConnection(PKvsWebRtcConfiguration pKvsWebRtcConfiguration, PRtcPeerConnection* ppRtcPeerConnection)
{
    STATUS retStatus = STATUS_SUCCESS;
    RtcConfiguration configuration;
    UINT32 i, j, iceConfigCount, uriCount;
    PIceConfigInfo pIceConfigInfo;
    const UINT32 maxTurnServer = 1;
    uriCount = 0;

    CHK(pKvsWebRtcConfiguration != NULL && ppRtcPeerConnection != NULL, STATUS_NULL_ARG);

    MEMSET(&configuration, 0x00, SIZEOF(RtcConfiguration));

    // Set this to custom callback to enable filtering of interfaces
    configuration.kvsRtcConfiguration.iceSetInterfaceFilterFunc = NULL;
    configuration.iceTransportPolicy = ICE_TRANSPORT_POLICY_ALL;

    // Set the  STUN server
    SNPRINTF(configuration.iceServers[0].urls, MAX_ICE_CONFIG_URI_LEN, KINESIS_VIDEO_STUN_URL, pKvsWebRtcConfiguration->channelInfo.pRegion);

    if (pKvsWebRtcConfiguration->useTurn) {
        // Set the URIs from the configuration
        CHK_STATUS(awaitGetIceConfigInfoCount(pKvsWebRtcConfiguration->signalingClientHandle, &iceConfigCount));

        for (uriCount = 0, i = 0; i < maxTurnServer; i++) {
            CHK_STATUS(signalingClientGetIceConfigInfo(pKvsWebRtcConfiguration->signalingClientHandle, i, &pIceConfigInfo));
            for (j = 0; j < pIceConfigInfo->uriCount; j++) {
                CHECK(uriCount < MAX_ICE_SERVERS_COUNT);
                STRNCPY(configuration.iceServers[uriCount + 1].urls, pIceConfigInfo->uris[j], MAX_ICE_CONFIG_URI_LEN);
                STRNCPY(configuration.iceServers[uriCount + 1].credential, pIceConfigInfo->password, MAX_ICE_CONFIG_CREDENTIAL_LEN);
                STRNCPY(configuration.iceServers[uriCount + 1].username, pIceConfigInfo->userName, MAX_ICE_CONFIG_USER_NAME_LEN);

                uriCount++;
            }
        }
    }
    pKvsWebRtcConfiguration->iceUriCount = uriCount + 1;
    CHK_STATUS(createPeerConnection(&configuration, ppRtcPeerConnection));

CleanUp:

    return retStatus;
}

STATUS awaitGetIceConfigInfoCount(SIGNALING_CLIENT_HANDLE signalingClientHandle, PUINT32 pIceConfigInfoCount)
{
    STATUS retStatus = STATUS_SUCCESS;
    UINT64 elapsed = 0;

    CHK(IS_VALID_SIGNALING_CLIENT_HANDLE(signalingClientHandle) && pIceConfigInfoCount != NULL, STATUS_NULL_ARG);

    while (TRUE) {
        // Get the configuration count
        CHK_STATUS(signalingClientGetIceConfigInfoCount(signalingClientHandle, pIceConfigInfoCount));

        // Return OK if we have some ice configs
        CHK(*pIceConfigInfoCount == 0, retStatus);

        // Check for timeout
        CHK_ERR(elapsed <= ASYNC_ICE_CONFIG_INFO_WAIT_TIMEOUT, STATUS_OPERATION_TIMED_OUT, "Couldn't retrieve ICE configurations in alotted time.");

        THREAD_SLEEP(ICE_CONFIG_INFO_POLL_PERIOD);
        elapsed += ICE_CONFIG_INFO_POLL_PERIOD;
    }

CleanUp:

    return retStatus;
}

STATUS createKvsWebRtcStreamingSession(PKvsWebRtcConfiguration pKvsWebRtcConfiguration,
                                       PCHAR peerId,
                                       BOOL isMaster,
                                       PKvsWebRtcStreamingSession *ppKvsWebRtcStreamingSession)
{
    STATUS retStatus = STATUS_SUCCESS;
    RtcMediaStreamTrack videoTrack, audioTrack;
    PKvsWebRtcStreamingSession pKvsWebRtcStreamingSession = NULL;

    MEMSET(&videoTrack, 0x00, SIZEOF(RtcMediaStreamTrack));
    MEMSET(&audioTrack, 0x00, SIZEOF(RtcMediaStreamTrack));

    CHK(pKvsWebRtcConfiguration != NULL && ppKvsWebRtcStreamingSession != NULL, STATUS_NULL_ARG);
    CHK((isMaster && peerId != NULL) || !isMaster, STATUS_INVALID_ARG);

    pKvsWebRtcStreamingSession = (PKvsWebRtcStreamingSession) MEMCALLOC(1, SIZEOF(KvsWebRtcStreamingSession));
    CHK(pKvsWebRtcStreamingSession != NULL, STATUS_NOT_ENOUGH_MEMORY);

    if (isMaster) {
        STRCPY(pKvsWebRtcStreamingSession->peerId, peerId);
    }
    ATOMIC_STORE_BOOL(&pKvsWebRtcStreamingSession->peerIdReceived, TRUE);

    pKvsWebRtcStreamingSession->pKvsWebRtcConfiguration = pKvsWebRtcConfiguration;

    ATOMIC_STORE_BOOL(&pKvsWebRtcStreamingSession->terminateFlag, FALSE);
    ATOMIC_STORE_BOOL(&pKvsWebRtcStreamingSession->candidateGatheringDone, FALSE);

    CHK_STATUS(initializePeerConnection(pKvsWebRtcConfiguration, &pKvsWebRtcStreamingSession->pPeerConnection));
    CHK_STATUS(peerConnectionOnIceCandidate(pKvsWebRtcStreamingSession->pPeerConnection,
                                            (UINT64) pKvsWebRtcStreamingSession,
                                            onIceCandidateHandler));
    CHK_STATUS(peerConnectionOnConnectionStateChange(pKvsWebRtcStreamingSession->pPeerConnection,
                                                     (UINT64) pKvsWebRtcStreamingSession,
                                                     onConnectionStateChange));
    if (pKvsWebRtcConfiguration->onDataChannel != NULL) {
        CHK_STATUS(peerConnectionOnDataChannel(pKvsWebRtcStreamingSession->pPeerConnection,
                                               (UINT64) pKvsWebRtcConfiguration,
                                               pKvsWebRtcConfiguration->onDataChannel));
    }
    switch (pKvsWebRtcConfiguration->videoFormatType)
    {
        case Aws::Kinesis::VideoFormatType::H264:
            // Declare that we support H264,Profile=42E01F,level-asymmetry-allowed=1,packetization-mode=1
            CHK_STATUS(addSupportedCodec(pKvsWebRtcStreamingSession->pPeerConnection, RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_MODE));
            // Add a SendRecv Transceiver of type video
            videoTrack.codec = RTC_CODEC_H264_PROFILE_42E01F_LEVEL_ASYMMETRY_ALLOWED_PACKETIZATION_MODE;
            break;
        case Aws::Kinesis::VideoFormatType::VP8:
            CHK_STATUS(addSupportedCodec(pKvsWebRtcStreamingSession->pPeerConnection, RTC_CODEC_VP8));
            videoTrack.codec = RTC_CODEC_VP8;
            break;
        default:
            AWS_LOG_ERROR(__func__, "Unknown VideoFormatType passed into createKvsWebRtcStreamingSession()");
            break;
    }
    videoTrack.kind = MEDIA_STREAM_TRACK_KIND_VIDEO;
    STRCPY(videoTrack.streamId, "myKvsVideoStream");
    STRCPY(videoTrack.trackId, "myVideoTrack");
    CHK_STATUS(addTransceiver(pKvsWebRtcStreamingSession->pPeerConnection,
                              &videoTrack,
                              NULL,
                              &pKvsWebRtcStreamingSession->pVideoRtcRtpTransceiver));
    
    CHK_STATUS(transceiverOnBandwidthEstimation(pKvsWebRtcStreamingSession->pVideoRtcRtpTransceiver,
                       reinterpret_cast<UINT64>(pKvsWebRtcStreamingSession),
                       bandwidthEstimationHandler));

    // Added this to utils to be able to send data over WebRTC
    CHK_STATUS(createDataChannel(pKvsWebRtcStreamingSession->pPeerConnection,
                                 const_cast<PCHAR>("myDataChannel"),
                                 NULL,
                                 &pKvsWebRtcStreamingSession->pRtcDataChannel));
    CHK_STATUS(dataChannelOnOpen(pKvsWebRtcStreamingSession->pRtcDataChannel,
                                 reinterpret_cast<UINT64>(pKvsWebRtcStreamingSession),
                                 pKvsWebRtcConfiguration->onDataChannelOpenSend));
CleanUp:

    if (STATUS_FAILED(retStatus) && pKvsWebRtcStreamingSession != NULL) {
        freeKvsWebRtcStreamingSession(&pKvsWebRtcStreamingSession);
        pKvsWebRtcStreamingSession = NULL;
    }

    if (ppKvsWebRtcStreamingSession != NULL) {
        *ppKvsWebRtcStreamingSession = pKvsWebRtcStreamingSession;
    }

    return retStatus;
}

STATUS freeKvsWebRtcStreamingSession(PKvsWebRtcStreamingSession* ppKvsWebRtcStreamingSession)
{
    STATUS retStatus = STATUS_SUCCESS;
    PKvsWebRtcStreamingSession pKvsWebRtcStreamingSession = NULL;

    CHK(ppKvsWebRtcStreamingSession != NULL, STATUS_NULL_ARG);
    pKvsWebRtcStreamingSession = *ppKvsWebRtcStreamingSession;
    CHK(pKvsWebRtcStreamingSession != NULL, retStatus);
    AWS_LOGSTREAM_INFO(__func__, "Freeing streaming session with peer id: " << pKvsWebRtcStreamingSession->peerId);

    ATOMIC_STORE_BOOL(&pKvsWebRtcStreamingSession->terminateFlag, TRUE);

    if (pKvsWebRtcStreamingSession->shutdownCallback != NULL) {
        pKvsWebRtcStreamingSession->shutdownCallback(pKvsWebRtcStreamingSession->shutdownCallbackCustomData, pKvsWebRtcStreamingSession);
    }

    if (IS_VALID_TID_VALUE(pKvsWebRtcStreamingSession->receiveAudioVideoSenderTid)) {
        THREAD_JOIN(pKvsWebRtcStreamingSession->receiveAudioVideoSenderTid, NULL);
    }

    CHK_LOG_ERR(closePeerConnection(pKvsWebRtcStreamingSession->pPeerConnection));
    CHK_LOG_ERR(freePeerConnection(&pKvsWebRtcStreamingSession->pPeerConnection));
    SAFE_MEMFREE(pKvsWebRtcStreamingSession);

CleanUp:

    CHK_LOG_ERR(retStatus);

    return retStatus;
}

STATUS streamingSessionOnShutdown(PKvsWebRtcStreamingSession pKvsWebRtcStreamingSession, UINT64 customData,
                                  StreamSessionShutdownCallback streamSessionShutdownCallback)
{
    STATUS retStatus = STATUS_SUCCESS;
    CHK(pKvsWebRtcStreamingSession != NULL && streamSessionShutdownCallback != NULL, STATUS_NULL_ARG);

    pKvsWebRtcStreamingSession->shutdownCallbackCustomData = customData;
    pKvsWebRtcStreamingSession->shutdownCallback = streamSessionShutdownCallback;

CleanUp:

    return retStatus;
}

VOID bandwidthEstimationHandler(UINT64 customData, DOUBLE maxiumBitrate)
{
    UNUSED_PARAM(customData);
    AWS_LOGSTREAM_DEBUG(__func__, "received bitrate suggestion: " << maxiumBitrate);
}

STATUS handleRemoteCandidate(PKvsWebRtcStreamingSession pKvsWebRtcStreamingSession, PSignalingMessage pSignalingMessage)
{
    STATUS retStatus = STATUS_SUCCESS;
    RtcIceCandidateInit iceCandidate;

    CHK_STATUS(deserializeRtcIceCandidateInit(pSignalingMessage->payload, pSignalingMessage->payloadLen, &iceCandidate));
    CHK_STATUS(addIceCandidate(pKvsWebRtcStreamingSession->pPeerConnection, iceCandidate.candidate));

CleanUp:

    CHK_LOG_ERR(retStatus);
    return retStatus;
}

STATUS getAwsCredentials(PAwsCredentialProvider pAwsCredentialProvider, PAwsCredentials* ppAwsCredentials) {
    AWS_LOG_DEBUG(__func__, "AWS credentials have been requested.")
    STATUS retStatus = STATUS_SUCCESS;
    static Aws::Auth::DefaultAWSCredentialsProviderChain defaultCredentialsProviderChain;
    PCHAR pAccessKey, pSecretKey, pSessionToken;
    Aws::String accessKey, secretKey, sessionToken;
    Aws::Auth::AWSCredentials credentials = defaultCredentialsProviderChain.GetAWSCredentials();

    CHK(pAwsCredentialProvider != NULL && ppAwsCredentials != NULL, STATUS_NULL_ARG);

    accessKey = credentials.GetAWSAccessKeyId();
    if (accessKey.empty()) {
        AWS_LOG_FATAL(__func__, "AWS_ACCESS_KEY_ID must be set");
        return STATUS_INVALID_OPERATION;
    }
    pAccessKey = const_cast<PCHAR>(accessKey.c_str());

    secretKey = credentials.GetAWSSecretKey();
    if (secretKey.empty()) {
        AWS_LOG_FATAL(__func__, "AWS_SECRET_ACCESS_KEY must be set");
        return STATUS_INVALID_OPERATION;
    }
    pSecretKey = const_cast<PCHAR>(secretKey.c_str());

    sessionToken = credentials.GetSessionToken();
    if (sessionToken.empty()) {
        AWS_LOG_DEBUG(__func__, "Optional AWS_SESSION_TOKEN not set");
        pSessionToken = NULL;
    } else {
        pSessionToken = const_cast<PCHAR>(sessionToken.c_str());
    }
    CHK(pAwsCredentialProvider != NULL && ppAwsCredentials != NULL, STATUS_NULL_ARG);
    CHK_STATUS(createAwsCredentials(pAccessKey, 0,
                                    pSecretKey, 0,
                                    pSessionToken, 0,
                                    MAX_UINT64, ppAwsCredentials));
CleanUp:
    return retStatus;
}

STATUS createAwsCredentialProvider(PAwsCredentialProvider* ppCredentialProvider)
{
    STATUS retStatus = STATUS_SUCCESS;
    PAwsCredentialProvider pAwsCredentialProvider = NULL;
    
    CHK(ppCredentialProvider != NULL, STATUS_NULL_ARG);

    pAwsCredentialProvider = static_cast<PAwsCredentialProvider>(MEMCALLOC(1, SIZEOF(AwsCredentialProvider)));
    CHK(pAwsCredentialProvider != NULL, STATUS_NOT_ENOUGH_MEMORY);

    pAwsCredentialProvider->getCredentialsFn = getAwsCredentials;
CleanUp:
    // Set the return value if it's not NULL
    if (ppCredentialProvider != NULL) {
        *ppCredentialProvider = pAwsCredentialProvider;
    }
    return retStatus;
}

STATUS freeAwsCredentialProvider(PAwsCredentialProvider* ppCredentialProvider)
{
    STATUS retStatus = STATUS_SUCCESS;
    PAwsCredentialProvider pAwsCredentialProvider = *ppCredentialProvider;
    CHK(ppCredentialProvider != NULL, STATUS_NULL_ARG);

    // Call is idempotent
    CHK(pAwsCredentialProvider != NULL, retStatus);

    // Release the object
    MEMFREE(pAwsCredentialProvider);

    // Set the pointer to NULL
    *ppCredentialProvider = NULL;
CleanUp:
    return retStatus;
}

STATUS createKvsWebRtcConfiguration(PCHAR channelName,
                                    SIGNALING_CHANNEL_ROLE_TYPE roleType,
                                    BOOL trickleIce,
                                    BOOL useTurn,
                                    PKvsWebRtcConfiguration* ppKvsWebRtcConfiguration)
{
    STATUS retStatus = STATUS_SUCCESS;

    PKvsWebRtcConfiguration pKvsWebRtcConfiguration = NULL;
    
    CHK(ppKvsWebRtcConfiguration != NULL, STATUS_NULL_ARG);

    CHK(NULL != (pKvsWebRtcConfiguration =
        static_cast<PKvsWebRtcConfiguration>(MEMCALLOC(1, SIZEOF(KvsWebRtcConfiguration)))), STATUS_NOT_ENOUGH_MEMORY);
    
    if ((pKvsWebRtcConfiguration->channelInfo.pRegion = getenv(DEFAULT_REGION_ENV_VAR)) == NULL) {
        pKvsWebRtcConfiguration->channelInfo.pRegion = DEFAULT_AWS_REGION;
    }

    CHK_STATUS(createAwsCredentialProvider(&pKvsWebRtcConfiguration->pCredentialProvider));

    pKvsWebRtcConfiguration->signalingClientHandle = INVALID_SIGNALING_CLIENT_HANDLE_VALUE;
    pKvsWebRtcConfiguration->configurationObjLock = MUTEX_CREATE(TRUE);
    pKvsWebRtcConfiguration->cvar = CVAR_CREATE();
    pKvsWebRtcConfiguration->trickleIce = trickleIce;
    pKvsWebRtcConfiguration->useTurn = useTurn;

    pKvsWebRtcConfiguration->channelInfo.version = CHANNEL_INFO_CURRENT_VERSION;
    pKvsWebRtcConfiguration->channelInfo.pChannelName = channelName;
    pKvsWebRtcConfiguration->channelInfo.pKmsKeyId = NULL;
    pKvsWebRtcConfiguration->channelInfo.tagCount = 0;
    pKvsWebRtcConfiguration->channelInfo.pTags = NULL;
    pKvsWebRtcConfiguration->channelInfo.channelType = SIGNALING_CHANNEL_TYPE_SINGLE_MASTER;
    pKvsWebRtcConfiguration->channelInfo.channelRoleType = roleType;
    pKvsWebRtcConfiguration->channelInfo.cachingPolicy = SIGNALING_API_CALL_CACHE_TYPE_FILE;
    pKvsWebRtcConfiguration->channelInfo.cachingPeriod = SIGNALING_API_CALL_CACHE_TTL_SENTINEL_VALUE;
    pKvsWebRtcConfiguration->channelInfo.asyncIceServerConfig = TRUE;
    pKvsWebRtcConfiguration->channelInfo.retry = TRUE;
    pKvsWebRtcConfiguration->channelInfo.reconnect = TRUE;
    pKvsWebRtcConfiguration->channelInfo.messageTtl = 0; // Default is 60 seconds

    pKvsWebRtcConfiguration->signalingClientCallbacks.version = SIGNALING_CLIENT_CALLBACKS_CURRENT_VERSION;
    pKvsWebRtcConfiguration->signalingClientCallbacks.messageReceivedFn = masterMessageReceived;
    pKvsWebRtcConfiguration->signalingClientCallbacks.errorReportFn = signalingClientError;
    pKvsWebRtcConfiguration->signalingClientCallbacks.stateChangeFn = signalingClientStateChanged;
    pKvsWebRtcConfiguration->signalingClientCallbacks.customData = (UINT64) pKvsWebRtcConfiguration;

    pKvsWebRtcConfiguration->clientInfo.version = SIGNALING_CLIENT_INFO_CURRENT_VERSION;

    ATOMIC_STORE_BOOL(&pKvsWebRtcConfiguration->interrupted, FALSE);
    ATOMIC_STORE_BOOL(&pKvsWebRtcConfiguration->mediaThreadStarted, FALSE);
    ATOMIC_STORE_BOOL(&pKvsWebRtcConfiguration->appTerminateFlag, FALSE);
    ATOMIC_STORE_BOOL(&pKvsWebRtcConfiguration->updatingStreamingSessionList, FALSE);
    ATOMIC_STORE_BOOL(&pKvsWebRtcConfiguration->recreateSignalingClient, FALSE);

    pKvsWebRtcConfiguration->iceUriCount = 0;
CleanUp:

    if (STATUS_FAILED(retStatus)) {
        freeKvsWebRtcConfiguration(&pKvsWebRtcConfiguration);
    }

    if (ppKvsWebRtcConfiguration != NULL) {
        *ppKvsWebRtcConfiguration = pKvsWebRtcConfiguration;
    }

    return retStatus;
}

STATUS freeKvsWebRtcConfiguration(PKvsWebRtcConfiguration* ppKvsWebRtcConfiguration)
{
    AWS_LOG_DEBUG(__func__, "Freeing KvsWebRtcConfiguration");
    STATUS retStatus = STATUS_SUCCESS;
    PKvsWebRtcConfiguration pKvsWebRtcConfiguration;
    UINT32 i;

    CHK(ppKvsWebRtcConfiguration != NULL, STATUS_NULL_ARG);
    pKvsWebRtcConfiguration = *ppKvsWebRtcConfiguration;

    CHK(pKvsWebRtcConfiguration != NULL, retStatus);

    for (i = 0; i < pKvsWebRtcConfiguration->streamingSessionCount; ++i) {
        freeKvsWebRtcStreamingSession(&pKvsWebRtcConfiguration->streamingSessionList[i]);
    }

    deinitKvsWebRtc();

    if (IS_VALID_CVAR_VALUE(pKvsWebRtcConfiguration->cvar) &&
        IS_VALID_MUTEX_VALUE(pKvsWebRtcConfiguration->configurationObjLock)) {
        CVAR_BROADCAST(pKvsWebRtcConfiguration->cvar);
        // lock to wait until awoken thread finish.
        MUTEX_LOCK(pKvsWebRtcConfiguration->configurationObjLock);
        MUTEX_UNLOCK(pKvsWebRtcConfiguration->configurationObjLock);
    }

    if (IS_VALID_MUTEX_VALUE(pKvsWebRtcConfiguration->configurationObjLock)) {
        MUTEX_FREE(pKvsWebRtcConfiguration->configurationObjLock);
    }

    if (IS_VALID_CVAR_VALUE(pKvsWebRtcConfiguration->cvar)) {
        CVAR_FREE(pKvsWebRtcConfiguration->cvar);
    }

    freeAwsCredentialProvider(&pKvsWebRtcConfiguration->pCredentialProvider);

    MEMFREE(*ppKvsWebRtcConfiguration);
    *ppKvsWebRtcConfiguration = NULL;

CleanUp:

    LEAVES();
    return retStatus;
}

STATUS sessionCleanupWait(PKvsWebRtcConfiguration pKvsWebRtcConfiguration)
{
    STATUS retStatus = STATUS_SUCCESS;
    PKvsWebRtcStreamingSession pKvsWebRtcStreamingSession = NULL;
    UINT32 i;
    BOOL locked = FALSE;
    SIGNALING_CLIENT_STATE signalingClientState;

    CHK(pKvsWebRtcConfiguration != NULL, STATUS_NULL_ARG);

    MUTEX_LOCK(pKvsWebRtcConfiguration->configurationObjLock);
    locked = TRUE;
    while (!ATOMIC_LOAD_BOOL(&pKvsWebRtcConfiguration->interrupted)) {
        // scan and cleanup terminated streaming session
        for (UINT32 i = 0; i < pKvsWebRtcConfiguration->streamingSessionCount; ++i) {
            if (ATOMIC_LOAD_BOOL(&pKvsWebRtcConfiguration->streamingSessionList[i]->terminateFlag)) {
                pKvsWebRtcStreamingSession = pKvsWebRtcConfiguration->streamingSessionList[i];
                ATOMIC_STORE_BOOL(&pKvsWebRtcConfiguration->updatingStreamingSessionList, TRUE);
                AWS_LOGSTREAM_INFO(__func__, "Ending streaming session for peer " << pKvsWebRtcStreamingSession->peerId);
                while (ATOMIC_LOAD(&pKvsWebRtcConfiguration->streamingSessionListReadingThreadCount) != 0) {
                    // busy loop until all media thread stopped reading stream session list
                    THREAD_SLEEP(5 * HUNDREDS_OF_NANOS_IN_A_MILLISECOND);
                }
                // swap with last element and decrement count
                pKvsWebRtcConfiguration->streamingSessionCount--;
                pKvsWebRtcConfiguration->streamingSessionList[i] = pKvsWebRtcConfiguration->streamingSessionList[pKvsWebRtcConfiguration->streamingSessionCount];
                ATOMIC_STORE_BOOL(&pKvsWebRtcConfiguration->updatingStreamingSessionList, FALSE);
                CHK_STATUS(freeKvsWebRtcStreamingSession(&pKvsWebRtcStreamingSession));
            }
        }
        // periodically wake up and clean up terminated streaming session
            CVAR_WAIT(pKvsWebRtcConfiguration->cvar, pKvsWebRtcConfiguration->configurationObjLock, 5 * HUNDREDS_OF_NANOS_IN_A_SECOND);
            // Check if we need to re-create the signaling client on-the-fly
            if (ATOMIC_LOAD_BOOL(&pKvsWebRtcConfiguration->recreateSignalingClient) &&
                STATUS_SUCCEEDED(freeSignalingClient(&pKvsWebRtcConfiguration->signalingClientHandle)) &&
                STATUS_SUCCEEDED(createSignalingClientSync(&pKvsWebRtcConfiguration->clientInfo, &pKvsWebRtcConfiguration->channelInfo,
                                                           &pKvsWebRtcConfiguration->signalingClientCallbacks, pKvsWebRtcConfiguration->pCredentialProvider,
                                                           &pKvsWebRtcConfiguration->signalingClientHandle))) {
                // Re-set the variable again
                ATOMIC_STORE_BOOL(&pKvsWebRtcConfiguration->recreateSignalingClient, FALSE);
            }

            // Check the signaling client state and connect if needed
            if (IS_VALID_SIGNALING_CLIENT_HANDLE(pKvsWebRtcConfiguration->signalingClientHandle)) {
                CHK_STATUS(signalingClientGetCurrentState(pKvsWebRtcConfiguration->signalingClientHandle, &signalingClientState));
                if (signalingClientState == SIGNALING_CLIENT_STATE_READY) {
                    UNUSED_PARAM(signalingClientConnectSync(pKvsWebRtcConfiguration->signalingClientHandle));
                }
            }
    }

CleanUp:

    CHK_LOG_ERR(retStatus);

    if (locked) {
        MUTEX_UNLOCK(pKvsWebRtcConfiguration->configurationObjLock);
    }

    LEAVES();
    return retStatus;
}
