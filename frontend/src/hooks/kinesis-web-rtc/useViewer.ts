import { useEffect, useRef, useState } from "react";
import { KinesisVideo } from "@aws-sdk/client-kinesis-video";
import { Role, SignalingClient } from "amazon-kinesis-video-streams-webrtc";
import { v4 as uuid } from "uuid";
import { useIceServers } from "./useIceServers";
import { useLocalMedia } from "./useLocalMedia";
import { useSignalingChannelEndpoints } from "./useSignalingChannelEndpoints";
import { useSignalingClient } from "./useSignalingClient";
import {
  ERROR_ICE_CANDIDATE_NOT_FOUND,
  ERROR_PEER_CONNECTION_LOCAL_DESCRIPTION_REQUIRED,
  ERROR_PEER_CONNECTION_NOT_INITIALIZED,
  ERROR_SIGNALING_CLIENT_NOT_CONNECTED,
  PeerConfigOptions,
  Peer,
} from "@/types";

/**
 * @description Opens a viewer connection to an active master signaling channel.
 **/
export function useViewer(
  config: Omit<PeerConfigOptions, "media"> & {
    media?: PeerConfigOptions["media"];
  },
): {
  _signalingClient: SignalingClient | undefined;
  error: Error | undefined;
  localMedia: MediaStream | undefined;
  peer: Peer | undefined;
} {
  const { channelARN, credentials, region, media } = config;
  const { error: streamError, media: localMedia } = useLocalMedia(
    media || { audio: false, video: false },
  );

  const role = Role.VIEWER;
  const clientId = useRef<string>(uuid());

  const kinesisVideoClientRef = useRef<KinesisVideo>(
    new KinesisVideo({
      region,
      credentials,
    }),
  );

  const kinesisVideoClient = kinesisVideoClientRef.current;
  const [peerConnection, setPeerConnection] = useState<RTCPeerConnection>();
  const [peerMedia, setPeerMedia] = useState<MediaStream>();
  const [peerError, setPeerError] = useState<Error>();
  // custom isOpen state - for debugging
  const [isOpen, setIsOpen] = useState<boolean>();
  const viewerOnly = !Boolean(media);
  const localMediaIsActive = Boolean(localMedia);

  const { error: signalingChannelEndpointsError, signalingChannelEndpoints } =
    useSignalingChannelEndpoints({
      channelARN,
      kinesisVideoClient,
      role,
    });

  const { error: iceServersError, iceServers } = useIceServers({
    channelARN,
    channelEndpoint: signalingChannelEndpoints?.HTTPS,
    credentials,
    region,
  });

  const { error: signalingClientError, signalingClient } = useSignalingClient({
    channelARN,
    channelEndpoint: signalingChannelEndpoints?.WSS,
    clientId: clientId.current,
    credentials,
    region,
    role,
    systemClockOffset: kinesisVideoClient.config.systemClockOffset,
  });

  const depsError =
    signalingChannelEndpointsError || iceServersError || signalingClientError;

  const peer = {
    id: clientId.current,
    connection: peerConnection,
    media: peerMedia,
  };

  async function handleSignalingClientOpen() {
    console.log(`[${clientId.current}] signaling client opened`);

    if (viewerOnly) {
      peerConnection?.addTransceiver("video");
      peerConnection
        ?.getTransceivers()
        .forEach((t) => (t.direction = "recvonly"));
    }

    const sessionDescription = await peerConnection?.createOffer({
      offerToReceiveAudio: true,
      offerToReceiveVideo: true,
    });

    try {
      await peerConnection?.setLocalDescription(sessionDescription);
    } catch (error) {
      console.error(error);
      return setPeerError(error as Error);
    }

    if (!peerConnection?.localDescription) {
      return setPeerError(
        new Error(ERROR_PEER_CONNECTION_LOCAL_DESCRIPTION_REQUIRED),
      );
    }
    setIsOpen(true);
    console.log(`[${clientId.current}] sending sdp offer`);

    signalingClient?.sendSdpOffer(peerConnection.localDescription);
  }

  async function handleSignalingClientSdpAnswer(
    answer: RTCSessionDescriptionInit,
  ) {
    console.log(`[${clientId.current}] received sdp answer`);

    if (!peerConnection) {
      throw new Error(ERROR_PEER_CONNECTION_NOT_INITIALIZED);
    }

    await peerConnection.setRemoteDescription(answer);
  }

  function handleSignalingChannelIceCandidate(candidate: RTCIceCandidate) {
    console.log(
      `[${clientId.current}] received signaling channel ice candidate`,
    );

    if (!candidate) {
      throw new Error(ERROR_ICE_CANDIDATE_NOT_FOUND);
    }

    if (!peerConnection) {
      throw new Error(ERROR_PEER_CONNECTION_NOT_INITIALIZED);
    }

    peerConnection?.addIceCandidate(candidate);
  }

  function handlePeerIceCandidate({ candidate }: RTCPeerConnectionIceEvent) {
    console.log(`[${clientId.current}] received peer ice candidate`);

    if (!signalingClient) {
      throw new Error(ERROR_SIGNALING_CLIENT_NOT_CONNECTED);
    }

    if (candidate) {
      signalingClient.sendIceCandidate(candidate);
    }
  }

  function handlePeerTrack({ streams = [] }: RTCTrackEvent) {
    console.log(`[${clientId.current}] received peer track`);

    setPeerMedia(streams[0]);
  }

  /** Initialize the peer connection with ice servers. */
  useEffect(() => {
    if (!iceServers) {
      return;
    }

    // in order to prevent certain race conditions, ensure the local media stream is active
    // before initializing the peer connection (one-way viewers are exempt)
    if (!viewerOnly && !localMediaIsActive) {
      return;
    }

    setPeerConnection(
      new RTCPeerConnection({
        iceServers,
        iceTransportPolicy: "all",
      }),
    );
  }, [localMediaIsActive, iceServers, viewerOnly]);

  /** Handle signaling client and remote peer lifecycle. */
  useEffect(() => {
    if (!peerConnection || !signalingClient) {
      return;
    }

    signalingClient.on("open", handleSignalingClientOpen);
    signalingClient.on("sdpAnswer", handleSignalingClientSdpAnswer);
    signalingClient.on("iceCandidate", handleSignalingChannelIceCandidate);
    // custom condition to check if already open. m
    !isOpen && signalingClient.open();

    peerConnection.addEventListener("icecandidate", handlePeerIceCandidate);
    peerConnection.addEventListener("track", handlePeerTrack);

    return function cleanup() {
      if (!peerConnection || !signalingClient) {
        return;
      }
      console.log(`[${clientId.current}] cleanup`);

      signalingClient.removeListener("open", handleSignalingClientOpen);
      signalingClient.removeListener(
        "sdpAnswer",
        handleSignalingClientSdpAnswer,
      );
      signalingClient.removeListener(
        "iceCandidate",
        handleSignalingChannelIceCandidate,
      );
      signalingClient.close();
      // need to debug - signal client not closing on hmr
      //setIsOpen(true);

      peerConnection.removeEventListener(
        "icecandidate",
        handlePeerIceCandidate,
      );
      peerConnection.removeEventListener("track", handlePeerTrack);
      peerConnection.close();
    };
  }, [
    clientId,
    localMediaIsActive,
    peerConnection,
    signalingClient,
    viewerOnly,
  ]);

  /** Handle peer media lifecycle. */
  useEffect(() => {
    return function cleanup() {
      peerMedia?.getTracks().forEach((track) => track.stop());
    };
  }, [peerMedia]);

  /** Send local media stream to remote peer. */
  useEffect(() => {
    if (!localMedia || !peer.connection) {
      return;
    }

    localMedia.getTracks().forEach((track: MediaStreamTrack) => {
      (peer.connection as RTCPeerConnection).addTrack(track, localMedia);
    });
  }, [localMedia, peer.connection]);

  return {
    _signalingClient: signalingClient,
    error: depsError || streamError || peerError,
    localMedia,
    peer,
  };
}
