import * as KVSWebRTC from "amazon-kinesis-video-streams-webrtc";
import { AWSCredentials } from "./aws";

type MediaConfig = {
  audio?: boolean;
  video?: boolean | MediaTrackConstraints;
};

export interface ConfigOptions {
  channelARN: string;
  credentials: AWSCredentials;
  debug?: boolean;
  region: string;
}

export interface PeerConfigOptions extends ConfigOptions {
  media: MediaConfig;
}

export interface SignalingClientConfigOptions extends ConfigOptions {
  channelEndpoint: string | undefined;
  clientId?: string;
  role: KVSWebRTC.Role;
  systemClockOffset: number;
}

export interface Peer {
  id: string;
  connection?: RTCPeerConnection;
  isWaitingForMedia?: boolean;
  media?: MediaStream;
}
