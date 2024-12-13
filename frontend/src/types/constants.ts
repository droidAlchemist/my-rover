export const DRAWER_WIDTH = 240;
export enum IOT_ROS2_TOPICS {
  "TELEMETRY" = "ros2_mock_telemetry_topic",
  "CONTROL1" = "cmd_vel",
}
export enum VIDEO_STREAM_TYPES {
  "HLS" = "hls",
  "WEBRTC" = "webrtc",
}
export type DIRECTION_TYPES = "FORWARD" | "RIGHT" | "LEFT" | "BACKWARD";
export type MOVEMENT_TYPES = "move" | "stop" | "start";
export const ROBOT_CONTROL_MULTIPLIER = 11;

// Kinesis
export const ERROR_CHANNEL_ARN_MISSING = "Missing channel ARN";
export const ERROR_CONNECTION_OBJECT_NOT_PROVIDED =
  "Please provide a connection object";
export const ERROR_ICE_CANDIDATE_NOT_FOUND = "No ice candidate found";
export const ERROR_ICE_SERVERS_RESPONSE = "Could not get ice servers response";
export const ERROR_PEER_CONNECTION_LOCAL_DESCRIPTION_REQUIRED =
  "Could not find local description for peer connection";
export const ERROR_PEER_CONNECTION_NOT_INITIALIZED =
  "Peer connection has not been initialized";
export const ERROR_PEER_CONNECTION_NOT_FOUND = "Peer connection not found";
export const ERROR_PEER_ID_MISSING = "Peer id is missing";
export const ERROR_RESOURCE_ENDPOINT_LIST_MISSING =
  "Missing ResourceEndpointList";
export const ERROR_SIGNALING_CLIENT_NOT_CONNECTED =
  "Signaling client connection has not been established";
