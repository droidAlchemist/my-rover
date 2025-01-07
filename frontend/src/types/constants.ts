// Design
export const DRAWER_WIDTH = 240;

// Telemetry
export enum IOT_ROS2_TOPICS {
  "TELEMETRY" = "rover1/telemetry",
  "CONTROL" = "rover1/cmd_vel",
  "CAMERA" = "rover1/camera",
}
export enum TELEMETRY_MESSAGE_TYPES {
  "VOLTAGE" = "voltage",
  "TEMPERATURE" = "temperature",
  "ODOMETRY" = "odom",
  "POSE" = "pose",
  "IMU" = "imu",
}

// Camera Control
export type CAMERA_ACTIONS = "stop" | "start";

// ROBOT Control

export type DIRECTION_TYPES = "FORWARD" | "RIGHT" | "LEFT" | "BACKWARD";
export type MOVEMENT_TYPES = "move" | "stop" | "start";

// Rasp Rover params
export const ROBOT_MAX_LINEAR_VELOCITY = 0.65;
export const ROBOT_MAX_ANGULAR_VELOCITY = 0.5;
// Multiplier to smoothen velocity
export const ROBOT_VELOCITY_MULTIPLIER = 0.01;
export const ROBOT_ANGULAR_MULTIPLIER = 0.01;

// Kinesis WebRTC
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
