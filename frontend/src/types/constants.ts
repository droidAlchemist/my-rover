import { IotVelocityMessageType } from "./iot";

// Design
export const DRAWER_WIDTH = 200;
export const CONTROLLER_CONTAINER_SIZE = 150;
export const CONTROLLER_STICK_SIZE = 70;

// Default Twist Message
export const DEFAULT_TWIST_MESSAGE: IotVelocityMessageType = {
  linear: {
    x: 0,
    y: 0,
    z: 0,
  },
  angular: {
    x: 0,
    y: 0,
    z: 0,
  },
};

// Rasp Rover params
export const ROBOT_MAX_LINEAR_VELOCITY = 0.6;
export const ROBOT_MAX_ANGULAR_VELOCITY = 1.5;
// Multiplier to smoothen velocity
export const ROBOT_VELOCITY_MULTIPLIER = 0.006;
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
