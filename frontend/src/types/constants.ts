export const DRAWER_WIDTH = 240;
export enum IOT_ROS2_TOPICS {
  "TELEMETRY" = "ros2_telemetry_topic",
  "CONTROL1" = "cmd_vel",
}
export enum VIDEO_STREAM_TYPES {
  "HLS" = "hls",
  "WEBRTC" = "webrtc",
}
export type DIRECTION_TYPES = "FORWARD" | "RIGHT" | "LEFT" | "BACKWARD";
export type MOVEMENT_TYPES = "move" | "stop" | "start";
export const ROBOT_CONTROL_MULTIPLIER = 7;

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

export const LINE_CHART_OPTIONS = {
  responsive: true,
  maintainAspectRatio: false,
  plugins: {
    legend: {
      display: false,
    },
  },
  interaction: {
    intersect: false,
    mode: "index",
  },
  scales: {
    y: {
      grid: {
        drawBorder: false,
        display: true,
        drawOnChartArea: true,
        drawTicks: false,
        borderDash: [5, 5],
      },
      ticks: {
        display: true,
        padding: 10,
        color: "#b2b9bf",
        font: {
          size: 11,
          family: "Open Sans, Helvetica, Arial, sans-serif",
          style: "normal",
          lineHeight: 2,
        },
      },
    },
    x: {
      grid: {
        drawBorder: false,
        display: false,
        drawOnChartArea: false,
        drawTicks: false,
        borderDash: [5, 5],
      },
      ticks: {
        display: true,
        color: "#b2b9bf",
        padding: 20,
        font: {
          size: 11,
          family: "Open Sans, Helvetica, Arial, sans-serif",
          style: "normal",
          lineHeight: 2,
        },
      },
    },
  },
};
