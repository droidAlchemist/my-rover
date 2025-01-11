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

export enum DIRECTION_TYPES {
  "FORWARD" = "FORWARD",
  "RIGHT" = "RIGHT",
  "LEFT" = "LEFT",
  "BACKWARD" = "BACKWARD",
}
export type MOVEMENT_TYPES = "move" | "stop" | "start";
