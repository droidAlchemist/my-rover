import {
  DIRECTION_TYPES,
  MOVEMENT_TYPES,
  TELEMETRY_MESSAGE_TYPES,
} from "./constants";

export type TelemetryMessageType = {
  type: TELEMETRY_MESSAGE_TYPES;
  data: unknown;
};
export type PoseMessageType = {
  x: number;
  y: number;
  distance: number;
};
export type OdometryMessageType = {
  pose: PoseMessageType;
  twist: IotVelocityMessageType;
  time: string;
};

export type IotSensorMessageType = {
  voltage?: number;
  temperature?: number;
  odometry?: OdometryMessageType;
  imu?: unknown;
};

export type CoordinateType = {
  x: number;
  y: number;
  z: number;
};

export type IotVelocityMessageType = {
  linear: CoordinateType;
  angular: CoordinateType;
};

export interface JoystickUpdateEventType {
  type: MOVEMENT_TYPES;
  x: number;
  y: number;
  direction: DIRECTION_TYPES | null;
  distance: number; // Percentile 0-100% of joystick
}
