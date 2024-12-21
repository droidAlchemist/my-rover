import { DIRECTION_TYPES, MOVEMENT_TYPES } from "./constants";

export type IotSensorMessageType = {
  voltage: number;
  temperature: number;
  velocity: number;
  imu: number;
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
