import {
  IotCameraMessageType,
  IotVelocityMessageType,
  ROBOT_ANGULAR_MULTIPLIER,
  ROBOT_MAX_ANGULAR_VELOCITY,
  ROBOT_MAX_LINEAR_VELOCITY,
  ROBOT_VELOCITY_MULTIPLIER,
} from "@/types";
import { Direction, IJoystickChangeValue } from "rc-joystick";

export const calculateVelocity = (value: number) => {
  let velocity = value * ROBOT_VELOCITY_MULTIPLIER;
  if (velocity > ROBOT_MAX_LINEAR_VELOCITY) {
    velocity = ROBOT_MAX_LINEAR_VELOCITY;
  }
  return velocity;
};

export const calculateAngularVelocity = (value: number) => {
  let angularVelocity = value * ROBOT_ANGULAR_MULTIPLIER;
  if (angularVelocity > ROBOT_MAX_ANGULAR_VELOCITY) {
    angularVelocity = ROBOT_MAX_ANGULAR_VELOCITY;
  }
  return angularVelocity;
};

export const getCameraCommand = (active: boolean) => {
  const cmd: IotCameraMessageType = {
    action: active ? "start" : "stop",
  };
  return cmd;
};

/* 
  ROS2 Commands: forward  --> x   backward --> -x   left --> z    right -> -z
  For Joystick control convert y --> linear x and x --> angular z  
  Send either linear velocity or angular basedon direction
*/
export const getCommandVelocity = (d: IJoystickChangeValue) => {
  console.log(d);
  let cmd: IotVelocityMessageType = {
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
  if (d.direction === Direction.Top || d.direction === Direction.Bottom) {
    cmd.linear.x = calculateVelocity(d.distance);
    if (d.direction === Direction.Bottom) {
      cmd.linear.x = -cmd.linear.x;
    }
  } else if (
    d.direction === Direction.Left ||
    d.direction === Direction.Right
  ) {
    cmd.angular.z = calculateAngularVelocity(d.distance);
    if (d.direction === Direction.Left) {
      cmd.angular.z = -cmd.angular.z;
    }
  }
  return cmd;
};

export const withErrorLog =
  (fn: (e: Error) => void) =>
  (error: Error): void => {
    console.log(error);
    return fn(error);
  };

export const roundDecimal = (d?: string) => (d ? parseFloat(d).toFixed(2) : 0);

export const getBatteryPercentage = (voltage: number) => {
  let batteryPercentage = 0;
  if (voltage <= 10.5) {
    batteryPercentage = 0;
  } else if (voltage <= 11.31) {
    batteryPercentage = 10;
  } else if (voltage <= 11.58) {
    batteryPercentage = 20;
  } else if (voltage <= 11.75) {
    batteryPercentage = 30;
  } else if (voltage <= 11.9) {
    batteryPercentage = 40;
  } else if (voltage <= 12.06) {
    batteryPercentage = 50;
  } else if (voltage <= 12.2) {
    batteryPercentage = 60;
  } else if (voltage <= 12.32) {
    batteryPercentage = 70;
  } else if (voltage <= 12.42) {
    batteryPercentage = 80;
  } else if (voltage <= 12.5) {
    batteryPercentage = 90;
  } else if (voltage <= 12.6) {
    batteryPercentage = 100;
  } else if (voltage >= 12.6) {
    batteryPercentage = 100;
  }
  return batteryPercentage;
};
