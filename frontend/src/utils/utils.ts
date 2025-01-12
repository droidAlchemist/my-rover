import {
  DEFAULT_TWIST_MESSAGE,
  DIRECTION_TYPES,
  IJoystickUpdateEvent,
  IotCameraMessageType,
  IotVelocityMessageType,
  ROBOT_ANGULAR_MULTIPLIER,
  ROBOT_MAX_ANGULAR_VELOCITY,
  ROBOT_MAX_LINEAR_VELOCITY,
  ROBOT_VELOCITY_MULTIPLIER,
} from "@/types";

export const calculateVelocity = (event: IJoystickUpdateEvent) => {
  let value = event.y;
  // let velocity = value * ROBOT_VELOCITY_MULTIPLIER;
  let velocity = value;
  if (
    event.direction === DIRECTION_TYPES.LEFT ||
    event.direction === DIRECTION_TYPES.RIGHT
  ) {
    velocity = event.distance * ROBOT_VELOCITY_MULTIPLIER;
    if (event.direction === DIRECTION_TYPES.RIGHT) {
      velocity = -velocity;
    }
    // console.log("forward/backward vel. = ", velocity);
  }
  velocity = limitVelocity(velocity);
  return velocity;
};

export const calculateAngularVelocity = (event: IJoystickUpdateEvent) => {
  let value = -1.25 * event.x;
  // let angularVelocity = value * ROBOT_ANGULAR_MULTIPLIER;
  let angularVelocity = value;
  if (
    event.direction === DIRECTION_TYPES.FORWARD ||
    event.direction === DIRECTION_TYPES.BACKWARD
  ) {
    angularVelocity = event.distance * ROBOT_ANGULAR_MULTIPLIER;
    // if (event.direction === DIRECTION_TYPES.BACKWARD) {
    //   angularVelocity = -angularVelocity;
    // }
    // console.log("forward/backward ang vel. = ", angularVelocity);
  }
  angularVelocity = limitAngularVelocity(angularVelocity);
  return angularVelocity;
};

/* 
  ROS2 Commands: forward  --> x   backward --> -x   left --> z    right -> -z
  For Joystick control using mouse convert y --> linear x and x --> angular z  
*/
export const getCommandVelocity = (d: IJoystickUpdateEvent) => {
  let cmd: IotVelocityMessageType = {
    ...DEFAULT_TWIST_MESSAGE,
  };
  let x = Number(roundDecimal(String(calculateVelocity(d))));
  let z = Number(roundDecimal(String(calculateAngularVelocity(d))));
  cmd.linear.x = x;
  cmd.angular.z = z;
  return cmd;
};

export const withErrorLog =
  (fn: (e: Error) => void) =>
  (error: Error): void => {
    console.log(error);
    return fn(error);
  };

export const getCameraCommand = (active: boolean) => {
  const cmd: IotCameraMessageType = {
    action: active ? "start" : "stop",
  };
  return cmd;
};

export const roundDecimal = (d?: string) => (d ? parseFloat(d).toFixed(2) : 0);

export const getBatteryPercentage = (voltage: number) => {
  let batteryPercentage = 0;
  if (voltage <= 10.5) {
    batteryPercentage = 0;
  } else if (voltage <= 10.8) {
    batteryPercentage = 5;
  } else if (voltage <= 10.9) {
    batteryPercentage = 10;
  } else if (voltage <= 11.0) {
    batteryPercentage = 15;
  } else if (voltage <= 11.2) {
    batteryPercentage = 25;
  } else if (voltage <= 11.4) {
    batteryPercentage = 35;
  } else if (voltage <= 11.6) {
    batteryPercentage = 45;
  } else if (voltage <= 11.8) {
    batteryPercentage = 55;
  } else if (voltage <= 11.9) {
    batteryPercentage = 65;
  } else if (voltage <= 12.05) {
    batteryPercentage = 75;
  } else if (voltage <= 12.1) {
    batteryPercentage = 85;
  } else if (voltage <= 12.2) {
    batteryPercentage = 95;
  } else if (voltage <= 12.25) {
    batteryPercentage = 98;
  } else if (voltage >= 12.3) {
    batteryPercentage = 100;
  }
  return batteryPercentage;
};

const limitVelocity = (velocity: number) => {
  if (velocity > ROBOT_MAX_LINEAR_VELOCITY) {
    velocity = ROBOT_MAX_LINEAR_VELOCITY;
  } else if (velocity < -ROBOT_MAX_LINEAR_VELOCITY) {
    velocity = -ROBOT_MAX_LINEAR_VELOCITY;
  }
  return velocity;
};

const limitAngularVelocity = (angularVelocity: number) => {
  if (angularVelocity > ROBOT_MAX_ANGULAR_VELOCITY) {
    angularVelocity = ROBOT_MAX_ANGULAR_VELOCITY;
  } else if (angularVelocity < -ROBOT_MAX_ANGULAR_VELOCITY) {
    angularVelocity = -ROBOT_MAX_ANGULAR_VELOCITY;
  }
  return angularVelocity;
};
