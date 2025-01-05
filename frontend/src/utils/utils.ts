import {
  IotCameraMessageType,
  IotVelocityMessageType,
  JoystickUpdateEventType,
  ROBOT_CONTROL_MULTIPLIER,
} from "@/types";

export const calculateVelocity = (value: number) => {
  return value * ROBOT_CONTROL_MULTIPLIER;
};

export const withErrorLog =
  (fn: (e: Error) => void) =>
  (error: Error): void => {
    console.log(error);
    return fn(error);
  };

export const roundDecimal = (d?: string) => (d ? parseFloat(d).toFixed(2) : 0);

export const caclulateBatteryPercent = (volt: number) => {
  const y = 0.2777 * volt * 10 * 2 - 5.9451 * volt + 31.809;
  if (y > 0 && y < 2) return 0;
  return y.toFixed(2);
};

export const caclulateBatteryPercent2 = (volt: number) => {
  const high = 12.25;
  const low = 10.5;
  let sum = ((volt - low) / (high - low)) * 100;
  if (Math.round(sum) >= 100) sum = 100; // if greater than 100% then keep it there.
  return sum.toFixed(2);
};

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

export const getCameraCommand = (active: boolean) => {
  const cmd: IotCameraMessageType = {
    action: active ? "start" : "stop",
  };
  return cmd;
};

/* 
  ROS2 Commands: forward  --> x   backward --> -x   left --> z    right -> -z
  For Joystick control convert y --> linear x and x --> angular z  
*/
export const getCommandVelocity = (d: JoystickUpdateEventType) => {
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
  if (d.type === "move") {
    cmd.linear.x = calculateVelocity(d.y);
    cmd.angular.z = d.x;
    return cmd;
  }
  return undefined;
};
