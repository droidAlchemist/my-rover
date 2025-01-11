import {
  IJoystickUpdateEvent,
  IOT_ROS2_TOPICS,
  IotVelocityMessageType,
} from "@/types";
import { getCommandVelocity } from "@/utils";
import { Box, Typography } from "@mui/material";
import { mqtt } from "aws-iot-device-sdk-v2";
import { useCallback, useEffect, useState } from "react";
import { Joystick } from "react-joystick-component";

interface RobotControllerCardProps {
  connection: mqtt.MqttClientConnection | null;
}

export function RobotControllerCard({ connection }: RobotControllerCardProps) {
  let counter = 0;
  let timer: NodeJS.Timeout | undefined;
  const [joystickStatus, setJoystickStatus] = useState<IJoystickUpdateEvent>();
  const [twist, setTwist] = useState<IotVelocityMessageType>({
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
  });

  const onChangeJoystick = useCallback((val: any) => {
    setJoystickStatus(val);
  }, []);

  const onStopJoystick = useCallback(() => {
    setJoystickStatus({
      type: "stop",
      x: 0,
      y: 0,
      direction: null,
      distance: 0,
    });
  }, []);

  const publishMessage = useCallback(
    (cmd: IotVelocityMessageType) => {
      if (connection) {
        setTwist(cmd);
        connection.publish(
          IOT_ROS2_TOPICS.CONTROL,
          JSON.stringify(cmd),
          mqtt.QoS.AtMostOnce,
        );
      }
    },
    [connection],
  );

  useEffect(() => {
    if (joystickStatus !== undefined) {
      const angle = joystickStatus?.x || 0;
      const distance = joystickStatus?.y;
      if (angle > 0 || distance > 0) {
        const cmd = getCommandVelocity(joystickStatus);
        publishMessage(cmd);
        timer = setInterval(() => {
          counter++;
          // console.log("---------------");
          // console.log("Send twist cmd -> " + counter);
          publishMessage(cmd);
          // console.log("---------------");
        }, 1000);
        if (joystickStatus?.distance === 0 && timer !== undefined) {
          // console.log("timer");
          // console.log(timer);
          // console.log("clear timer");
          clearInterval(timer);
        }
      }
    }
    return () => {
      if (timer !== undefined) {
        clearInterval(timer);
      }
    };
  }, [joystickStatus, publishMessage]);

  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
      }}
    >
      <Box
        sx={{
          display: "flex",
          flexDirection: "row",
          gap: 3,
        }}
      >
        <Typography>x = {twist?.linear.x}</Typography>
        <Typography>z = {twist?.angular.z}</Typography>
        {/* <Typography>d = {joystickStatus?.distance}</Typography> */}
      </Box>
      <Joystick
        throttle={250}
        baseColor={"#FFFF99"}
        stickColor={"#FFD300"}
        move={onChangeJoystick}
        stop={onStopJoystick}
      />
    </Box>
  );
}
