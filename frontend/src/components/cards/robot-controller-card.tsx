import { IOT_ROS2_TOPICS, IotVelocityMessageType } from "@/types";
import { getCommandVelocity } from "@/utils";
import { Box } from "@mui/material";
import { mqtt } from "aws-iot-device-sdk-v2";
import { useCallback, useEffect, useState } from "react";
import Joystick, { Direction, IJoystickChangeValue } from "rc-joystick";

interface RobotControllerCardProps {
  connection: mqtt.MqttClientConnection | null;
}

export function RobotControllerCard({ connection }: RobotControllerCardProps) {
  let counter = 0;
  let timer: NodeJS.Timeout | undefined;
  const [joystickStatus, setJoystickStatus] = useState<IJoystickChangeValue>();

  const onChangeJoystick = useCallback((val: IJoystickChangeValue) => {
    setJoystickStatus(val);
  }, []);

  const publishMessage = useCallback(
    (cmd: IotVelocityMessageType) => {
      if (connection) {
        console.log("message sent");
        console.log(cmd);
        // connection.publish(
        //   IOT_ROS2_TOPICS.CONTROL,
        //   JSON.stringify(cmd),
        //   mqtt.QoS.AtMostOnce,
        // );
      }
    },
    [connection],
  );

  useEffect(() => {
    if (joystickStatus !== undefined) {
      const angle = joystickStatus?.angle || 0;
      const distance = joystickStatus?.distance;
      if (angle > 0 || distance > 0) {
        const cmd = getCommandVelocity(joystickStatus);
        publishMessage(cmd);
        timer = setInterval(() => {
          counter++;
          console.log("---------------");
          console.log("Send twist cmd -> " + counter);
          publishMessage(cmd);
          console.log("---------------");
        }, 1000);
        if (
          joystickStatus.direction === Direction.Center &&
          timer !== undefined
        ) {
          console.log("timer");
          console.log(timer);
          console.log("clear timer");
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
      <Joystick throttle={500} onChange={onChangeJoystick} />
    </Box>
  );
}
