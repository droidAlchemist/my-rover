import { JoystickUpdateEventType, IOT_ROS2_TOPICS } from "@/types";
import { getCommandVelocity } from "@/utils";
import { Box } from "@mui/material";
import { mqtt } from "aws-iot-device-sdk-v2";
import { useCallback } from "react";
import { Joystick, JoystickShape } from "react-joystick-component";

interface RobotControllerCardProps {
  connection: mqtt.MqttClientConnection | null;
}

export function RobotControllerCard({ connection }: RobotControllerCardProps) {
  const onChangeJoystick = useCallback(
    (d: JoystickUpdateEventType) => {
      const cmd = getCommandVelocity(d);
      if (cmd && connection) {
        connection.publish(
          IOT_ROS2_TOPICS.CONTROL,
          JSON.stringify(cmd),
          mqtt.QoS.AtMostOnce,
        );
      }
    },
    [connection],
  );

  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
      }}
    >
      <Joystick
        size={140}
        stickSize={50}
        sticky={false}
        throttle={100}
        minDistance={1}
        move={(d: any) => onChangeJoystick(d)}
        baseImage="gamepad_v2.svg"
        stickImage="ball.svg"
        baseShape={JoystickShape.Square}
      />
    </Box>
  );
}
