import { JoystickUpdateEventType, IOT_ROS2_TOPICS } from "@/types";
import { getCommandVelocity } from "@/utils";
import { Box } from "@mui/material";
import { mqtt } from "aws-iot-device-sdk-v2";
import { useCallback } from "react";
import { Joystick, JoystickShape } from "react-joystick-component";

interface RobotControllerCardProps {
  connection: mqtt.MqttClientConnection | null;
}

export default function RobotControllerCard({
  connection,
}: RobotControllerCardProps) {
  const onChangeJoystick = useCallback(
    (d: JoystickUpdateEventType) => {
      const cmd = getCommandVelocity(d);
      if (cmd && connection) {
        connection.publish(
          IOT_ROS2_TOPICS.CONTROL1,
          JSON.stringify(cmd),
          mqtt.QoS.AtMostOnce,
        );
      }
    },
    [connection],
  );

  return (
    <Box sx={{ display: "flex", flexDirection: "column" }}>
      <Joystick
        size={140}
        stickSize={50}
        sticky={false}
        baseColor="gray"
        throttle={1000}
        minDistance={10}
        move={(d: any) => onChangeJoystick(d)}
        baseImage="gamepad.jpg"
        stickImage="green-ball.png"
        baseShape={JoystickShape.Square}
      />
    </Box>
  );
}
