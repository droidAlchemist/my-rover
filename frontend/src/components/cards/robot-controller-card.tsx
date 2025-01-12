import {
  CONTROLLER_CONTAINER_SIZE,
  CONTROLLER_STICK_SIZE,
  IJoystickUpdateEvent,
  IOT_ROS2_TOPICS,
  IotVelocityMessageType,
} from "@/types";
import { getCommandVelocity } from "@/utils";
import { mqtt } from "aws-iot-device-sdk-v2";
import {
  Dispatch,
  SetStateAction,
  useCallback,
  useEffect,
  useState,
} from "react";
import { Joystick, JoystickShape } from "react-joystick-component";
import { JoystickWrapper } from "../basic-components";

interface RobotControllerCardProps {
  connection: mqtt.MqttClientConnection | null;
  setTwist: Dispatch<SetStateAction<IotVelocityMessageType>>;
}

export function RobotControllerCard({
  connection,
  setTwist,
}: RobotControllerCardProps) {
  let counter = 0;
  let timer: NodeJS.Timeout | undefined;
  const [joystickStatus, setJoystickStatus] = useState<IJoystickUpdateEvent>();

  const onChangeJoystick = useCallback((val: any) => {
    console.log(val);
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
        console.log("Send twist cmd ");
        console.log(cmd);
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
          console.log("---------------");
          console.log("Send twist cmd -> " + counter);
          publishMessage(cmd);
          console.log("---------------");
        }, 1000);
        if (joystickStatus?.distance === 0 && timer !== undefined) {
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
    <JoystickWrapper joystickStatus={joystickStatus}>
      <Joystick
        throttle={250}
        baseShape={JoystickShape.Circle}
        size={CONTROLLER_CONTAINER_SIZE}
        stickSize={CONTROLLER_STICK_SIZE}
        baseColor={"#f1f1f1"}
        stickColor={"#FFFFFF"}
        move={onChangeJoystick}
        stop={onStopJoystick}
      />
    </JoystickWrapper>
  );
}
