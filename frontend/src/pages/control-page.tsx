import { Box } from "@mui/material";
import Grid from "@mui/material/Grid";
import {
  Assessment,
  Camera,
  FlashlightOn,
  Navigation,
} from "@mui/icons-material";
import {
  VideoStreaming,
  StatusInfoCard,
  RobotControllerCard,
  LidarCard,
  TelemetryCard,
} from "@/components";
import { useAwsCredentials, useAwsIotMqtt } from "@/hooks";
import { useCallback, useEffect, useState } from "react";
import {
  IOT_ROS2_TOPICS,
  TELEMETRY_MESSAGE_TYPES,
  TelemetryMessageType,
} from "@/types";
import { mqtt } from "aws-iot-device-sdk-v2";

const { VOLTAGE } = TELEMETRY_MESSAGE_TYPES;

export const ControlPage = () => {
  const credentials = useAwsCredentials();
  const connection = useAwsIotMqtt(credentials);
  const [voltageData, setVoltageData] = useState<number>();

  const onClickLed = useCallback((value: boolean) => {
    console.log(value);
  }, []);
  const onClickCamera = useCallback((value: boolean) => {
    console.log(value);
  }, []);

  const setMessageHandler = useCallback((message: string) => {
    if (message) {
      const messageObject: TelemetryMessageType = JSON.parse(message);
      switch (messageObject.type) {
        case VOLTAGE:
          setVoltageData(Number(messageObject.data));
          break;
        default:
          break;
      }
    }
  }, []);

  useEffect(() => {
    if (connection) {
      connection.subscribe(
        IOT_ROS2_TOPICS.TELEMETRY,
        mqtt.QoS.AtLeastOnce,
        (_topic, payload) => {
          const message = new TextDecoder("utf-8").decode(
            new Uint8Array(payload),
          );
          setMessageHandler(message);
        },
      );
    }
  }, [connection]);

  return (
    <Box sx={{ position: "relative" }}>
      <Grid container spacing={3} mb={3}>
        <Grid item xs={12} md={6} lg={3}>
          <TelemetryCard
            gradientColor="linear-gradient(310deg, #a8ff78, #11cdef)"
            icon={<Assessment />}
            data={{
              voltage: voltageData,
              temperature: 0,
            }}
          />
        </Grid>
        <Grid item xs={12} md={6} lg={3}>
          <StatusInfoCard
            title="LED Light"
            count="OFF"
            gradientColor="linear-gradient(310deg, #fb6340, #fbb140)"
            activeStatus={false}
            onAction={onClickLed}
            icon={<FlashlightOn />}
            percentage={{
              color: "#F0000F",
              count: "55%",
              text: "battery remaining.",
            }}
          />
        </Grid>
        <Grid item xs={12} md={6} lg={3}>
          <StatusInfoCard
            title="Camera 1"
            count="OFF"
            gradientColor="linear-gradient(310deg,#8A2387,#FF0080)"
            activeStatus={false}
            onAction={onClickCamera}
            icon={<Camera />}
            percentage={{
              color: "#F0000F",
              count: "x=1 y=1",
              text: " Pose",
            }}
          />
        </Grid>
        <Grid item xs={12} md={6} lg={3}>
          <StatusInfoCard
            title="LIDAR"
            count="OFF"
            gradientColor="linear-gradient(310deg,#1171ef,#11cdef)"
            activeStatus={false}
            icon={<Navigation />}
            percentage={{
              color: "error",
              count: "Unavailable!",
              text: "Please check wiring",
            }}
          />
        </Grid>
      </Grid>
      <Grid container spacing={3} mb={3}>
        <Grid item xs={12} lg={9}>
          {/* Show video data using kinesis from ROS2 */}
          <VideoStreaming />
        </Grid>
        <Grid item xs={12} lg={3}>
          <LidarCard image="lidar.jpg" />
          {/* Control robot by sending x,z pos using IOT Core to ROS2 */}
          <RobotControllerCard connection={connection} />
        </Grid>
      </Grid>
    </Box>
  );
};

export default ControlPage;
