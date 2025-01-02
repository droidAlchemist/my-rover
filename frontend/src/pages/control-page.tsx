import { Box } from "@mui/material";
import Grid from "@mui/material/Grid2";
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
import { getBatteryPercentage } from "@/utils";

const { VOLTAGE, ODOMETRY, TEMPERATURE } = TELEMETRY_MESSAGE_TYPES;

export const ControlPage = () => {
  const credentials = useAwsCredentials();
  const connection = useAwsIotMqtt(credentials);
  const [voltageData, setVoltageData] = useState<string>();
  const [temperatureData, setTemperatureData] = useState<string>();
  const [batteryPercent, setBatteryPercent] = useState<string>("0%");
  const [odometryData, setOdometryData] = useState<number[]>();

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
          const volt = messageObject.data as number;
          const calculatedPercent = getBatteryPercentage(volt * 100);
          setVoltageData(String(volt));
          setBatteryPercent(String(calculatedPercent) + "%");
          break;
        case TEMPERATURE:
          setTemperatureData(String(messageObject.data));
          break;
        case ODOMETRY:
          setOdometryData(messageObject.data as number[]);
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
        mqtt.QoS.AtMostOnce,
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
    <Box
      sx={{ display: "flex", flexDirection: "column", position: "relative" }}
    >
      <Grid container spacing={3}>
        <Grid size={{ xs: 9 }}>
          <Grid container spacing={3}>
            <Grid size={{ xs: 12, md: 6, lg: 4 }}>
              <StatusInfoCard
                title="LED Light"
                count="OFF"
                gradientColor="linear-gradient(310deg, #fb6340, #fbb140)"
                activeStatus={false}
                onAction={onClickLed}
                icon={<FlashlightOn />}
                percentage={{
                  color: "#F0000F",
                  count: batteryPercent,
                  text: "battery remaining.",
                }}
              />
            </Grid>
            <Grid size={{ xs: 12, md: 6, lg: 4 }}>
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
            <Grid size={{ xs: 12, md: 6, lg: 4 }}>
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
            <Grid size={{ xs: 12 }}>
              {/* Show video data using kinesis from ROS2 */}
              {/* <VideoStreaming /> */}
            </Grid>
          </Grid>
        </Grid>
        <Grid size={{ xs: 3 }}>
          <Box sx={{ display: "flex", flexDirection: "column", gap: 4 }}>
            <TelemetryCard
              gradientColor="linear-gradient(310deg, #a8ff78, #11cdef)"
              icon={<Assessment />}
              data={{
                voltage: voltageData,
                odometry: odometryData,
                temperature: temperatureData,
              }}
            />
            {/* Show Lidar data */}
            <LidarCard image="lidar.jpg" />
            {/* Control robot by sending x,z pos using IOT Core to ROS2 */}
            <RobotControllerCard connection={connection} />
          </Box>
        </Grid>
      </Grid>
    </Box>
  );
};

export default ControlPage;
