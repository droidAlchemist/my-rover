import { Box } from "@mui/material";
import Grid from "@mui/material/Grid2";
import {
  Assessment,
  Camera,
  ElectricCar,
  FlashlightOn,
  NavigationRounded,
  SportsEsports,
} from "@mui/icons-material";
import {
  VideoStreaming,
  StatusInfoCard,
  RobotControllerCard,
  TelemetryCard,
  OdometryCard,
  MiniDrawer,
} from "@/components";
import { useAwsCredentials, useAwsIotMqtt } from "@/hooks";
import { useCallback, useEffect, useState } from "react";
import {
  DEFAULT_TWIST_MESSAGE,
  IOT_ROS2_TOPICS,
  IotVelocityMessageType,
  TELEMETRY_MESSAGE_TYPES,
  TelemetryMessageType,
} from "@/types";
import { mqtt } from "aws-iot-device-sdk-v2";
import { getBatteryPercentage, getCameraCommand } from "@/utils";

const { VOLTAGE, ODOMETRY, TEMPERATURE } = TELEMETRY_MESSAGE_TYPES;

export const ControlPage = () => {
  const credentials = useAwsCredentials();
  const connection = useAwsIotMqtt(credentials);
  const [cameraActive, setCameraActive] = useState<boolean>();
  const [voltageData, setVoltageData] = useState<string>();
  const [temperatureData, setTemperatureData] = useState<string>();
  const [batteryPercent, setBatteryPercent] = useState<string>("0%");
  const [odometryData, setOdometryData] = useState<number[]>();
  const [twist, setTwist] = useState<IotVelocityMessageType>({
    ...DEFAULT_TWIST_MESSAGE,
  });
  const [openJoystickDrawer, setJoystickDrawer] = useState(true);

  const toggleDrawer = useCallback(
    (newOpen: boolean) => () => {
      setJoystickDrawer(newOpen);
    },
    [],
  );

  const onClickLed = useCallback((value: boolean) => {
    console.log(value);
  }, []);

  const onClickCamera = useCallback(
    (value: boolean) => {
      setCameraActive(value);
      const cmd = getCameraCommand(value);
      if (cmd && connection) {
        console.log(cmd);
        connection.publish(
          IOT_ROS2_TOPICS.CAMERA,
          JSON.stringify(cmd),
          mqtt.QoS.AtMostOnce,
        );
      }
    },
    [connection],
  );

  const setMessageHandler = useCallback((message: string) => {
    if (message) {
      const messageObject: TelemetryMessageType = JSON.parse(message);
      switch (messageObject.type) {
        case VOLTAGE:
          const volt = messageObject.data as number;
          const calculatedPercent = getBatteryPercentage(volt);
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
      sx={{
        display: "flex",
        flexDirection: "column",
      }}
    >
      <Grid size={{ xs: 9 }}>
        <Grid container spacing={3}>
          <Grid size={{ xs: 12, md: 6, lg: 2 }}>
            <StatusInfoCard
              title="LED Light"
              statusText="OFF"
              gradientColor="linear-gradient(310deg, #fb6340, #fbb140)"
              activeStatus={false}
              onAction={onClickLed}
              icon={<FlashlightOn />}
              details={{
                color: "#F0000F",
                mainText: batteryPercent,
                subText: "battery remaining.",
              }}
            />
          </Grid>
          <Grid size={{ xs: 12, md: 6, lg: 2 }}>
            <StatusInfoCard
              title="Camera 1"
              statusText={cameraActive ? "ON" : "OFF"}
              gradientColor="linear-gradient(310deg,#8A2387,#FF0080)"
              activeStatus={cameraActive}
              onAction={onClickCamera}
              icon={<Camera />}
              details={{
                color: "#F0000F",
                mainText: "Damaged",
                subText: " _",
              }}
            />
          </Grid>
          <Grid size={{ xs: 12, md: 6, lg: 2 }}>
            <StatusInfoCard
              title="LIDAR"
              statusText={"OFF"}
              gradientColor="linear-gradient(310deg,#8A2387,#0F0080)"
              icon={<NavigationRounded />}
              details={{
                color: "#F0000F",
                mainText: "NA",
                subText: " _",
              }}
            />
          </Grid>
          <Grid size={{ xs: 12, md: 6, lg: 2 }}>
            <TelemetryCard
              gradientColor="linear-gradient(310deg,rgb(80, 92, 184), #11cdef)"
              icon={<Assessment />}
              data={{
                voltage: voltageData,
                temperature: temperatureData,
              }}
            />
          </Grid>
          <Grid size={{ xs: 12, md: 6, lg: 2 }}>
            <OdometryCard
              gradientColor="linear-gradient(310deg, #a8ff78, #11cdef)"
              icon={<ElectricCar />}
              data={{
                odometry: odometryData,
              }}
            />
          </Grid>
          <Grid size={{ xs: 12, md: 6, lg: 2 }}>
            <StatusInfoCard
              title="Joystick"
              statusText={openJoystickDrawer ? "Active" : "Inactive"}
              gradientColor="linear-gradient(310deg,#FA2387,#9A0080)"
              icon={<SportsEsports />}
              activeStatus={openJoystickDrawer}
              onAction={toggleDrawer(!openJoystickDrawer)}
              details={{
                color: "#F0000F",
                mainText: `x = ${twist?.linear.x}, z = ${twist?.angular.z}`,
                subText: " ",
              }}
            />
          </Grid>
          <Grid size={{ xs: 12 }}>
            {/* Show video data using kinesis from ROS2 */}
            {/* <VideoStreaming /> */}
          </Grid>
        </Grid>
      </Grid>
      <MiniDrawer open={openJoystickDrawer}>
        <Box
          sx={{
            display: "flex",
            flexDirection: "column",
            width: 190,
            py: 2,
            gap: 1,
          }}
        >
          <RobotControllerCard setTwist={setTwist} connection={connection} />
        </Box>
      </MiniDrawer>
    </Box>
  );
};

export default ControlPage;
