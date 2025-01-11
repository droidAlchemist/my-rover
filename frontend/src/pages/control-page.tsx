import { Box, Button, Divider, Drawer, Fab, Typography } from "@mui/material";
import Grid from "@mui/material/Grid2";
import {
  Assessment,
  AutoAwesome,
  Camera,
  ElectricCar,
  FlashlightOn,
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
  IOT_ROS2_TOPICS,
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
  const [open, setOpen] = useState(false);

  const toggleDrawer = (newOpen: boolean) => () => {
    setOpen(newOpen);
  };

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
      sx={{
        display: "flex",
        flexDirection: "column",
      }}
    >
      <Grid size={{ xs: 9 }}>
        <Grid container spacing={3}>
          <Grid size={{ xs: 12, md: 6, lg: 3 }}>
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
          <Grid size={{ xs: 12, md: 6, lg: 3 }}>
            <StatusInfoCard
              title="Camera 1"
              count={cameraActive ? "ON" : "OFF"}
              gradientColor="linear-gradient(310deg,#8A2387,#FF0080)"
              activeStatus={cameraActive}
              onAction={onClickCamera}
              icon={<Camera />}
              percentage={{
                color: "#F0000F",
                count: "x=1 y=1",
                text: " Pose",
              }}
            />
          </Grid>
          <Grid size={{ xs: 12, md: 6, lg: 3 }}>
            <TelemetryCard
              gradientColor="linear-gradient(310deg,rgb(80, 92, 184), #11cdef)"
              icon={<Assessment />}
              data={{
                voltage: voltageData,
                temperature: temperatureData,
              }}
            />
          </Grid>
          <Grid size={{ xs: 12, md: 6, lg: 3 }}>
            <OdometryCard
              gradientColor="linear-gradient(310deg, #a8ff78, #11cdef)"
              icon={<ElectricCar />}
              data={{
                odometry: odometryData,
              }}
            />
          </Grid>
          <Grid size={{ xs: 12 }}>
            {/* Show video data using kinesis from ROS2 */}
            <VideoStreaming />
          </Grid>
        </Grid>
      </Grid>
      <Box sx={{ position: "fixed", left: 30, top: "42.5%" }}>
        <Fab
          color={open ? "error" : "warning"}
          aria-label="Show/Hide Controller"
          onClick={toggleDrawer(!open)}
          size="large"
          variant="extended"
          sx={{ mt: 2 }}
        >
          <SportsEsports sx={{ mr: 1 }} />
          {open ? "Stop" : "Start"}
        </Fab>
      </Box>
      <MiniDrawer open={open}>
        <Box
          sx={{
            display: "flex",
            flexDirection: "column",
            width: 190,
            py: 2,
            gap: 1,
          }}
        >
          <Box
            sx={{
              display: "flex",
              alignItems: "center",
              pl: 1,
              gap: 1,
            }}
          >
            <AutoAwesome sx={{ fontSize: 20, color: "#11cdef" }} />
            <Typography
              variant="h6"
              sx={{
                fontSize: "0.875rem !important",
                fontWeight: 600,
                color: "#344767",
              }}
              component="div"
            >
              Robot Controller
            </Typography>
          </Box>
          <Divider />
          <RobotControllerCard connection={connection} />
        </Box>
      </MiniDrawer>
    </Box>
  );
};

export default ControlPage;
