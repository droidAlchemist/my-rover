import { Box } from "@mui/material";
import Grid from "@mui/material/Grid";
import {
  BatteryFull,
  Camera,
  FlashlightOn,
  Navigation,
} from "@mui/icons-material";
import {
  VideoStreaming,
  StatusInfoCard,
  RobotControllerCard,
  LidarCard,
} from "@/components";
import { useAwsCredentials, useAwsIotMqtt } from "@/hooks";
import { useCallback } from "react";

export const ControlPage = () => {
  const credentials = useAwsCredentials();
  const connection = useAwsIotMqtt(credentials);

  const onClickLed = useCallback((value: boolean) => {
    console.log(value);
  }, []);
  const onClickCamera = useCallback((value: boolean) => {
    console.log(value);
  }, []);
  const onClickLidar = useCallback((value: boolean) => {
    console.log(value);
  }, []);

  return (
    <Box sx={{ position: "relative" }}>
      <Grid container spacing={3} mb={3}>
        <Grid item xs={12} md={6} lg={3}>
          <StatusInfoCard
            title="Battery Status"
            count="55%"
            gradientColor="linear-gradient(310deg, #a8ff78, #11cdef)"
            icon={<BatteryFull />}
            percentage={{
              color: "#F0000F",
              count: "1 hour",
              text: "remaining",
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
              color: "green",
              count: "",
              text: "Turn off to save battery",
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
            // onAction={onClickLidar}
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
