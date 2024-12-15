import { Box, Grid } from "@mui/material";
import { mqtt } from "aws-iot-device-sdk-v2";

import { TelemetryItem } from "./telemetry-item";
import { useCallback, useEffect, useState } from "react";
import { IOT_ROS2_TOPICS, IotSensorMessageType } from "@/types";

interface TelemetryProps {
  connection: mqtt.MqttClientConnection | null;
}

const DEFAULT_DATA = {
  battery: 0,
  temperature: 0,
  velocity: 0,
  distance: 0,
};

export function Telemetry({ connection }: TelemetryProps) {
  const [telemetryData, setTelemetryData] =
    useState<IotSensorMessageType>(DEFAULT_DATA);

  const setMessageHandler = useCallback((message: string) => {
    if (message) {
      const messageObject: IotSensorMessageType = JSON.parse(message);
      const entries = Object.entries(messageObject);
      let calculatedTelemetry: IotSensorMessageType = {
        ...DEFAULT_DATA,
      };
      for (let [key, value] of entries) {
        if (value > 0) {
          calculatedTelemetry[key as keyof IotSensorMessageType] = value / 100;
        }
      }
      setTelemetryData(calculatedTelemetry);
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
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        gap: 2,
        flexGrow: 1,
      }}
    >
      <Grid container columnSpacing={4} columns={4}>
        <Grid item xs={1}>
          <TelemetryItem
            title="Battery"
            nrOfLevels={20}
            percent={telemetryData?.battery}
          />
        </Grid>
        <Grid item xs={1}>
          <TelemetryItem
            title="Temperature"
            nrOfLevels={420}
            arcsLength={[0.3, 0.5, 0.2]}
            colors={["#5BE12C", "#F5CD19", "#EA4228"]}
            percent={telemetryData?.temperature}
            arcPadding={0.02}
            formatTextValue={(value) => value + " C"}
          />
        </Grid>
        <Grid item xs={1}>
          <TelemetryItem
            title="Velocity"
            nrOfLevels={10}
            percent={telemetryData?.velocity}
            formatTextValue={(value) => value + " km/hr"}
          />
        </Grid>
        <Grid item xs={1}>
          <TelemetryItem
            title="Distance"
            percent={telemetryData?.distance}
            formatTextValue={(value) => value + " m"}
          />
        </Grid>
      </Grid>
    </Box>
  );
}
