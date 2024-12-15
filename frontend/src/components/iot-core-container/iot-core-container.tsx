import { Box } from "@mui/material";
import { Telemetry } from "./telemetry/telemetry";
import { useAwsCredentials, useAwsIotMqtt } from "@/hooks";

export function IotCoreContainer() {
  const credentials = useAwsCredentials();
  const connection = useAwsIotMqtt(credentials);

  return (
    <Box sx={{ display: "flex", flexDirection: "row" }}>
      {/* Show Telemtry data using IOT Core from ROS2 */}
      <Telemetry connection={connection} />
    </Box>
  );
}
