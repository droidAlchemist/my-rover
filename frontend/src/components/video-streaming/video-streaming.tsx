import KinesisWebRTC from "./kinesis-web-rtc";
import NormalCard from "../cards/normal-card";
import { Box } from "@mui/material";

export function VideoStreaming() {
  return (
    <NormalCard>
      <Box
        sx={{
          display: "flex",
          flexDirection: "column",
          alignItems: "center",
          backgroundColor: "black",
          minHeight: 600,
        }}
      >
        <KinesisWebRTC />
      </Box>
    </NormalCard>
  );
}
