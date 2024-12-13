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
          backgroundColor: "black",
        }}
      >
        <KinesisWebRTC />
      </Box>
    </NormalCard>
  );
}
