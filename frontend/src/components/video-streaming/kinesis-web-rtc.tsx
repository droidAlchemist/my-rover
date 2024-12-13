import { useViewer } from "@/hooks";
import { Box, Typography } from "@mui/material";
import ReactPlayer from "react-player";

const {
  VITE_AWS_REGION,
  VITE_AWS_SECRET_KEY,
  VITE_AWS_ACCESS_ID,
  VITE_AWS_SIGNALING_CHANNEL_ARN,
} = import.meta.env;

const config = {
  credentials: {
    accessKeyId: VITE_AWS_ACCESS_ID,
    secretAccessKey: VITE_AWS_SECRET_KEY,
  },
  channelARN: VITE_AWS_SIGNALING_CHANNEL_ARN,
  region: VITE_AWS_REGION,
  // Enable below lines in case local media available
  // media: {
  //   audio: true,
  //   video: true,
  // },
};

export function KinesisWebRTC() {
  const { error, peer } = useViewer(config);

  // Display an error
  if (error) {
    return <Typography>An error occurred: {error.message}</Typography>;
  } else if (peer?.isWaitingForMedia) {
    return <Typography>Waiting to start playing</Typography>;
  }

  return (
    <Box sx={{ display: "flex", flexDirection: "column", gap: 2 }}>
      <ReactPlayer
        controls
        playing={!!peer?.media}
        playsinline={true}
        muted={true}
        url={peer?.media}
      />
    </Box>
  );
}

export default KinesisWebRTC;
