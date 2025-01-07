import { useViewer } from "@/hooks";
import { Box, Typography } from "@mui/material";
import { useEffect, useState } from "react";
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
};

export function KinesisWebRTC() {
  const { error, peer } = useViewer(config);
  const [mediaProps, setMediaProps] = useState<MediaTrackConstraints>();

  useEffect(() => {
    if (peer?.media) {
      const tracks = peer?.media?.getVideoTracks();
      tracks && tracks.length > 0 && setMediaProps(tracks[0].getConstraints());
    }
  }, [peer?.media]);

  return (
    <Box sx={{ display: "flex", flexDirection: "column", gap: 2 }}>
      {error && <Typography>{error.message}</Typography>}
      {peer?.isWaitingForMedia && (
        <Typography>Waiting to start playing</Typography>
      )}
      <ReactPlayer
        controls
        playing={!!peer?.media}
        playsinline={true}
        muted={true}
        url={peer?.media}
        width={mediaProps?.width as number}
        height={mediaProps?.height as number}
      />
    </Box>
  );
}

export default KinesisWebRTC;
