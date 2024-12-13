import { Box, Typography } from "@mui/material";
import { DRAWER_WIDTH } from "@/types";

export function Header() {
  return (
    <Box
      sx={{
        position: "absolute",
        left: `calc(${DRAWER_WIDTH}px + 40px)`,
      }}
    >
      <Typography variant="h6" noWrap component="div" color="white">
        Dashboard
      </Typography>
    </Box>
  );
}
