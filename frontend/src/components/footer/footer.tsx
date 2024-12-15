import { Box, Container, Typography } from "@mui/material";

export function Footer() {
  return (
    <Box
      sx={{
        width: "100%",
        // position: "absolute",
        // bottom: 0,
        backgroundColor: "transparent",
      }}
      component="footer"
    >
      <Container maxWidth="lg">
        <Box
          sx={{
            flexGrow: 1,
            justifyContent: "center",
            display: "flex",
            my: 1,
          }}
        >
          <Typography variant="caption" color="initial">
            A demo project using IOT Core, Kinesis WebRTC and ROS2 custom
            packages. Copyright © {new Date().getFullYear()}.
          </Typography>
        </Box>
      </Container>
    </Box>
  );
}
