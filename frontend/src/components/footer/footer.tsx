import { Box, Container, Typography } from "@mui/material";

export function Footer() {
  return (
    <Box
      sx={{
        width: "100%",
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
            A project using ROS2 packages, IOT Core and Kinesis WebRTC to
            control the rover. Copyright © {new Date().getFullYear()}.
          </Typography>
        </Box>
      </Container>
    </Box>
  );
}
