import { Container, Box } from "@mui/material";
import { IotCoreContainer } from "@/components";

export const DashboardPage = () => {
  return (
    <Container sx={{ position: "relative" }}>
      <Box sx={{ display: "flex", flexDirection: "column", gap: 2 }}>
        {/* Show IOT core implementations */}
        <IotCoreContainer />

        <Box sx={{ display: "flex", flexDirection: "row", gap: 2 }}></Box>
      </Box>
    </Container>
  );
};

export default DashboardPage;
