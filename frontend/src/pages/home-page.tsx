import { Container, Box } from "@mui/material";
import { IotCoreContainer } from "@/components";

const HomePage = () => {
  return (
    <Container sx={{ position: "relative" }}>
      <Box sx={{ display: "flex", flexDirection: "column", gap: 2 }}>
        {/* Show IOT core implementations */}
        <IotCoreContainer />
      </Box>
    </Container>
  );
};

export default HomePage;
