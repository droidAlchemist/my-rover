import { Container, Box } from "@mui/material";

export const DashboardPage = () => {
  return (
    <Container sx={{ position: "relative" }}>
      <Box sx={{ display: "flex", flexDirection: "column", gap: 2 }}>
        No Data
        <Box sx={{ display: "flex", flexDirection: "row", gap: 2 }}></Box>
      </Box>
    </Container>
  );
};

export default DashboardPage;
