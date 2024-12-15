import { Container, Box } from "@mui/material";
import { InfoCard, RobotSettingsCard } from "@/components";

export const DetailsPage = () => {
  return (
    <Container sx={{ position: "relative" }}>
      <Box sx={{ display: "flex", flexDirection: "column", gap: 2 }}>
        <Box sx={{ display: "flex", flexDirection: "row", gap: 2 }}>
          <InfoCard
            title="Specification"
            description="Hi, I'm Alec Thompson, Decisions: If you can't decide, the answer is no. If two equally difficult paths, choose the one more painful in the short term (pain avoidance is creating an illusion of equality)."
            info={{
              fullName: "Alec M. Thompson",
              mobile: "(44) 123 1234 123",
              email: "alecthompson@mail.com",
              location: "USA",
            }}
          />
          <InfoCard
            title="Robot information"
            description="Hi, I'm Alec Thompson, Decisions: If you can't decide, the answer is no. If two equally difficult paths, choose the one more painful in the short term (pain avoidance is creating an illusion of equality)."
            info={{
              fullName: "Alec M. Thompson",
              mobile: "(44) 123 1234 123",
              email: "alecthompson@mail.com",
              location: "USA",
            }}
          />
        </Box>
        <RobotSettingsCard />
      </Box>
    </Container>
  );
};

export default DetailsPage;
