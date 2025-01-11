import { Box, Container } from "@mui/material";
import { ReactNode } from "react";

interface MainLayoutProps {
  children: ReactNode;
}
export const MainLayout = ({ children }: MainLayoutProps) => {
  return (
    <Container
      maxWidth="xl"
      sx={{
        minHeight: "96vh",
        px: "0 !important",
        py: 4,
        mt: 11,
        overflow: "auto",
      }}
    >
      <Box
        sx={{
          opacity: 1,
          background: "#11cdef",
          height: "400px",
          width: "100vw",
          position: "absolute",
          zIndex: 1,
          top: 0,
          left: 0,
        }}
      />
      <Box
        sx={{
          display: "flex",
          flexDirection: "column",
          position: "relative",
          zIndex: 8,
        }}
      >
        {children}
      </Box>
    </Container>
  );
};

export default MainLayout;
