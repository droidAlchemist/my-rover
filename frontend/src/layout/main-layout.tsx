import { Box } from "@mui/material";
import { ReactNode } from "react";

interface MainLayoutProps {
  children: ReactNode;
}
export const MainLayout = ({ children }: MainLayoutProps) => {
  return (
    <Box
      sx={{
        px: 3,
        display: "flex",
      }}
    >
      <Box
        height="300px"
        width="100vw"
        position="absolute"
        top={0}
        left={0}
        sx={{
          opacity: 1,
          background: "#11cdef",
        }}
      />
      {children}
    </Box>
  );
};

export default MainLayout;
