import { Box } from "@mui/material";
import { ReactNode } from "react";

interface MainLayoutProps {
  children: ReactNode;
}
function MainLayout({ children }: MainLayoutProps) {
  return (
    <Box
      sx={{
        p: 3,
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
          background: "rgb(17, 205, 239)",
        }}
      />
      {children}
    </Box>
  );
}

export default MainLayout;
