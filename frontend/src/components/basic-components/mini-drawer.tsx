import Box from "@mui/material/Box";
import { ReactNode } from "react";
import { Drawer } from "@mui/material";

interface MiniDrawerProps {
  children: ReactNode;
  open: boolean;
}

export function MiniDrawer({ children, open }: MiniDrawerProps) {
  return (
    <Box sx={{ display: "flex" }}>
      <Drawer
        open={open}
        variant="persistent"
        anchor="right"
        PaperProps={{
          sx: {
            height: 250,
            top: "35%",
            backgroundColor: "transparent",
            border: "none",
          },
        }}
      >
        {children}
      </Drawer>
    </Box>
  );
}

export default MiniDrawer;
