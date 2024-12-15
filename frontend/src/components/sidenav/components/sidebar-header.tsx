import { AutoAwesome } from "@mui/icons-material";
import { Box, Typography, IconButton, styled } from "@mui/material";
import ChevronLeftIcon from "@mui/icons-material/ChevronLeft";
import MenuIcon from "@mui/icons-material/Menu";

interface SidebarHeaderProps {
  open: boolean;
  setOpen: React.Dispatch<React.SetStateAction<boolean>>;
}

export const SidebarHeader = ({ open, setOpen }: SidebarHeaderProps) => {
  const handleDrawerClose = () => {
    setOpen(false);
  };
  const handleDrawerOpen = () => {
    setOpen(true);
  };

  return (
    <DrawerHeader>
      {open && (
        <>
          <Box sx={{ display: "flex", alignItems: "center", pl: 1, gap: 1 }}>
            <AutoAwesome sx={{ fontSize: 20, color: "#11cdef" }} />
            <Typography
              variant="h6"
              sx={{
                fontSize: "0.875rem !important",
                fontWeight: 600,
                color: "#344767",
              }}
              component="div"
            >
              My Rover
            </Typography>
          </Box>
          <IconButton onClick={handleDrawerClose}>
            <ChevronLeftIcon />
          </IconButton>
        </>
      )}
      {!open && (
        <IconButton onClick={handleDrawerOpen}>
          <MenuIcon />
        </IconButton>
      )}
    </DrawerHeader>
  );
};

const DrawerHeader = styled("div")(({ theme }) => ({
  display: "flex",
  alignItems: "center",
  justifyContent: "space-between",
  padding: theme.spacing(0, 1),
  ...theme.mixins.toolbar,
}));
