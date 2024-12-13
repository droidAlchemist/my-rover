import * as React from "react";
import { styled, Theme, CSSObject } from "@mui/material/styles";
import Box from "@mui/material/Box";
import MuiDrawer from "@mui/material/Drawer";
import List from "@mui/material/List";
import Divider from "@mui/material/Divider";
import BroadcastOnHomeIcon from "@mui/icons-material/BroadcastOnHome";
import { DRAWER_WIDTH, ROUTES } from "@/types";
import { SportsEsports } from "@mui/icons-material";
import NavListItem from "./components/list-item";
import { SidebarHeader } from "./components/sidebar-header";

const openedMixin = (theme: Theme): CSSObject => ({
  width: DRAWER_WIDTH,
  transition: theme.transitions.create("width", {
    easing: theme.transitions.easing.sharp,
    duration: theme.transitions.duration.enteringScreen,
  }),
  overflowX: "hidden",
});

const closedMixin = (theme: Theme): CSSObject => ({
  transition: theme.transitions.create("width", {
    easing: theme.transitions.easing.sharp,
    duration: theme.transitions.duration.leavingScreen,
  }),
  overflowX: "hidden",
  width: `calc(${theme.spacing(7)} + 1px)`,
  [theme.breakpoints.up("sm")]: {
    width: `calc(${theme.spacing(8)} + 1px)`,
  },
});

const Drawer = styled(MuiDrawer, {
  shouldForwardProp: (prop) => prop !== "open",
})(({ theme }) => ({
  width: DRAWER_WIDTH,
  flexShrink: 0,
  whiteSpace: "nowrap",
  boxSizing: "border-box",
  "& .MuiDrawer-paper": {
    height: "calc(-2rem + 100vh)",
    margin: "1rem 0px 1rem 1rem",
    borderRadius: "1rem",
  },

  variants: [
    {
      props: ({ open }) => open,
      style: {
        ...openedMixin(theme),
        "& .MuiDrawer-paper": openedMixin(theme),
      },
    },
    {
      props: ({ open }) => !open,
      style: {
        ...closedMixin(theme),
        "& .MuiDrawer-paper": closedMixin(theme),
      },
    },
  ],
}));

const sidebarList = [
  {
    text: "Telemetry",
    icon: <BroadcastOnHomeIcon color="success" />,
    link: ROUTES.HOME_PAGE,
  },
  {
    text: "Control Robot",
    icon: <SportsEsports color="error" />,
    link: ROUTES.CONTROL_PAGE,
  },
];

interface SidenavProps {
  children: React.ReactNode;
}

export default function Sidenav({ children }: SidenavProps) {
  const [open, setOpen] = React.useState(true);

  return (
    <Box sx={{ display: "flex" }}>
      <Drawer variant="permanent" open={open}>
        <SidebarHeader open={open} setOpen={setOpen} />
        <Divider />
        <List
          sx={{
            // minHeight: 48,
            "& .MuiListItemIcon-root": {
              minWidth: 40,
            },
            "& .MuiListItemText-root": {
              ...(!open && {
                display: "none",
              }),
            },
          }}
        >
          {sidebarList.map((item) => (
            <NavListItem {...item} />
          ))}
        </List>
      </Drawer>
      <Box component="main" sx={{ flexGrow: 1, p: 3 }}>
        {children}
      </Box>
    </Box>
  );
}
