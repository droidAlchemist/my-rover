import * as React from "react";
import { styled, Theme, CSSObject, alpha } from "@mui/material/styles";
import Box from "@mui/material/Box";
import MuiDrawer from "@mui/material/Drawer";
import List from "@mui/material/List";
import Divider from "@mui/material/Divider";
import { DRAWER_WIDTH, ROUTES } from "@/types";
import { BarChart, History, Info, SportsEsports } from "@mui/icons-material";
import { SidebarHeader, NavListItem } from "./components";

const MENU_LIST = [
  {
    text: "Dashboard",
    icon: <BarChart color="warning" />,
    url: ROUTES.DASHBOARD_PAGE,
  },
  {
    text: "Control Robot",
    icon: <SportsEsports color="error" />,
    url: ROUTES.CONTROL_PAGE,
  },
  {
    text: "Log",
    icon: <History sx={{ color: "orange" }} />,
    url: ROUTES.LOG_PAGE,
  },
  {
    text: "Details",
    icon: <Info sx={{ color: "#11dfgf" }} />,
    url: ROUTES.DETAILS_PAGE,
  },
];

interface SidenavProps {
  children: React.ReactNode;
}

export function Sidenav({ children }: SidenavProps) {
  const [open, setOpen] = React.useState(true);

  return (
    <Box sx={{ display: "flex" }}>
      <Drawer variant="permanent" open={open}>
        <SidebarHeader open={open} setOpen={setOpen} />
        <Divider />
        <List
          sx={{
            "& .MuiListItemIcon-root": {
              minWidth: 30,
            },
            "& .MuiListItemIcon-root svg": {
              fontSize: 20,
            },
            "& .MuiListItemText-root": {
              ...(!open && {
                display: "none",
              }),
            },
            "& .MuiListItemText-root span": {
              fontSize: "0.875rem",
              fontWeight: 400,
              color: "#344767",
            },
            "& .Mui-selected": {
              backgroundColor: `${alpha("#11cdef", 0.2)} !important`,
              "& .MuiListItemText-root span": {
                fontWeight: 600,
              },
            },
          }}
        >
          {MENU_LIST.map((item) => (
            <NavListItem key={item.url} {...item} />
          ))}
        </List>
      </Drawer>
      <Box component="main" sx={{ flexGrow: 1, p: 3 }}>
        {children}
      </Box>
    </Box>
  );
}

export default Sidenav;

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
    height: "calc(-3rem + 100vh)",
    margin: "1.5rem 0px 1rem 1rem",
    borderRadius: "1rem",
    boxShadow:
      "0 4px 8px 0 rgba(0, 0, 0, 0.1), 0 6px 20px 0 rgba(0, 0, 0, 0.1)",
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
