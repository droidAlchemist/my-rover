import Box from "@mui/material/Box";
import Toolbar from "@mui/material/Toolbar";
import IconButton from "@mui/material/IconButton";
import Typography from "@mui/material/Typography";
import Menu from "@mui/material/Menu";
import MenuIcon from "@mui/icons-material/Menu";
import Container from "@mui/material/Container";
import Tooltip from "@mui/material/Tooltip";
import MenuItem from "@mui/material/MenuItem";
import AdbIcon from "@mui/icons-material/Adb";
import AccountCircle from "@mui/icons-material/AccountCircle";
import { useState, MouseEvent } from "react";
import { BarChart, Info, SportsEsports, History } from "@mui/icons-material";
import { PageMenuItem, ROUTES } from "@/types";
import { ListItemIcon, ListItemText, Tab, Tabs } from "@mui/material";
import { useLocation } from "react-router-dom";

const PROFILE_SETTINGS = ["Profile", "Logout"];

const PAGE_LIST: PageMenuItem[] = [
  {
    text: "Dashboard",
    icon: <BarChart color="warning" />,
    url: ROUTES.DASHBOARD_PAGE,
  },
  {
    text: "Control Robot",
    icon: <SportsEsports sx={{ color: "red" }} />,
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
  {
    text: "Settings",
    icon: <Info sx={{ color: "#11dfgf" }} />,
    url: ROUTES.DETAILS_PAGE,
  },
];

export const Header = () => {
  const [anchorElNav, setAnchorElNav] = useState<null | HTMLElement>(null);
  const [anchorElUser, setAnchorElUser] = useState<null | HTMLElement>(null);
  const { pathname } = useLocation();

  const handleOpenNavMenu = (event: MouseEvent<HTMLElement>) => {
    setAnchorElNav(event.currentTarget);
  };
  const handleOpenUserMenu = (event: MouseEvent<HTMLElement>) => {
    setAnchorElUser(event.currentTarget);
  };

  const handleCloseNavMenu = () => {
    setAnchorElNav(null);
  };

  const handleCloseUserMenu = () => {
    setAnchorElUser(null);
  };

  return (
    <Container maxWidth="xl" sx={{ position: "sticky", top: 0, zIndex: 10 }}>
      <Box
        sx={{
          marginTop: 2,
          paddingX: 3,
          width: 1,
          position: "absolute",
          left: 0,
          zIndex: 3,
          opacity: 1,
          background: "white",
          color: "#344767",
          borderRadius: "0.75rem",
          boxShadow:
            "rgba(0, 0, 0, 0.1) 0rem 0.25rem 0.375rem -0.0625rem, rgba(0, 0, 0, 0.06) 0rem 0.125rem 0.25rem -0.0625rem",
          backdropFilter: "saturate(200%) blur(30px)",
        }}
      >
        <Toolbar disableGutters>
          <AdbIcon sx={{ display: { xs: "none", md: "flex" }, mr: 1 }} />
          <Typography
            variant="h6"
            noWrap
            component="a"
            href="#app-bar-with-responsive-menu"
            sx={{
              mr: 3,
              display: { xs: "none", md: "flex" },
              fontFamily: "monospace",
              fontWeight: 700,
              letterSpacing: ".3rem",
              color: "inherit",
              textDecoration: "none",
            }}
          >
            My Rover
          </Typography>
          <Box sx={{ flexGrow: 1, display: { xs: "flex", md: "none" } }}>
            <IconButton
              size="large"
              aria-label="account of current user"
              aria-controls="menu-appbar"
              aria-haspopup="true"
              onClick={handleOpenNavMenu}
              color="inherit"
            >
              <MenuIcon />
            </IconButton>
            <Menu
              id="menu-appbar"
              anchorEl={anchorElNav}
              anchorOrigin={{
                vertical: "bottom",
                horizontal: "left",
              }}
              keepMounted
              transformOrigin={{
                vertical: "top",
                horizontal: "left",
              }}
              open={Boolean(anchorElNav)}
              onClose={handleCloseNavMenu}
              sx={{ display: { xs: "block", md: "none" } }}
            >
              {PAGE_LIST.map((page) => (
                <MenuItem key={page.url} onClick={handleCloseNavMenu}>
                  <ListItemIcon>{page.icon}</ListItemIcon>
                  <ListItemText>{page.text}</ListItemText>
                </MenuItem>
              ))}
            </Menu>
          </Box>
          <AdbIcon sx={{ display: { xs: "flex", md: "none" }, mr: 1 }} />
          <Typography
            variant="h5"
            noWrap
            component="a"
            href="#app-bar-with-responsive-menu"
            sx={{
              mr: 2,
              display: { xs: "flex", md: "none" },
              flexGrow: 1,
              fontFamily: "monospace",
              fontWeight: 700,
              letterSpacing: ".3rem",
              color: "inherit",
              textDecoration: "none",
            }}
          >
            My Rover
          </Typography>
          <Tabs
            value={pathname}
            textColor="inherit"
            indicatorColor="secondary"
            sx={{
              flexGrow: 1,
              display: {
                xs: "none",
                md: "flex",
                flexDirection: "row",
              },
            }}
          >
            {PAGE_LIST.map(({ icon, text, url }) => (
              <Tab
                label={text}
                icon={icon}
                href={url}
                value={url}
                iconPosition="start"
                sx={{ padding: 0, marginRight: 3, fontWeight: 600 }}
              />
            ))}
          </Tabs>
          <Box sx={{ flexGrow: 0 }}>
            <Tooltip title="Profile">
              <IconButton onClick={handleOpenUserMenu} sx={{ p: 0 }}>
                <AccountCircle />
              </IconButton>
            </Tooltip>
            <Menu
              sx={{ mt: "45px" }}
              id="menu-appbar"
              anchorEl={anchorElUser}
              anchorOrigin={{
                vertical: "top",
                horizontal: "right",
              }}
              keepMounted
              transformOrigin={{
                vertical: "top",
                horizontal: "right",
              }}
              open={Boolean(anchorElUser)}
              onClose={handleCloseUserMenu}
            >
              {PROFILE_SETTINGS.map((setting) => (
                <MenuItem key={setting} onClick={handleCloseUserMenu}>
                  <Typography sx={{ textAlign: "center" }}>
                    {setting}
                  </Typography>
                </MenuItem>
              ))}
            </Menu>
          </Box>
        </Toolbar>
      </Box>
    </Container>
  );
};
export default Header;
