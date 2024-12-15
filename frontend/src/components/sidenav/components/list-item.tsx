import {
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
} from "@mui/material";
import { ReactNode } from "react";
import { Link, useLocation } from "react-router-dom";

interface NavListItemProps {
  text: string;
  url: string;
  icon?: ReactNode;
}

export const NavListItem = ({ text, url, icon }: NavListItemProps) => {
  const { pathname } = useLocation();
  return (
    <ListItem disablePadding>
      <ListItemButton selected={url === pathname} component={Link} to={url}>
        <ListItemIcon>{icon}</ListItemIcon>
        <ListItemText primary={text} />
      </ListItemButton>
    </ListItem>
  );
};

export default NavListItem;
