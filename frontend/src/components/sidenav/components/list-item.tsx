import {
  ListItem,
  ListItemButton,
  ListItemIcon,
  ListItemText,
} from "@mui/material";
import { ReactNode } from "react";

interface NavListItemProps {
  text: string;
  link: string;
  icon?: ReactNode;
}

export const NavListItem = ({ text, link, icon }: NavListItemProps) => {
  return (
    <ListItem key={text} disablePadding sx={{ display: "block" }}>
      <ListItemButton href={link}>
        <ListItemIcon>{icon}</ListItemIcon>
        <ListItemText primary={text} />
      </ListItemButton>
    </ListItem>
  );
};

export default NavListItem;
