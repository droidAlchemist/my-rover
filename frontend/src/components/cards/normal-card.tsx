import Card from "@mui/material/Card";
import { ReactNode } from "react";

interface NormalCardProps {
  children: ReactNode;
}

function NormalCard({ children }: NormalCardProps) {
  return <Card>{children}</Card>;
}

export default NormalCard;
