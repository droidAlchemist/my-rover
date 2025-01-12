import { Box, Typography } from "@mui/material";
import Card from "@mui/material/Card";
import Grid from "@mui/material/Grid2";
import { ReactNode, useState } from "react";
import { IOSSwitch } from "../basic-components";

type CardDetails = {
  color: string;
  mainText: string;
  subText: string;
};

interface StatusInfoCardProps {
  title: string;
  statusText: string;
  gradientColor: string;
  details: CardDetails;
  icon: ReactNode;
  activeStatus?: boolean;
  onAction?: (value: boolean) => void;
}

export function StatusInfoCard({
  title,
  statusText,
  details,
  icon,
  gradientColor,
  activeStatus = false,
  onAction,
}: StatusInfoCardProps) {
  const [checked, setChecked] = useState(activeStatus);

  const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setChecked(event.target.checked);
    onAction?.(event.target.checked);
  };

  return (
    <Card
      sx={{
        minHeight: "120px",
        "&.MuiPaper-root": {
          borderRadius: "1rem",
        },
      }}
    >
      <Box sx={{ bgcolor: "white", color: "#344767" }}>
        <Box p={2}>
          <Grid container>
            <Grid size={{ xs: 8 }}>
              <Box
                sx={{
                  lineHeight: 1,
                  background: "transparent",
                  color: "#344767",
                }}
              >
                <Typography
                  variant="button"
                  sx={{
                    fontSize: "0.875rem",
                    lineHeight: 1.5,
                    fontWeight: 600,
                    textTransform: "uppercase",
                    color: "#67748e",
                  }}
                >
                  {title}
                </Typography>
                <Typography
                  variant="h5"
                  sx={{
                    margin: "0px 0px 8px",
                    fontSize: "1.25rem",
                    lineHeight: 1.375,
                    color: "#344767",
                    fontWeight: 700,
                  }}
                >
                  {statusText}
                </Typography>
              </Box>
            </Grid>
            <Grid size={{ xs: 4 }}>
              <Box
                sx={{
                  width: "3rem",
                  height: "3rem",
                  display: "flex",
                  justifyContent: "center",
                  alignItems: "center",
                  marginLeft: "auto",
                  opacity: 1,
                  background: gradientColor,
                  color: "white",
                  borderRadius: "10rem",
                }}
              >
                <Box
                  sx={{
                    fontSize: "1.125rem",
                    display: "grid",
                  }}
                >
                  {icon}
                </Box>
              </Box>
            </Grid>
          </Grid>
          <Box
            sx={{
              display: "flex",
              flexDirection: "row",
              justifyContent: "space-between",
            }}
          >
            <Box
              sx={{
                display: "flex",
                flexDirection: "row",
              }}
            >
              <Typography
                sx={{
                  display: "flex",
                  fontSize: "0.875rem",
                  fontWeight: 600,
                  color: details.color,
                }}
              >
                {details.mainText}
              </Typography>
              <Typography
                variant="body2"
                sx={{
                  margin: "0 0 0 4px",
                  fontSize: "0.875rem",
                  lineHeight: 1.6,
                  color: "#67748e",
                  fontWeight: 500,
                }}
              >
                {details.subText}
              </Typography>
            </Box>
            {onAction && (
              <IOSSwitch checked={checked} onChange={handleChange} />
            )}
          </Box>
        </Box>
      </Box>
    </Card>
  );
}

export default StatusInfoCard;
