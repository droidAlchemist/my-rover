import { IotSensorMessageType } from "@/types";
import { Box, Typography } from "@mui/material";
import Card from "@mui/material/Card";
import { ReactNode } from "react";
import { TelemetryTypographyItem } from "../basic-components";

interface OdometryCardProps {
  gradientColor: string;
  icon: ReactNode;
  data: IotSensorMessageType;
}

export function OdometryCard({ icon, gradientColor, data }: OdometryCardProps) {
  return (
    <Card
      sx={{
        "&.MuiPaper-root": {
          borderRadius: "1rem",
        },
      }}
    >
      <Box sx={{ bgcolor: "white", color: "#344767", p: 2 }}>
        <Box
          sx={{
            display: "flex",
            flexDirection: "row",
            justifyContent: "space-between",
            gap: 1,
          }}
        >
          <Box
            sx={{
              lineHeight: 1,
              display: "flex",
              flexDirection: "column",
              gap: 1,
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
              Odometry
            </Typography>
            <Box
              sx={{
                display: "flex",
                flexDirection: "column",
                gap: 1,
              }}
            >
              <TelemetryTypographyItem
                title="Velocity"
                value={data?.odometry?.[0]}
                suffix="m/s"
              />

              <TelemetryTypographyItem
                title="Ang. Velocity"
                value={data?.odometry?.[1]}
                suffix="m/s"
              />

              <TelemetryTypographyItem title="IMU" />
            </Box>
          </Box>
          <Box
            sx={{
              width: "3rem",
              height: "3rem",
              display: "flex",
              flexShrink: 0,
              justifyContent: "center",
              alignItems: "center",
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
        </Box>
        <Box
          sx={{
            display: "flex",
            flexDirection: "row",
            justifyContent: "space-between",
          }}
        ></Box>
      </Box>
    </Card>
  );
}

export default OdometryCard;
