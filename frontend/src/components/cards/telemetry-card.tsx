import { IotSensorMessageType } from "@/types";
import { roundDecimal } from "@/utils";
import { Box, Typography } from "@mui/material";
import Card from "@mui/material/Card";
import Grid from "@mui/material/Grid2";
import { ReactNode } from "react";

interface OdometryCardProps {
  gradientColor: string;
  icon: ReactNode;
  data: IotSensorMessageType;
}

export function TelemetryCard({
  icon,
  gradientColor,
  data,
}: OdometryCardProps) {
  return (
    <Card
      sx={{
        "&.MuiPaper-root": {
          borderRadius: "1rem",
        },
      }}
    >
      <Box sx={{ bgcolor: "white", color: "#344767" }}>
        <Box p={2}>
          <Box
            sx={{
              display: "flex",
              flexDirection: "row",
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
                Telemetry
              </Typography>
              <Grid container rowSpacing={1} columnSpacing={1}>
                <Grid size={{ xs: 6 }}>
                  {renderTypographyItem("Velocity", data?.odometry?.[0], "m/s")}
                </Grid>
                <Grid size={{ xs: 6 }}>
                  {renderTypographyItem(
                    "Ang. Velocity",
                    data?.odometry?.[1],
                    "m/s",
                  )}
                </Grid>
                <Grid size={{ xs: 6 }}>
                  {renderTypographyItem(
                    "Voltage",
                    roundDecimal(data?.voltage),
                    "V",
                  )}
                </Grid>
                <Grid size={{ xs: 6 }}>
                  {renderTypographyItem(
                    "Temperature",
                    roundDecimal(data?.temperature),
                    "°C",
                  )}
                </Grid>
                <Grid size={{ xs: 6 }}>
                  {renderTypographyItem("IMU Velocity", "_ ")}
                </Grid>
                <Grid size={{ xs: 6 }}>
                  {renderTypographyItem("IMU Ang. Velocity", "_ ")}
                </Grid>
              </Grid>
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
      </Box>
    </Card>
  );
}

export default TelemetryCard;

const renderTypographyItem = (
  title: string,
  value?: string | number,
  suffix = "",
) => {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "row",
        gap: 1,
      }}
    >
      <Typography
        sx={{
          fontSize: "0.75rem",
          lineHeight: 1.25,
          fontWeight: 500,
          color: "#67748e",
        }}
      >
        {title}:
      </Typography>
      <Typography
        sx={{
          fontSize: "0.75rem",
          lineHeight: 1.25,
          color: "#219e5c",
          fontWeight: 600,
        }}
      >
        {value ? value : "_"} {suffix}
      </Typography>
    </Box>
  );
};
