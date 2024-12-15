import { Box, Typography } from "@mui/material";
import GaugeChart, { GaugeChartProps } from "react-gauge-chart";

type Props = GaugeChartProps & {
  title?: string;
  subheader?: string;
};

export function TelemetryItem({ title, subheader, ...other }: Props) {
  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        gap: 2,
        flexGrow: 1,
        textAlign: "center",
      }}
    >
      <Typography sx={{ color: "white" }} variant="h5">
        {title}
      </Typography>
      <GaugeChart id={`gauge-chart-${title}`} {...other} />
    </Box>
  );
}
