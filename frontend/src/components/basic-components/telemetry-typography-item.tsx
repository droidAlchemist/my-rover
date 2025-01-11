import { Box, Typography } from "@mui/material";

interface TelemetryTypographyItem {
  title: string;
  value?: string | number;
  suffix?: string;
}

export const TelemetryTypographyItem = ({
  title,
  value,
  suffix = "",
}: TelemetryTypographyItem) => {
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
