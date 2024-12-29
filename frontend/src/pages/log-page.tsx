import { Box } from "@mui/material";
import Grid from "@mui/material/Grid2";
import { BasicTable } from "@/components";

export const LogPage = () => {
  return (
    <Box sx={{ position: "relative" }}>
      <Grid container spacing={3}>
        <Grid size={{ xs: 8 }}>
          <BasicTable />
        </Grid>
        <Grid size={{ xs: 4 }}>
          <BasicTable />
        </Grid>
      </Grid>
    </Box>
  );
};

export default LogPage;
