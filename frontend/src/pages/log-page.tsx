import { Box } from "@mui/material";
import Grid from "@mui/material/Grid";
import { BasicTable } from "@/components";

export const LogPage = () => {
  return (
    <Box sx={{ position: "relative" }}>
      <Grid container spacing={3} mb={3}>
        <Grid item xs={12} md={6} lg={8}>
          <BasicTable />
        </Grid>
        <Grid item xs={12} md={6} lg={4}>
          <BasicTable />
        </Grid>
      </Grid>
    </Box>
  );
};

export default LogPage;
