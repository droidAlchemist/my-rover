import { Box, Typography } from "@mui/material";
import Card from "@mui/material/Card";
import Grid from "@mui/material/Grid";
import { ReactNode } from "react";

type CardPercentage = {
  color: string;
  count: string;
  text: string;
};

interface DetailedStaticsCardProps {
  title: string;
  count: string;
  percentage: CardPercentage;
  icon: ReactNode;
}

function DetailedStaticsCard({
  title,
  count,
  percentage,
  icon,
}: DetailedStaticsCardProps) {
  return (
    <Card>
      <Box sx={{ bgcolor: "white" }}>
        <Box p={2}>
          <Grid container>
            <Grid item>
              <Box
                width="3rem"
                height="3rem"
                borderRadius="section"
                display="flex"
                justifyContent="center"
                alignItems="center"
              >
                <Box
                  fontSize="1.125rem"
                  display="grid"
                  color="inherit"
                  sx={{
                    placeItems: "center",
                  }}
                >
                  {icon}
                </Box>
              </Box>
            </Grid>

            <Grid item xs={8}>
              <Box ml={2} lineHeight={1}>
                <Typography
                  variant="button"
                  textTransform="uppercase"
                  fontWeight="medium"
                >
                  {title}
                </Typography>
                <Typography variant="h5" fontWeight="bold" mb={1}>
                  {count}
                </Typography>
              </Box>
            </Grid>
          </Grid>
          <Typography
            display="flex"
            alignItems="center"
            variant="button"
            fontWeight="bold"
            color={percentage.color}
          >
            {percentage.count}
            <Typography
              variant="body2"
              fontWeight="regular"
              ml={0.5}
              mt={-0.125}
            >
              {percentage.text}
            </Typography>
          </Typography>
        </Box>
      </Box>
    </Card>
  );
}

export default DetailedStaticsCard;
