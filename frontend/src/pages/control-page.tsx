import { Box } from "@mui/material";
import Grid from "@mui/material/Grid";
import {
  CleaningServices,
  FoodBank,
  Money,
  WrongLocation,
} from "@mui/icons-material";
import DetailedStatisticsCard from "@/components/cards/detailed-statistics-card";
import { VideoStreaming } from "@/components";
import RobotControllerCard from "@/components/cards/robot-controller-card";
import { useAwsCredentials, useAwsIotMqtt } from "@/hooks";

function ControlPage() {
  const credentials = useAwsCredentials();
  const connection = useAwsIotMqtt(credentials);

  return (
    <Box sx={{ position: "relative" }}>
      <Grid container spacing={3} mb={3}>
        <Grid item xs={12} md={6} lg={3}>
          <DetailedStatisticsCard
            title="today's money"
            count="$53,000"
            icon={<Money />}
            percentage={{
              color: "success",
              count: "+55%",
              text: "since yesterday",
            }}
          />
        </Grid>
        <Grid item xs={12} md={6} lg={3}>
          <DetailedStatisticsCard
            title="today's users"
            count="2,300"
            icon={<WrongLocation />}
            percentage={{
              color: "success",
              count: "+3%",
              text: "since last week",
            }}
          />
        </Grid>
        <Grid item xs={12} md={6} lg={3}>
          <DetailedStatisticsCard
            title="new clients"
            count="+3,462"
            icon={<CleaningServices />}
            percentage={{
              color: "error",
              count: "-2%",
              text: "since last quarter",
            }}
          />
        </Grid>
        <Grid item xs={12} md={6} lg={3}>
          <DetailedStatisticsCard
            title="sales"
            count="$103,430"
            icon={<FoodBank />}
            percentage={{
              color: "success",
              count: "+5%",
              text: "than last month",
            }}
          />
        </Grid>
      </Grid>
      <Grid container spacing={3} mb={3}>
        <Grid item xs={12} lg={9}>
          {/* Show video data using kinesis from ROS2 */}
          <VideoStreaming />
          {/* <GradientLineChart
              title="Sales Overview"
              description={
                <Box display="flex" alignItems="center">
                  <Box fontSize={size.lg} color="success" mb={0.3} mr={0.5} lineHeight={0}>
                    <Icon sx={{ fontWeight: "bold" }}>arrow_upward</Icon>
                  </Box>
                  <ArgonTypography variant="button" color="text" fontWeight="medium">
                    4% more{" "}
                    <ArgonTypography variant="button" color="text" fontWeight="regular">
                      in 2022
                    </ArgonTypography>
                  </ArgonTypography>
                </Box>
              }
              chart={gradientLineChartData}
            /> */}
        </Grid>
        <Grid item xs={12} lg={3}>
          {/* <Slider /> */}
          {/* Control robot by sending x,z pos using IOT Core to ROS2 */}
          <RobotControllerCard connection={connection} />
        </Grid>
      </Grid>
      {/* <Grid container spacing={3}>
          <Grid item xs={12} md={8}>
            <SalesTable title="Sales by Country" rows={salesTableData} />
          </Grid>
          <Grid item xs={12} md={4}>
            <CategoriesList title="categories" categories={categoriesListData} />
          </Grid>
        </Grid> */}
    </Box>
  );
}

export default ControlPage;
