/**
=========================================================
* Argon Dashboard 2 MUI - v3.0.1
=========================================================

* Product Page: https://www.creative-tim.com/product/argon-dashboard-material-ui
* Copyright 2023 Creative Tim (https://www.creative-tim.com)

Coded by www.creative-tim.com

 =========================================================

* The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/

// react-routers components

// prop-types is library for typechecking of props

// @mui material components
import Card from "@mui/material/Card";
import Divider from "@mui/material/Divider";
import { Box, Typography } from "@mui/material";

type RobotInfoType = {
  [key: string]: string;
};

interface InfoCardProps {
  title: string;
  description: string;
  info: RobotInfoType;
}

export function InfoCard({ title, description, info }: InfoCardProps) {
  const labels: string[] = [];
  const values: string[] = [];

  // Convert this form `objectKey` of the object key in to this `object key`
  Object.keys(info).forEach((el) => {
    if (el.match(/[A-Z\s]+/)) {
      const uppercaseLetter = Array.from(el).find((i) => i.match(/[A-Z]+/));
      if (uppercaseLetter) {
        const newElement = el.replace(
          uppercaseLetter,
          ` ${uppercaseLetter.toLowerCase()}`,
        );
        labels.push(newElement);
      }
    } else {
      labels.push(el);
    }
  });

  // Push the object values into the values array
  Object.values(info).forEach((el) => values.push(el));

  // Render the card info items
  const renderItems = labels.map((label, key) => (
    <Box key={label} display="flex" py={1} pr={2}>
      <Typography variant="button" fontWeight="bold" textTransform="capitalize">
        {label}: &nbsp;
      </Typography>
      <Typography variant="button" fontWeight="regular" color="text">
        &nbsp;{values[key]}
      </Typography>
    </Box>
  ));

  return (
    <Card sx={{ height: "100%" }}>
      <Box
        display="flex"
        justifyContent="space-between"
        alignItems="center"
        pt={2}
        px={2}
      >
        <Typography variant="h6" fontWeight="medium" textTransform="capitalize">
          {title}
        </Typography>
      </Box>
      <Box p={2}>
        <Box mb={2} lineHeight={1}>
          <Typography
            color="text"
            fontWeight="regular"
            textTransform="unset"
            sx={{ color: "rgb(103, 116, 142)" }}
          >
            {description}
          </Typography>
        </Box>
        <Box sx={{ opacity: 0.3 }}>
          <Divider />
        </Box>
        <Box>{renderItems}</Box>
      </Box>
    </Card>
  );
}

export default InfoCard;
