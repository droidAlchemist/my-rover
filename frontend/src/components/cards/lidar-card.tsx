import { CardContent, CardMedia, Typography } from "@mui/material";
import Card from "@mui/material/Card";

interface LidarCardProps {
  image: string;
}

export function LidarCard({ image }: LidarCardProps) {
  return (
    <Card>
      <CardMedia component="img" height="180" image={image} alt="Lidar data" />
      <CardContent>
        <Typography gutterBottom variant="h5" component="div">
          Lidar
        </Typography>
        <Typography variant="body2" sx={{ color: "text.secondary" }}>
          LiDAR(Light Detection and Ranging) is a remote sensing technology that
          uses lasers to measure distances and generate precise
          three-dimensional information about the shape and characteristics of
          its surrounding objects.
        </Typography>
      </CardContent>
    </Card>
  );
}

export default LidarCard;
