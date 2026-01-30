import { LinearProgress, Typography } from "@mui/material";
import { RosDashboardBar } from "../../types/rosProvider";
import DashboardCard from "./DashboardCard";

function RosCard({ title, content, value, min = 0, max = 1 }: RosDashboardBar) {
  let barValue = ((value - min) / (max - min)) * 100;
  if (value < min) {
    barValue = 0;
  }
  if (value > max) {
    barValue = 100;
  }

  return (
    <DashboardCard title={title}>
      <Typography sx={{ color: "text.secondary", fontSize: "0.8rem", mb: 1 }}>
        {content}
      </Typography>
      <LinearProgress
        variant="determinate"
        value={barValue}
        sx={{ height: 8, borderRadius: 5, mb: 1 }}
      />
    </DashboardCard>
  );
}

export default RosCard;
