import { LinearProgress, Typography } from "@mui/material";
import { RosDashboardBar } from "../../types/rosProvider";
import DashboardCard from "./DashboardCard";

function RosCard({ title, content, value, min, max }: RosDashboardBar) {
  return (
    <DashboardCard title={title}>
      <Typography sx={{ color: "text.secondary", fontSize: "0.8rem", mb: 1 }}>
        {content}
      </Typography>
      <LinearProgress
        variant="determinate"
        value={((value - min) / (max - min)) * 100}
        sx={{ height: 8, borderRadius: 5, mb: 1 }}
      />
    </DashboardCard>
  );
}

export default RosCard;
