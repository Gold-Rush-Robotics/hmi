import { Typography } from "@mui/material";
import { RosDashboardCard } from "../../types/rosProvider";
import DashboardCard from "./DashboardCard";

function RosCard({ title, content }: RosDashboardCard) {
  return (
    <DashboardCard title={title}>
      <Typography sx={{ color: "text.secondary", fontSize: "0.8rem" }}>
        {content}
      </Typography>
    </DashboardCard>
  );
}

export default RosCard;
