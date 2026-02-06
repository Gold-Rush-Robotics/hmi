import { Box, Typography } from "@mui/material";
import type { RosDashboardColor } from "../../types/rosProvider";
import DashboardCard from "./DashboardCard";

function RosColor({ title, content, color }: RosDashboardColor) {
  return (
    <DashboardCard title={title}>
      <Box sx={{ display: "flex", flexDirection: "column", gap: 1.5 }}>
        <Typography sx={{ color: "text.secondary", fontSize: "0.8rem" }}>
          {content}
        </Typography>
        <Box
          sx={{
            width: "100%",
            height: "0.5em",
            backgroundColor: color,
            borderRadius: 2,
          }}
        />
      </Box>
    </DashboardCard>
  );
}

export default RosColor;
