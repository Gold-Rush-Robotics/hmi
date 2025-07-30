import { Typography } from "@mui/material";
import { useContext } from "react";
import { GlobalStatusContext } from "../Providers/ROSProvider";
import DashboardCard from "./DashboardCard";

/**
 * "Current Task" section of the dashboard, displays the extended
 * status of the latest status message (or "Unknown if that is
 * undefined"). The latest status should be the last item in the
 * `globalStatusHistory` array.
 */
function CurrentTask() {
  const { globalStatusHistory } = useContext(GlobalStatusContext);

  return (
    <DashboardCard title="Current Task">
      <Typography
        sx={{
          color: "text.secondary",
          fontSize: "0.8rem",
          lineHeight: 1.3,
        }}
      >
        {globalStatusHistory.at(-1)?.extendedStatus ?? "Unknown"}
      </Typography>
    </DashboardCard>
  );
}

export default CurrentTask;
