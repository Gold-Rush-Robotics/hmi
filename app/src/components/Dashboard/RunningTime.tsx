import { Typography } from "@mui/material";
import { useContext } from "react";
import { Status } from "../../types/status";
import { isRunning } from "../../util/status";
import { GlobalStatusContext } from "../Providers/ROSProvider";
import DashboardCard from "./DashboardCard";
import RunningTimeClock from "./RunningTimeClock";

/**
 * The "Running Time" section of the dashboard; contains some dates to show when the bot started
 * and how long it's been running for.
 */
function RunningTime() {
  const { globalStatusHistory } = useContext(GlobalStatusContext);
  const now = new Date();

  function getLastRunningTimestamp() {
    // Check the earliest state it was running in since the last time it was stopped
    let earliestRunningIndex = 0; // negatively indexed
    while (
      isRunning(
        globalStatusHistory.at(earliestRunningIndex - 1)?.status ??
          Status.Unknown,
      )
    ) {
      earliestRunningIndex--;
    }

    // If it's 0 then it's currently stopped
    if (earliestRunningIndex === 0) {
      return;
    }

    return globalStatusHistory.at(earliestRunningIndex)?.timestamp;
  }
  const lastRunningTimestamp = getLastRunningTimestamp();

  return (
    <DashboardCard title="Running Time">
      {lastRunningTimestamp !== undefined ? (
        <RunningTimeClock timestamp={lastRunningTimestamp} />
      ) : (
        <Typography sx={{ color: "text.secondary", fontSize: "0.8rem" }}>
          Not Running
        </Typography>
      )}
    </DashboardCard>
  );
}

export default RunningTime;
