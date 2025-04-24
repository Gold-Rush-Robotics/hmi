import { Box, Typography } from "@mui/material";
import { useContext } from "react";
import { Status } from "../../types/status";
import { isRunning } from "../../util/status";
import { GlobalStatusContext } from "../Providers/ROSProvider";
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
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        alignItems: "flex-start",
        p: 0.5,
      }}
    >
      <Typography variant="h6">Running Time</Typography>
      {lastRunningTimestamp !== undefined ? (
        <RunningTimeClock timestamp={lastRunningTimestamp} />
      ) : (
        <Typography>Not Running</Typography>
      )}
    </Box>
  );
}

export default RunningTime;
