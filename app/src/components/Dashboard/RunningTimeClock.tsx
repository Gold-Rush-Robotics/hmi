import { Typography } from "@mui/material";
import { formatDate } from "date-fns";
import { useEffect, useState } from "react";
import type { RunningTimeClock } from "../../types/dashboard";
import { formatTimeDifference } from "../../util/util";

/**
 * Helper component for RunningTime to abstract date formatting logic and
 * also so that when I re-render the component to update the time it doesn't
 * go looping through to find the latest timestamp each time.
 *
 * @param props.timestamp The timestamp representing the time
 * @returns Text containing a timestamp of the running time and how long it's
 * been running for.
 */
function RunningTimeClock({ ...props }: RunningTimeClock) {
  const [now, setNow] = useState(new Date());

  // Format the starting timestamp
  const formattedTimestamp = formatDate(props.timestamp, "HH:mm:ss");

  // Calculate + format the duration between the start date (props.timestamp) and the end date (now)
  const timeDistance = formatTimeDifference(props.timestamp, now);

  // Update `now` every 100ms
  useEffect(() => {
    // Interval to update in 100ms (if I did every 1000ms it could be up to a second off)
    const intervalId = setInterval(() => {
      setNow(new Date());
    }, 100);

    // Clear interval on component refresh
    return () => {
      clearInterval(intervalId);
    };
  }, [props.timestamp, now]);

  return (
    <>
      <Typography>Started {formattedTimestamp}</Typography>
      <Typography>Running for {timeDistance}</Typography>
    </>
  );
}

export default RunningTimeClock;
