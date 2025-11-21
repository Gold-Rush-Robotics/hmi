import { Typography } from "@mui/material";
import { useContext, useEffect, useState } from "react";
import { formatTimeDifference } from "../../util/util";
import { GlobalStatusContext } from "../Providers/ROSProvider";
import DashboardCard from "./DashboardCard";
import TaskHistoryDialog from "./TaskHistoryDialog";

/**
 * "Current Task" section of the dashboard, displays the extended
 * status of the latest status message (or "Unknown if that is
 * undefined"). The latest status should be the last item in the
 * `globalStatusHistory` array.
 */
function CurrentTask() {
  const { globalStatusHistory } = useContext(GlobalStatusContext);
  const [historyDialogOpen, setHistoryDialogOpen] = useState(false);
  const [now, setNow] = useState(new Date());

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
  }, [now]);

  const handleOpenHistory = () => {
    setHistoryDialogOpen(true);
  };

  const handleCloseHistory = () => {
    setHistoryDialogOpen(false);
  };

  const prevTasks = globalStatusHistory.length - 1;

  return (
    <>
      <DashboardCard
        title="Current Task"
        action={{
          label: "History",
          onClick: handleOpenHistory,
        }}
      >
        <Typography
          sx={{
            color: "text.secondary",
            fontSize: "0.8rem",
            lineHeight: 1.3,
          }}
        >
          {prevTasks} previous task
          {/* Make plural when prevTasks != 1 */}
          {prevTasks !== 1 && "s"}
        </Typography>
        <Typography
          sx={{
            color: "text.secondary",
            fontSize: "0.8rem",
            lineHeight: 1.3,
          }}
        >
          {globalStatusHistory.at(-1)?.extendedStatus ?? "Unknown"} (
          {!!globalStatusHistory.at(-1)?.timestamp &&
            formatTimeDifference(
              globalStatusHistory.at(-1)!.timestamp,
              new Date(),
            )}
          )
        </Typography>
      </DashboardCard>

      <TaskHistoryDialog
        open={historyDialogOpen}
        onClose={handleCloseHistory}
        statusHistory={globalStatusHistory}
      />
    </>
  );
}

export default CurrentTask;
