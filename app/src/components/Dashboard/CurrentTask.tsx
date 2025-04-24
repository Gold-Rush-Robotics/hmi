import { Box, Typography } from "@mui/material";
import { useContext } from "react";
import { GlobalStatusContext } from "../Providers/ROSProvider";

/**
 * "Current Task" section of the dashboard, displays the extended
 * status of the latest status message (or "Unknown if that is
 * undefined"). The latest status should be the last item in the
 * `globalStatusHistory` array.
 */
function CurrentTask() {
  const { globalStatusHistory } = useContext(GlobalStatusContext);

  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        alignItems: "flex-start",
        p: 0.5,
      }}
    >
      <Typography variant="h6">Current Task</Typography>
      <Typography>
        {globalStatusHistory.at(-1)?.extendedStatus ?? "Unknown"}
      </Typography>
    </Box>
  );
}

export default CurrentTask;
