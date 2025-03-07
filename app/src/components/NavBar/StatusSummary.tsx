import { Box } from "@mui/material";
import type { StatusSummary } from "../../types/navbar";
import StatusIcon from "../StatusIcon";

/**
 * This is used in the NavBar as a summary of the entire state of ROS.
 *
 * @param props.status The status to show.
 */
function StatusSummary({ ...props }: StatusSummary) {
  return (
    <Box
      sx={{
        ml: 2,
        bgcolor: "#eee",
        paddingY: 1,
        paddingX: 2,
        borderRadius: 25,
      }}
    >
      Status: {props.status}
      <StatusIcon status={props.status} />
    </Box>
  );
}

export default StatusSummary;
