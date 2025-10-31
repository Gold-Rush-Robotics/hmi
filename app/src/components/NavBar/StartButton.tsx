import { Check, PlayArrow } from "@mui/icons-material";
import { IconButton } from "@mui/material";
import { useContext } from "react";
import type { StartButton } from "../../types/navbar";
import { GlobalStatus } from "../../types/rosProvider";
import { Status } from "../../types/status";
import { isRunning } from "../../util/status";
import { ROSCommunicationContext } from "../Providers/ROSProvider";

/**
 * Sends a message to ROS when clicked if the robot is not already in a running state.
 *
 * @param props.status The current status of the robot.
 * @param props.setStatus A function to update the state if the status.
 */
function StartButton({ ...props }: StartButton) {
  const send = useContext(ROSCommunicationContext);
  const disabled = isRunning(props.status) || props.status === Status.Unknown;

  function onClick() {
    const loading: GlobalStatus = {
      timestamp: new Date(),
      status: Status.Loading,
      extendedStatus: "Sending start message to ROS...", // Current task message
    };

    // Add new status to the history
    props.setStatus((prev) => [...prev, loading]);
    send.publish("/hmi_start_stop", "start");
  }

  return (
    <IconButton
      {...(props.status === Status.Unknown ? { loading: true } : undefined)}
      onClick={onClick}
      disabled={disabled}
      sx={{
        ml: 2,
        bgcolor: "primary.main", // Background color (using theme primary color)
        color: "primary.contrastText", // Text/icon color that contrasts with background
        "&:hover": {
          bgcolor: "primary.dark", // Darker shade on hover
        },
      }}
    >
      {disabled ? <Check /> : <PlayArrow />}
    </IconButton>
  );
}

export default StartButton;
