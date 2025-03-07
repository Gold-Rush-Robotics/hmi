import { IconButton, SxProps } from "@mui/material";
import type { StartButton } from "../../types/navbar";
import { Status } from "../../types/status";
import { isRunning } from "../../util/status";
import { Check, PlayArrow } from "@mui/icons-material";
import { useEffect, useState } from "react";
import { ThemeContext } from "@emotion/react";

/**
 * Sends a message to ROS when clicked if the robot is not already in a running state.
 *
 * @param props.status The current status of the robot.
 * @param props.setStatus A function to update the state if the status.
 */
function StartButton({ ...props }: StartButton) {
  const [wait, setWait] = useState(-1); // until proper implementation
  const [disabled, setDisabled] = useState(
    isRunning(props.status) || props.status === Status.Unknown,
  );
  function onClick() {
    props.setStatus(Status.Loading);
    setDisabled(true);
    setWait(5); // until proper implementation
  }

  /**
   * Temporary delay until this is handled in ROS.
   */
  useEffect(() => {
    if (wait === 0) {
      if (props.status === Status.Loading) {
        props.setStatus(Status.OK);
        setWait(-1);
      }
      return;
    }
    if (wait < 0) {
      setDisabled(isRunning(props.status) || props.status === Status.Unknown);
      return;
    }

    setTimeout(() => {
      setWait((prev) => prev - 1);
    }, 1000);
  }, [wait, props.status]);

  return (
    <IconButton
      {...(wait > -1 || props.status === Status.Unknown
        ? { loading: true }
        : undefined)}
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
