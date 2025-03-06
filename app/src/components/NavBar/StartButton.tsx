import { IconButton } from "@mui/joy";
import type { StartButton } from "../../types/navbar";
import { Status } from "../../types/status";
import { isRunning } from "../../util/status";
import { PlayArrow } from "@mui/icons-material";
import { useEffect, useState } from "react";

/**
 * Sends a message to ROS when clicked if the robot is not already in a running state.
 *
 * @param props.status The current status of the robot.
 * @param props.setStatus A function to update the state if the status.
 */
function StartButton({ ...props }: StartButton) {
  const [wait, setWait] = useState(-1); // until proper implementation
  const [disabled, setDisabled] = useState(isRunning(props.status));
  function onClick() {
    setDisabled(true);
    setWait(5); // until proper implementation
  }

  /**
   * Temporary delay until this is handled in ROS.
   */
  useEffect(() => {
    if (wait === 0) {
      if (props.status == Status.Stopped) {
        props.setStatus(Status.OK);
        setWait(-1);
      }
      return;
    }
    if (wait < 0) {
      setDisabled(isRunning(props.status));
      return;
    }

    setTimeout(() => {
      setWait((prev) => prev - 1);
    }, 1000);
  }, [wait, props.status]);

  return (
    <IconButton
      variant="solid"
      color="primary"
      onClick={onClick}
      disabled={disabled}
      sx={{ borderRadius: 50, ml: 2 }}
    >
      <PlayArrow />
    </IconButton>
  );
}

export default StartButton;
