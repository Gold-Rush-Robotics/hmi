import { Avatar, Button, Typography } from "@mui/material";
import { useContext } from "react";
import type { EStop } from "../../types/navbar";
import { ROSCommunicationContext } from "../Providers/ROSProvider";

/**
 * Emergency Stop button to immediately send a message to ROS to stop the robot.
 *
 * @param props.style Whether the button should be large (just the "button") or
 * small (a short but long style with text as well).
 */
function EStop({ ...props }: EStop) {
  const send = useContext(ROSCommunicationContext);

  function onClick() {
    send.publish("/hmi_start_stop", "stop");
  }

  if (props.style == "small")
    return (
      <Button
        onClick={onClick}
        variant="contained"
        sx={{
          flex: 1,
          height: "40px",
          borderRadius: 25,
          backgroundColor: "#fee415",
          color: "red",
          p: 0,
          display: "flex",
          alignItems: "center",
          justifyContent: "space-between",
        }}
      >
        <Typography sx={{ fontWeight: "bold", m: 2 }}>
          EMERGENCY STOP
        </Typography>
        <Avatar src="/estop.png">STOP</Avatar>
      </Button>
    );

  if (props.style == "large")
    return (
      <Button
        onClick={onClick}
        variant="contained"
        sx={{
          height: "120px",
          width: "120px",
          borderRadius: 25,
          backgroundColor: "#fee415",
          color: "red",
          p: 0,
          alignItems: "center",
          justifyContent: "space-between",
        }}
      >
        <Avatar src="/estop.png" sx={{ width: "100%", height: "100%" }}>
          STOP
        </Avatar>
      </Button>
    );
}

export default EStop;
