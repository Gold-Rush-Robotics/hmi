import { Avatar, Button, IconButton, Stack, Typography } from "@mui/material";
import { Status } from "../../types/status";
import type { EStop } from "../../types/navbar";
import { SpaceBar } from "@mui/icons-material";

/**
 * Emergency Stop button to immediately send a message to ROS and stop the robot.
 *
 * @param props.setStatus A function to update the state if the status.
 */
function EStop({ ...props }: EStop) {
  function onClick() {
    console.log(props.setStatus(Status.Stopped));
  }

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
      <Typography sx={{ fontWeight: "bold", m: 2 }}>EMERGENCY STOP</Typography>
      <Avatar src="/estop.png">STOP</Avatar>
    </Button>
  );
}

export default EStop;
