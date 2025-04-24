import { Avatar, Box, Typography } from "@mui/material";
import type { NavBar } from "../../types/navbar";
import EStop from "./EStop";
import StartButton from "./StartButton";
import StatusSummary from "./StatusSummary";

/**
 * The upper navigation bar of the app.
 *
 * @param props.status The current status history.
 * @param props.setStatus A function to update the status history state.
 * @returns
 */
function NavBar({ ...props }: NavBar) {
  return (
    <Box
      sx={{
        display: "grid",
        gridTemplateColumns: "auto auto", // Two columns: left and right
        alignItems: "center",
        padding: "1rem",
        backgroundColor: "background.surface",
      }}
    >
      <Box sx={{ display: "flex", alignItems: "center" }}>
        <Avatar src="/logo192.png" sx={{ mr: 2 }} /> {/* GRR Logo */}
        <Typography variant="h4" sx={{ mr: 0.5 }}>
          GRR-inator
        </Typography>
        <Avatar
          src="/lil-guy.png"
          sx={{
            ml: 1,
            borderRadius: 0,
            height: "1.5em",
            width: "1.5em",
            bgcolor: "transparent",
          }}
        />
      </Box>
      <Box
        sx={{
          display: "flex",
          justifyContent: "flex-end",
          alignItems: "center",
        }}
      >
        <EStop style={"small"} />
        <StatusSummary status={props.status} />
        <StartButton status={props.status} setStatus={props.setStatus} />
      </Box>
    </Box>
  );
}

export default NavBar;
