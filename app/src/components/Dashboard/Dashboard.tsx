import { Avatar, Grid2, Paper, Typography } from "@mui/material";
import type { Dashboard } from "../../types/dashboard";
import EStop from "../NavBar/EStop";
import CurrentTask from "./CurrentTask";
import NodeHealth from "./NodeHealth";
import RunningTime from "./RunningTime";

/**
 * The Dashboard component renders the main layout for the application dashboard.
 * It displays key information such as node health, current task, running time,
 * a large E-Stop button, and a fun graphic.
 * It organizes these elements into a two-column layout.
 *
 * @returns The rendered Dashboard component.
 */
function Dashboard({ ...props }: Dashboard) {
  return (
    <Paper
      sx={{
        height: "inherit",
        display: "flex",
        flexDirection: "column",
        borderRadius: "20px 0 0 0",
      }}
    >
      <Typography variant="h5" sx={{ p: 2 }}>
        Dashboard
      </Typography>
      <Grid2 container spacing={2} sx={{ pb: 2 }}>
        <Grid2
          size={6}
          sx={{
            display: "flex",
            flexDirection: "column",
            alignItems: "flex-start",
            pl: 2,
          }}
        >
          <NodeHealth />
          <CurrentTask />
          <RunningTime />
        </Grid2>
        <Grid2
          size={6}
          sx={{
            display: "flex",
            flexDirection: "column",
            alignItems: "center",
            justifyContent: "center",
          }}
        >
          <EStop style="large" />
          <br />
          <Avatar
            src="/yes-yes-sir.gif"
            sx={{
              width: "150px",
              height: "150px",
              pt: 6,
              borderRadius: 0,
            }}
          />
        </Grid2>
      </Grid2>
    </Paper>
  );
}

export default Dashboard;
