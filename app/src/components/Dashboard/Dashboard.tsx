import { Avatar, Grid2, Paper, Typography } from "@mui/material";
import type { Dashboard } from "../../types/dashboard";
import EStop from "../NavBar/EStop";
import CurrentTask from "./CurrentTask";
import NodeHealth from "./NodeHealth";
import RunningTime from "./RunningTime";

function Dashboard({ ...props }: Dashboard) {
  return (
    <Paper>
      <Typography variant="h5" sx={{ fontWeight: "bold", p: 2 }}>
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
          <EStop style="large" setStatus={props.setStatus} />
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
