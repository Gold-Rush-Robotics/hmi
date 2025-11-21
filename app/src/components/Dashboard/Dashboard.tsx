import { NavigateBefore, NavigateNext } from "@mui/icons-material";
import {
  Avatar,
  Box,
  Grid2,
  IconButton,
  Paper,
  Stack,
  Typography,
} from "@mui/material";
import { useState } from "react";
import type { Dashboard } from "../../types/dashboard";
import EStop from "../NavBar/EStop";
import CurrentTask from "./CurrentTask";
import NodeHealth from "./NodeHealth";
import PageDots from "./PageDots";
import RunningTime from "./RunningTime";

// type DashboardProps = {

// }

/**
 * The Dashboard component renders the main layout for the application dashboard.
 * It displays key information such as node health, current task, running time,
 * a large E-Stop button, and a fun graphic.
 * It organizes these elements into a two-column layout.
 *
 * @returns The rendered Dashboard component.
 */
function Dashboard({ ...props }: Dashboard) {
  const [screenIndex, setScreenIndex] = useState(0);
  const screenElements = 6;

  function onNavigateBefore() {
    setScreenIndex((prev) => {
      let newIndex = prev - 1;
      if (newIndex < 0) {
        newIndex = screenElements - 1;
      }
      return newIndex;
    });
  }

  function onNavigateNext() {
    setScreenIndex((prev) => {
      let newIndex = prev + 1;
      if (newIndex >= screenElements) {
        newIndex = 0;
      }
      return newIndex;
    });
  }

  return (
    <Paper
      sx={{
        height: "inherit",
        display: "flex",
        flexDirection: "column",
        borderRadius: "12px 0 0 0",
        bgcolor: "background.paper",
      }}
    >
      <Box
        sx={{
          p: 1.5,
          display: "flex",
          justifyContent: "space-between",
        }}
      >
        <Typography
          variant="h6"
          sx={{
            fontWeight: 600,
            color: "text.primary",
            mb: 0.5,
          }}
        >
          Dashboard
        </Typography>
        <PageDots steps={screenElements} index={screenIndex} />
        <Stack direction={"row"} gap={1} sx={{ alignItems: "center" }}>
          <IconButton
            onClick={onNavigateBefore}
            size="small"
            sx={{ bgcolor: "black" }}
          >
            <NavigateBefore />
          </IconButton>
          <IconButton
            onClick={onNavigateNext}
            size="small"
            sx={{ bgcolor: "black" }}
          >
            <NavigateNext />
          </IconButton>
        </Stack>
      </Box>

      <Box sx={{ flex: 1, p: 1.5, pt: 0 }}>
        <Grid2 container spacing={1.5} sx={{ height: "100%" }}>
          <Grid2
            size={6}
            sx={{
              display: "flex",
              flexDirection: "column",
              gap: 1.5,
            }}
          >
            <NodeHealth />
            <CurrentTask />
          </Grid2>

          <Grid2
            size={6}
            sx={{
              display: "flex",
              flexDirection: "column",
              gap: 1.5,
            }}
          >
            <RunningTime />
            <Stack
              direction="row"
              spacing={3}
              sx={{
                mt: 2,
                width: "100%",
                alignItems: "center",
                justifyContent: "center",
              }}
            >
              <EStop style="large" />
              <Avatar
                src="/yes-yes-sir.gif"
                sx={{
                  width: "100px",
                  height: "100px",
                  borderRadius: "8px",
                  border: "2px solid",
                  borderColor: "divider",
                }}
              />
            </Stack>
          </Grid2>
        </Grid2>
      </Box>
    </Paper>
  );
}

export default Dashboard;
