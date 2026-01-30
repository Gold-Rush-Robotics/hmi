import { NavigateBefore, NavigateNext } from "@mui/icons-material";
import {
  Avatar,
  Box,
  IconButton,
  Paper,
  Stack,
  Typography,
} from "@mui/material";
import { ReactNode, useContext, useState } from "react";
import type { Dashboard } from "../../types/dashboard";
import EStop from "../NavBar/EStop";
import { ROSDashboardDataContext } from "../Providers/ROSProvider";
import CurrentTask from "./CurrentTask";
import NodeHealth from "./NodeHealth";
import PageDots from "./PageDots";
import RosBar from "./RosBar";
import RosCard from "./RosCard";
import RunningTime from "./RunningTime";

/**
 * The Dashboard component renders the main layout for the application dashboard,
 * featuring a multi-screen paginated interface.
 * * It dynamically maps data from the dashboard data context to their corresponding components,
 * and includes default components (node health, task status, running time, E-Stop button).
 *
 * @returns The rendered Dashboard component with paginated layout and navigation.
 */
function Dashboard({}: Dashboard) {
  const [screenIndex, setScreenIndex] = useState(0);
  const dashboardData = useContext(ROSDashboardDataContext);
  const screens = Object.keys(dashboardData).sort((a, b) => {
    // Main screen should be first, then alphabetical
    if (a === "Dashboard") {
      return -1;
    }
    if (b === "Dashboard") {
      return 1;
    }
    return a.localeCompare(b);
  });
  const pages = screens.length;

  // Default built-in components for the dashboard
  const dashboardComponents = [
    { id: "nodeHealth", element: <NodeHealth /> },
    { id: "currentTask", element: <CurrentTask /> },
    { id: "runningTime", element: <RunningTime /> },
    {
      id: "yippeeEStopStack",
      element: (
        <Stack
          direction="row"
          spacing={3}
          sx={{
            width: "100%",
            alignItems: "center",
            justifyContent: "center",
            py: "6px",
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
      ),
    },
  ];

  // Map the dashboard data for the current screen to the correct components
  const screenCards = Object.values(dashboardData[screens[screenIndex]]).map(
    (cardData) => {
      let element: ReactNode;
      if (cardData.type === "card") {
        element = (
          <RosCard
            key={cardData.id}
            title={cardData.data.title}
            content={cardData.data.content}
          />
        );
      }
      if (cardData.type === "bar") {
        element = (
          <RosBar
            key={cardData.id}
            title={cardData.data.title}
            content={cardData.data.content}
            value={cardData.data.value}
            min={cardData.data.min}
            max={cardData.data.max}
          />
        );
      }
      return { element, id: cardData.id };
    },
  );

  // Combine the screen components with the default components (default components go last)
  const components = [
    ...screenCards,
    ...(screenIndex === 0 ? dashboardComponents : []),
  ];

  /**
   * Navigates to the previous screen.
   */
  function onNavigateBefore() {
    setScreenIndex((prev) => {
      let newIndex = prev - 1;
      if (newIndex < 0) {
        newIndex = pages - 1;
      }
      return newIndex;
    });
  }

  /**
   * Navigates to the next screen.
   */
  function onNavigateNext() {
    setScreenIndex((prev) => {
      let newIndex = prev + 1;
      if (newIndex >= pages) {
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
        {/* Title of the current screen */}
        <Typography
          variant="h6"
          sx={{
            fontWeight: 600,
            color: "text.primary",
            mb: 0.5,
          }}
        >
          {screens[screenIndex]}
        </Typography>

        {/* Pagination dots */}
        <PageDots steps={pages} index={screenIndex} />

        {/* Navigation buttons */}
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

      {/* Container for screen components */}
      <Box sx={{ flex: 1, p: 1.5, pt: 0 }}>
        <Box
          sx={{
            columnCount: 2,
            columnGap: 1.5,
            "& > *": {
              breakInside: "avoid",
              marginBottom: 1.5,
            },
          }}
        >
          {components.map((comp) => (
            <Box key={comp.id}>{comp.element}</Box>
          ))}
        </Box>
      </Box>
    </Paper>
  );
}

export default Dashboard;
