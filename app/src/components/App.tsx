import { Box } from "@mui/material";
import { useContext, useEffect, useState } from "react";
import { Status } from "../types/status";
import Console from "./Console/Console";
import Dashboard from "./Dashboard/Dashboard";
import NavBar from "./NavBar/NavBar";
import NodeManager from "./NodeManager/NodeManager";
import { GlobalStatusContext } from "./Providers/ROSProvider";

const SIDEBAR_WIDTH_EXPANDED = 320;
const SIDEBAR_WIDTH_COLLAPSED = 60; // enough for expand/collapse tab

/**
 * The entry point of the program.
 */
function App() {
  const [selectedNode, setSelectedNode] = useState<string | null>(null);
  const [sidebarCollapsed, setSidebarCollapsed] = useState(() => {
    // Attempt to load initial state from localStorage
    if (typeof window === "undefined") return false;
    try {
      const stored = window.localStorage.getItem("sidebarCollapsed");
      return stored === "true";
    } catch {
      return false;
    }
  });
  const { globalStatusHistory, setGlobalStatusHistory } =
    useContext(GlobalStatusContext);

  // Save sidebar collapsed state to localStorage
  useEffect(() => {
    if (typeof window === "undefined") return;
    try {
      window.localStorage.setItem(
        "sidebarCollapsed",
        sidebarCollapsed ? "true" : "false",
      );
    } catch {
      // ignore storage errors
    }
  }, [sidebarCollapsed]);

  const mainSection =
    selectedNode == null ? (
      <Dashboard />
    ) : (
      <Console
        selectedNode={selectedNode}
        clearSelectedNode={() => setSelectedNode(null)}
      />
    );

  const sidebarWidth = sidebarCollapsed
    ? SIDEBAR_WIDTH_COLLAPSED
    : SIDEBAR_WIDTH_EXPANDED;

  return (
    <Box sx={{ height: "100dvh", display: "flex", flexDirection: "column" }}>
      <NavBar
        status={globalStatusHistory.at(-1)?.status ?? Status.Unknown}
        setStatus={setGlobalStatusHistory}
      />
      <Box
        sx={{
          flex: 1,
          display: "flex",
          overflow: "hidden",
          minHeight: 0,
        }}
      >
        <Box
          sx={{
            flexShrink: 0,
            width: sidebarWidth,
            minWidth: sidebarWidth,
            height: "100%",
            overflow: "hidden",
            position: "relative",
            transition: "min-width 0.2s ease, width 0.2s ease",
          }}
        >
          <NodeManager
            setSelectedNode={setSelectedNode}
            selectedNode={selectedNode}
            collapsed={sidebarCollapsed}
            setCollapsed={setSidebarCollapsed}
          />
        </Box>
        <Box
          sx={{
            flex: 1,
            minWidth: 0,
            height: "100%",
            pb: 1,
            pr: 1,
            overflow: "hidden",
          }}
        >
          {mainSection}
        </Box>
      </Box>
    </Box>
  );
}

export default App;
