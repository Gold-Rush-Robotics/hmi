import { Box, Grid2 as Grid } from "@mui/material";
import { useContext, useState } from "react";
import { Status } from "../types/status";
import Console from "./Console/Console";
import Dashboard from "./Dashboard/Dashboard";
import NavBar from "./NavBar/NavBar";
import NodeManager from "./NodeManager/NodeManager";
import { GlobalStatusContext } from "./Providers/ROSProvider";

/**
 * The entry point of the program.
 */
function App() {
  const [selectedNode, setSelectedNode] = useState<string | null>(null);
  const { globalStatusHistory, setGlobalStatusHistory } =
    useContext(GlobalStatusContext);

  const mainSection =
    selectedNode == null ? (
      <Dashboard />
    ) : (
      <Console
        selectedNode={selectedNode}
        clearSelectedNode={() => setSelectedNode(null)}
      />
    );

  return (
    <Box sx={{ height: "100dvh", display: "flex", flexDirection: "column" }}>
      <NavBar
        status={globalStatusHistory.at(-1)?.status ?? Status.Unknown}
        setStatus={setGlobalStatusHistory}
      />
      <Grid
        container
        spacing={0}
        sx={{
          flexGrow: 1, // This makes the Grid take up all remaining space
          overflow: "hidden", // Prevents scrolling issues
        }}
      >
        <Grid size={4}>
          <NodeManager
            setSelectedNode={setSelectedNode}
            selectedNode={selectedNode}
          />
        </Grid>
        <Grid size={8} height="100%">
          {mainSection}
        </Grid>
      </Grid>
    </Box>
  );
}

export default App;
