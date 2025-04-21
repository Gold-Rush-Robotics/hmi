import { Box, Grid2 as Grid } from "@mui/material";
import { useContext, useState } from "react";
import Console from "./Console/Console";
import Dashboard from "./Dashboard/Dashboard";
import NavBar from "./NavBar/NavBar";
import NodeManager from "./NodeManager/NodeManager";
import { GlobalStatusContext } from "./Providers/ROSProvider";
import "/src/css/App.css";

/**
 * The entry point of the program.
 */
function App() {
  const [selectedNode, setSelectedNode] = useState<string | null>(null);
  const { globalStatus, setGlobalStatus } = useContext(GlobalStatusContext);

  const mainSection =
    selectedNode == null ? (
      <Dashboard setStatus={setGlobalStatus} />
    ) : (
      <Console
        selectedNode={selectedNode}
        clearSelectedNode={() => setSelectedNode(null)}
      />
    );

  return (
    <Box height="100%">
      <NavBar status={globalStatus} setStatus={setGlobalStatus} />
      <Grid container spacing={2}>
        <Grid size={3}>
          <NodeManager
            setSelectedNode={setSelectedNode}
            selectedNode={selectedNode}
          />
        </Grid>
        <Grid size={9} height="100%">
          {mainSection}
        </Grid>
      </Grid>
    </Box>
  );
}

export default App;
