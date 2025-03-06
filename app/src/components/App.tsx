import { useState } from "react";
import "/src/css/App.css";
import NavBar from "./NavBar/NavBar";
import { Box, Grid } from "@mui/joy";
import NodeManager from "./NodeManager/NodeManager";
import Console from "./Console/Console";
import { Status } from "../types/status";

/**
 * The entry point of the program.
 */
function App() {
  const [selectedNode, setSelectedNode] = useState<string | null>(null);
  const [status, setStatus] = useState<Status>(Status.Stopped);

  return (
    <Box height="100%">
      <NavBar status={status} setStatus={setStatus} />
      <Grid container spacing={2}>
        <Grid md={3}>
          <NodeManager
            setSelectedNode={setSelectedNode}
            selectedNode={selectedNode}
          />
        </Grid>
        <Grid md={9} height="100%">
          <Console
            selectedNode={selectedNode}
            clearSelectedNode={() => setSelectedNode(null)}
          />
        </Grid>
      </Grid>
    </Box>
  );
}

export default App;
