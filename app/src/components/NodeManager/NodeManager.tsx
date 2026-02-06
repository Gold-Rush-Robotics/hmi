import { Stack, Typography } from "@mui/material";
import { useContext } from "react";
import type { ReactElement } from "react";
import type { NodeManager } from "../../types/nodeManager";
import { DiscoveredNodesContext } from "../Providers/ROSProvider";
import NodeItem from "./NodeItem";

/**
 * A component that lists all the nodes as detected from ROS, with an icon for their
 * status. Updates the selected node when a node is clicked.
 *
 * @param props.selectedNode The currently selected node.
 * @param props.setSelectedNode Used to update the selected node state.
 */
function NodeManager({ ...props }: NodeManager) {
  const nodeData = useContext(DiscoveredNodesContext);

  const nodeList: ReactElement[] = [];
  for (const node in nodeData) {
    nodeList.push(
      <NodeItem
        setSelectedNode={props.setSelectedNode}
        selection={props.selectedNode}
        name={node}
        key={node}
        status={nodeData[node].status}
      />,
    );
  }
  return (
    <Stack spacing={1} sx={{ p: 2 }}>
      <Typography variant="h6">Running Nodes</Typography>
      {nodeList}
    </Stack>
  );
}

export default NodeManager;
