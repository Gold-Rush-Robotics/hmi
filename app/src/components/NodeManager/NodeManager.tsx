import { Box, Stack, Typography } from "@mui/joy";
import type { NodeManager } from "../../types/nodeManager";
import NodeItem from "./NodeItem";
import { Status } from "../../types/status";
import { ReactElement, useContext } from "react";
import { DiscoveredNodesContext } from "../Providers/ROSProvider";

function NodeManager({ ...props }: NodeManager) {
  const nodeData = useContext(DiscoveredNodesContext);

  let nodeList: ReactElement[] = [];
  for (const node in nodeData) {
    nodeList.push(
      <NodeItem
        setSelectedNode={props.setSelectedNode}
        selection={props.selectedNode}
        name={node}
        key={node}
        status={Status.OK}
      />,
    );
  }
  return (
    <Stack spacing={1}>
      <Typography>Running Nodes</Typography>
      {nodeList}
    </Stack>
  );
}

export default NodeManager;
