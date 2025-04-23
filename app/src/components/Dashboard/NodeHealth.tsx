import { Box, Typography } from "@mui/material";
import { differenceInMilliseconds } from "date-fns";
import { useContext, useEffect, useState } from "react";
import { isRunning } from "../../util/status";
import {
  DiscoveredNodesContext,
  WSHistoryContext,
} from "../Providers/ROSProvider";

function NodeHealth({ ...props }) {
  const nodeData = useContext(DiscoveredNodesContext);
  const nodeHistory = useContext(WSHistoryContext);
  const [_, setUpdate] = useState({}); // dummy state to force update
  const now = new Date();

  let nodeHealthData = {
    discovered: 0,
    running: 0,
    stopped: 0,
    topics: new Set(),
  };

  for (const nodeId in nodeData) {
    const node = nodeData[nodeId];
    nodeHealthData.discovered++;

    // Check if nodes are running/stopped
    if (isRunning(node.status)) {
      nodeHealthData.running++;
    } else {
      nodeHealthData.stopped++;
    }

    // Count topics
    for (const topic of node.publishers) {
      nodeHealthData.topics.add(topic.topic);
    }
    for (const topic of node.subscribers) {
      // in case there are things we don't see
      nodeHealthData.topics.add(topic.topic);
    }
  }

  // Calculate messages per second
  let mps = 0;
  for (const nodeId in nodeHistory) {
    const node = nodeHistory[nodeId];
    for (const topic in node) {
      const msgs = node[topic];
      // Looping backwards (newest at end of array)
      for (let i = msgs.length - 1; i >= 0; i--) {
        const rawMsg = msgs[i];
        if (differenceInMilliseconds(now, rawMsg.timestamp) > 1000) {
          // breaking here stops processing messages older than 1 second.
          break;
        }
        mps++;
      }
    }
  }

  /**
   * Updates the state so that mps will still be recalculated if no new messages
   * come in for more than 100ms
   */
  useEffect(() => {
    let intervalId: number | undefined;
    if (mps > 0) {
      intervalId = setInterval(() => {
        setUpdate(new Object());
      }, 100);
    }
    return () => {
      if (intervalId) {
        clearInterval(intervalId);
      }
    };
  }, [nodeHistory]);

  return (
    <Box
      sx={{
        display: "flex",
        flexDirection: "column",
        alignItems: "flex-start",
        p: 0.5,
      }}
    >
      <Typography variant="h6">Node Health</Typography>
      <Typography>{nodeHealthData.discovered} discovered nodes</Typography>
      <Typography sx={{ pl: 1 }}>- {nodeHealthData.running} running</Typography>
      <Typography sx={{ pl: 1 }}>- {nodeHealthData.stopped} stopped</Typography>
      <Typography>{nodeHealthData.topics.size} discovered topics</Typography>
      <Typography sx={{ pl: 1 }}>- {mps} msgs/sec</Typography>
    </Box>
  );
}

export default NodeHealth;
