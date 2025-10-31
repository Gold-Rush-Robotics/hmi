import { Box, Typography } from "@mui/material";
import { differenceInMilliseconds } from "date-fns";
import { useContext, useEffect, useState } from "react";
import { isRunning } from "../../util/status";
import {
  DiscoveredNodesContext,
  WSHistoryContext,
} from "../Providers/ROSProvider";
import DashboardCard from "./DashboardCard";

/**
 * The "Node Health" section of the dashboard; displays information about how many nodes
 * there are, and how many topics there are.
 */
function NodeHealth() {
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

  // Updates the state so that mps will still be recalculated if no new messages
  // come in for more than 100ms
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
    <DashboardCard title="Node Health">
      <Box sx={{ display: "flex", flexDirection: "column", gap: 0.25 }}>
        {/* Discovered nodes text */}
        <Typography sx={{ color: "text.secondary", fontSize: "0.8rem" }}>
          {nodeHealthData.discovered} discovered nodes
        </Typography>
        <Typography sx={{ color: "text.secondary", fontSize: "0.75rem" }}>
          • {nodeHealthData.running} running
        </Typography>
        <Typography sx={{ color: "text.secondary", fontSize: "0.75rem" }}>
          • {nodeHealthData.stopped} stopped
        </Typography>

        {/* Discovered topics text */}
        <Typography
          sx={{ color: "text.secondary", fontSize: "0.8rem", mt: 0.75 }}
        >
          {nodeHealthData.topics.size} discovered topics
        </Typography>
        <Typography sx={{ color: "text.secondary", fontSize: "0.75rem" }}>
          • {mps} msgs/sec
        </Typography>
      </Box>
    </DashboardCard>
  );
}

export default NodeHealth;
