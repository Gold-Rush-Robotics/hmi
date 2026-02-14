import { KeyboardTab } from "@mui/icons-material";
import { Box, IconButton, Stack, Typography } from "@mui/material";
import { useCallback, useContext, useEffect, useRef, useState } from "react";
import type { NodeManager } from "../../types/nodeManager";
import { Status } from "../../types/status";
import { requiresAttention } from "../../util/status";
import { DiscoveredNodesContext } from "../Providers/ROSProvider";
import NodeItem from "./NodeItem";

function isStatusOk(status: Status) {
  return status === Status.OK;
}

function renderNodeList(
  nodes: string[],
  nodeData: Record<string, { status: Status }>,
  props: NodeManager,
) {
  return nodes.map((node) => (
    <NodeItem
      setSelectedNode={props.setSelectedNode}
      selection={props.selectedNode}
      name={node}
      key={node}
      status={nodeData[node].status}
      collapsed={props.collapsed}
    />
  ));
}

/**
 * A component that lists all the nodes as detected from ROS, with an icon for their
 * status. Updates the selected node when a node is clicked.
 * Nodes with a non-OK status are shown at the top in a separate section.
 *
 * @param props.selectedNode The currently selected node.
 * @param props.setSelectedNode Used to update the selected node state.
 * @param props.collapsed Whether the sidebar is collapsed.
 * @param props.setCollapsed Used to update the collapsed state.
 */
const SCROLL_SHADOW_THRESHOLD = 8;

function NodeManager({ ...props }: NodeManager) {
  const nodeData = useContext(DiscoveredNodesContext);
  const scrollRef = useRef<HTMLDivElement>(null);
  const [scrollShadows, setScrollShadows] = useState({
    top: false,
    bottom: false,
  });

  const updateScrollShadows = useCallback(() => {
    const el = scrollRef.current;
    if (!el) return;
    const { scrollTop, scrollHeight, clientHeight } = el;
    const canScrollUp = scrollTop > SCROLL_SHADOW_THRESHOLD;
    const canScrollDown =
      scrollHeight - clientHeight - scrollTop > SCROLL_SHADOW_THRESHOLD;
    setScrollShadows((prev) =>
      prev.top !== canScrollUp || prev.bottom !== canScrollDown
        ? { top: canScrollUp, bottom: canScrollDown }
        : prev,
    );
  }, []);

  const sorted = Object.keys(nodeData).sort((a, b) => a.localeCompare(b));
  const problemNodes: string[] = [];
  const okNodes: string[] = [];
  for (const node of sorted) {
    if (requiresAttention(nodeData[node].status)) problemNodes.push(node);
    else okNodes.push(node);
  }

  const problemList = renderNodeList(problemNodes, nodeData, props);
  const okList = renderNodeList(okNodes, nodeData, props);

  useEffect(() => {
    updateScrollShadows();
    const el = scrollRef.current;
    if (!el) return;
    const ro = new ResizeObserver(updateScrollShadows);
    ro.observe(el);
    return () => ro.disconnect();
  }, [updateScrollShadows, problemList.length, okList.length]);

  return (
    <Stack
      spacing={1}
      sx={{
        px: props.collapsed ? 1 : 2,
        py: 1,
        height: "100%",
        display: "flex",
        flexDirection: "column",
        minHeight: 0,
      }}
    >
      {!props.collapsed && (
        <Typography variant="h6" sx={{ flexShrink: 0 }}>
          Nodes
        </Typography>
      )}
      <Box
        sx={{
          flex: 1,
          minHeight: 0,
          position: "relative",
          display: "flex",
          flexDirection: "column",
        }}
      >
        {/* Top shadow when more content above */}
        <Box
          aria-hidden
          sx={{
            position: "absolute",
            left: 0,
            right: 0,
            top: 0,
            height: 24,
            pointerEvents: "none",
            zIndex: 1,
            opacity: scrollShadows.top ? 1 : 0,
            transition: "opacity 0.5s ease-out",
            background: (theme) =>
              `linear-gradient(to bottom, ${theme.palette.background.default} 0%, transparent 100%)`,
          }}
        />
        {/* Bottom shadow when more content below */}
        <Box
          aria-hidden
          sx={{
            position: "absolute",
            left: 0,
            right: 0,
            bottom: 0,
            height: 24,
            pointerEvents: "none",
            zIndex: 1,
            opacity: scrollShadows.bottom ? 1 : 0,
            transition: "opacity 0.5s ease-out",
            background: (theme) =>
              `linear-gradient(to top, ${theme.palette.background.default} 0%, transparent 100%)`,
          }}
        />
        <Stack
          ref={scrollRef}
          spacing={1}
          sx={{
            flex: 1,
            minHeight: 0,
            overflow: "auto",
          }}
          onScroll={updateScrollShadows}
        >
          {problemList.length > 0 && (
            <Stack spacing={1} sx={{ pb: 1 }}>
              {problemList}
            </Stack>
          )}
          {okList.length > 0 && <Stack spacing={1}>{okList}</Stack>}
        </Stack>
      </Box>
      <IconButton
        sx={{
          flexShrink: 0,
          width: "100%",
          borderRadius: 2,
          borderColor: "divider",
          color: "text.primary",
          textTransform: "none", // Prevents text from being transformed to uppercase
          "&:hover": {
            bgcolor: "action.hover",
            borderColor: "primary.main",
          },
          overflow: "hidden",
          whiteSpace: "nowrap",
        }}
        onClick={() => props.setCollapsed((prev) => !prev)}
      >
        {props.collapsed ? (
          <KeyboardTab />
        ) : (
          <>
            <KeyboardTab sx={{ transform: "rotate(180deg)" }} />
            <Typography
              sx={{
                display: "flex",
                alignItems: "center",
                justifyContent: "left",
                gap: 2,
                ml: 2,
              }}
            >
              Collapse Sidebar
            </Typography>
          </>
        )}
      </IconButton>
    </Stack>
  );
}

export default NodeManager;
