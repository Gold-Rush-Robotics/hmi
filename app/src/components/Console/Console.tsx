import { Close } from "@mui/icons-material";
import { Box, IconButton, Paper, Stack, Typography } from "@mui/material";
import { format } from "date-fns";
import { useContext, useEffect, useRef, useState } from "react";
import type { Console, RosConsoleMessage } from "../../types/console";
import { WSHistoryContext } from "../Providers/ROSProvider";
import ConsoleFilters from "./ConsoleFilters";

/**
 * Console component for displaying ROS messages.
 *
 * @param props.selectedNode - The selected node to display messages from.
 * @param props.clearSelectedNode - Function to clear the selected node.
 */
function Console({ ...props }: Console) {
  const consoleTitle = props.selectedNode
    ? `Console: ${props.selectedNode}`
    : "Console";
  const consoleOutputRef = useRef<HTMLDivElement>(null);
  const rawSocketHistory = useContext(WSHistoryContext);
  const filteredSocketHistory = rawSocketHistory[props.selectedNode || ""];
  const [disabledTopics, setDisabledTopics] = useState<string[]>(["/rosout"]);
  let socketHistory: RosConsoleMessage[] = [];
  const [autoScroll, setAutoScroll] = useState(true);
  const maxHistoryLength = 1000; // in lines

  // key = topic, value = enabled (t/f)
  let topics = new Map<string, boolean>();

  for (const topic in filteredSocketHistory) {
    const enabled = !disabledTopics.includes(topic);
    topics.set(topic, enabled);
    if (!enabled) continue; // don't render disabled topics

    const topicHistory = filteredSocketHistory[topic].map((message) => {
      return {
        ...message,
        topic,
      };
    });
    socketHistory = [...socketHistory, ...topicHistory];
  }
  socketHistory.sort((a, b) => a.timestamp.getTime() - b.timestamp.getTime());

  /**
   * Handles console scrolling, including auto-scrolling to the bottom.
   */
  useEffect(() => {
    if (!consoleOutputRef.current) return;

    // Keeps the console scrolled at the bottom
    if (autoScroll) {
      const { current } = consoleOutputRef;
      current.scrollTop = current.scrollHeight;
    }

    // Scrolls up when console history is full and more lines get added to prevent it from shifting down.
    // This isn't perfect but it mostly works.
    if (!autoScroll && socketHistory.length === maxHistoryLength) {
      const { current } = consoleOutputRef;
      current.scrollTop -= 12; // 12px represents roughly 1 line
    }
  }, [socketHistory, autoScroll]);

  /**
   * Checks if auto scrolling should be enabled by checking if the user is at the bottom
   * of the page. Updates `autoScroll` state accordingly.
   */
  function checkEnableAutoScroll() {
    if (consoleOutputRef.current) {
      const { current } = consoleOutputRef;
      // this confused me, it's total height - amount scrolled - height of what's visible
      let distFromBottom = current.scrollHeight - current.scrollTop;
      distFromBottom -= current.clientHeight;

      // 10px to add a little bit of margin
      if (distFromBottom < 10) {
        // Re-enable auto scroll when the user scrolls to the bottom
        setAutoScroll(true);
      } else {
        // Disable auto scroll when the user scrolls manually
        setAutoScroll(false);
      }
    }
  }

  /**
   * Maps the messages from `socketHistory` into a preformatted string that can be used
   * in the console.
   *
   * @returns Preformatted text of the entire console history.
   */
  function renderConsoleText() {
    let text = socketHistory
      .map((msg) => {
        return `[${format(msg.timestamp, "HH:mm:ss.SSS")}] [${msg.topic}] ${msg.message}`;
      })
      .join("\n");
    if (text === "") {
      text =
        "No messages received yet. Perhaps you should try changing your filters?";
    }
    if (socketHistory.length === maxHistoryLength) {
      text = `...history limited to ${maxHistoryLength} lines\n${text}`;
    }
    return text;
  }

  return (
    <Paper
      sx={{
        height: "inherit",
        display: "flex",
        flexDirection: "column",
        borderRadius: "12px 0 0 0",
      }}
    >
      <Box sx={{ px: 1.5, py: 1 }}>
        <Stack
          direction="row"
          justifyContent="space-between"
          alignItems="center"
          sx={{ mb: 1 }}
        >
          <Typography
            variant="h6"
            sx={{
              whiteSpace: "nowrap",
              overflow: "hidden",
              textOverflow: "ellipsis",
              fontWeight: 600,
            }}
          >
            {consoleTitle}
          </Typography>
          <IconButton
            onClick={props.clearSelectedNode}
            sx={{
              borderRadius: 20,
              scale: 0.95,
              border: "1px solid",
              borderColor: "divider",
              bgcolor: "#404040",
              ":hover": {
                borderColor: "primary.main",
                bgcolor: "#505050",
              },
            }}
          >
            <Close sx={{ fontSize: "1.2rem" }} />
          </IconButton>
        </Stack>
        <ConsoleFilters
          topicMap={topics}
          disabledTopics={disabledTopics}
          setDisabledTopics={setDisabledTopics}
        />
      </Box>
      <Paper
        ref={consoleOutputRef}
        onScroll={checkEnableAutoScroll}
        sx={{
          bgcolor: "background.default",
          textAlign: "left",
          overflow: "scroll",
          flexGrow: 1,
          m: 1,
          p: 1.5,
        }}
      >
        <pre
          style={{
            margin: 0,
            padding: 0,
            whiteSpace: "pre-wrap",
            fontSize: "0.75rem",
            lineHeight: 1.2,
            fontFamily: "Cascadia Code, monospace",
          }}
        >
          {renderConsoleText()}
        </pre>
      </Paper>
    </Paper>
  );
}

export default Console;
