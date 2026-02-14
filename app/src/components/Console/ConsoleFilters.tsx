import { FilterList } from "@mui/icons-material";
import { Box, Chip, ToggleButton } from "@mui/material";
import { useState } from "react";
import type { ConsoleFilters } from "../../types/console";

/**
 * ConsoleFilters component for displaying and managing topic filters.
 *
 * @param props.topicMap - A map of topics and their enabled/disabled state.
 * @param props.disabledTopics - List of topic names that are currently disabled.
 * @param props.setDisabledTopics - Function to set the disabled topics.
 */
function ConsoleFilters({ ...props }: ConsoleFilters) {
  const [showFilters, setShowFilters] = useState(true);

  /**
   * Creates a list of chips for the topic filters.
   *
   * @returns A list of chips for each topic, that you can click to enable/disable.
   */
  function renderFilterChips() {
    if (!showFilters) return;
    const enabledChips = [];
    const disabledChips = [];
    for (const [topic, enabled] of props.topicMap) {
      const chip = getChip(topic, enabled);
      if (enabled) enabledChips.push(chip);
      else disabledChips.push(chip);
    }

    if (enabledChips.length + disabledChips.length === 0) {
      return (
        <Chip
          label="No topics in node!"
          sx={{ ml: 0.5, height: "28px", fontSize: "0.8rem" }}
        />
      );
    }
    return [...enabledChips, ...disabledChips]; // so they are organized :)
  }

  /**
   * Creates a styled chip for the topic that can be clicked to toggle the topic enabled state.
   *
   * @param topic The topic corresponding to the chip
   * @param enabled Whether the topic is enabled or not (true = enabled)
   * @returns A `Chip` component with proper styling and click handlers.
   */
  function getChip(topic: string, enabled: boolean) {
    if (enabled) {
      return (
        <Chip
          key={topic}
          label={topic}
          size="small"
          sx={{
            ml: 0.5,
            height: "28px",
            fontSize: "0.8rem",
            "& .MuiChip-label": {
              px: 1.5,
            },
          }}
          color="primary"
          onDelete={() => disableTopic(topic)}
          onClick={() => disableTopic(topic)}
        />
      );
    }
    return (
      <Chip
        key={topic}
        label={topic}
        size="small"
        sx={{
          ml: 0.5,
          height: "28px",
          fontSize: "0.8rem",
          "& .MuiChip-label": {
            px: 1.5,
          },
        }}
        variant="outlined"
        onClick={() => enableTopic(topic)}
      />
    );
  }

  /**
   * Disables a topic by adding it to the list of disabled topics.
   *
   * @param topic - The topic to disable.
   */
  function disableTopic(topic: string) {
    props.setDisabledTopics((prev) => {
      return [...prev, topic];
    });
  }

  /**
   * Enables a topic by removing it from the list of disabled topics.
   *
   * @param topic - The topic to enable.
   */
  function enableTopic(topic: string) {
    props.setDisabledTopics((prev) => {
      const update = [...prev];
      update.splice(update.indexOf(topic), 1);
      return update;
    });
  }

  /**
   * Renders a toggle button to show or hide the filters.
   *
   * @returns A `ToggleButton` component.
   */
  function renderToggleShowButton() {
    return (
      <ToggleButton
        sx={{
          borderRadius: 8,
          scale: 0.8,
          height: "28px",
          minWidth: "36px",
          border: "1px solid",
          borderColor: "divider",
          bgcolor: "#404040",
          ":hover": {
            borderColor: "primary.main",
            bgcolor: "#505050",
          },
        }}
        size="small"
        value="toggleShowFilters"
        selected={showFilters}
        onClick={() => {
          setShowFilters((prev) => !prev);
        }}
      >
        <FilterList sx={{ fontSize: "1rem" }} />
      </ToggleButton>
    );
  }

  return (
    <Box
      sx={{
        width: "100%",
        display: "flex",
        alignItems: "center",
        overflowX: "scroll", // Only horizontal scroll
        overflowY: "hidden", // Prevent vertical scroll
        "&::-webkit-scrollbar": {
          // Chrome, Safari, newer Edge
          display: "none",
        },
        msOverflowStyle: "none", // IE and Edge
        whiteSpace: "nowrap", // Keeps content in a single line
        py: 0.5,
      }}
    >
      {renderToggleShowButton()}
      {renderFilterChips()}
    </Box>
  );
}

export default ConsoleFilters;
