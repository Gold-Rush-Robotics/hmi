import { Box, Button, Chip, ToggleButton } from "@mui/material";
import type { ConsoleFilters } from "../../types/console";
import { useState } from "react";
import { Filter, Filter1, FilterList } from "@mui/icons-material";

function ConsoleFilters({ ...props }: ConsoleFilters) {
  const [showFilters, setShowFilters] = useState(true);

  /**
   * Creates a list of chips for the topic filters.
   * @returns A list of chips for each topic, that you can click to enable/disable.
   */
  function renderFilterChips() {
    if (!showFilters) return;
    let enabledChips = [];
    let disabledChips = [];
    for (const [topic, enabled] of props.topicMap) {
      const chip = getChip(topic, enabled);
      if (enabled) enabledChips.push(chip);
      else disabledChips.push(chip);
    }

    if (enabledChips.length + disabledChips.length === 0) {
      return <Chip label="No topics in node!" sx={{ ml: 1 }} />;
    }
    return [...enabledChips, ...disabledChips]; // so they are organized :)
  }

  /**
   * Creates a styled chip for the topic that can be clicked to toggle the topic enabled state.
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
          sx={{ ml: 1 }}
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
        sx={{ ml: 1 }}
        variant="outlined"
        onClick={() => enableTopic(topic)}
      />
    );
  }

  function disableTopic(topic: string) {
    props.setDisabledTopics((prev) => {
      return [...prev, topic];
    });
  }

  function enableTopic(topic: string) {
    props.setDisabledTopics((prev) => {
      let update = [...prev];
      update.splice(update.indexOf(topic), 1);
      return update;
    });
  }

  function renderToggleShowButton() {
    const text = showFilters ? "Hide Filters" : "Show Filters";
    return (
      <ToggleButton
        sx={{ borderRadius: 10, scale: 0.85 }}
        size="small"
        value="toggleShowFilters"
        selected={showFilters}
        onClick={() => {
          setShowFilters((prev) => !prev);
        }}
      >
        <FilterList />
      </ToggleButton>
    );
  }

  return (
    <Box sx={{ width: "100%", display: "flex", alignItems: "center" }}>
      {renderToggleShowButton()}
      {renderFilterChips()}
    </Box>
  );
}

export default ConsoleFilters;
