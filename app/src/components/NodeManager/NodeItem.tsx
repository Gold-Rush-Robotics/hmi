import { ChevronRightRounded } from "@mui/icons-material";
import { Box, Button, Stack, SxProps, Typography } from "@mui/material";
import type { NodeItem } from "../../types/nodeManager";
import StatusIcon from "../StatusIcon";

/**
 * NodeItem component for displaying a node in a list.
 *
 * @param props.name The name of the node.
 * @param props.status The status of the node.
 * @param props.selection The currently selected node name.
 * @param props.setSelectedNode Function to set the selected node.
 */
function NodeItem({ ...props }: NodeItem) {
  const selected = props.selection === props.name;
  let selectedStatusBorder: SxProps = {};
  if (selected) {
    selectedStatusBorder = {
      border: "2px solid",
      borderColor: "#000",
      borderRadius: 25,
    };
  }

  return (
    <Button
      onClick={() => props.setSelectedNode(props.name)}
      sx={{
        width: "100%",
        borderRadius: 25,
        border: "1px solid",
        borderColor: selected ? "black" : "gray",
        bgcolor: selected ? "#aaa" : "#eee",
        textTransform: "none", // Prevents text from being transformed to uppercase
        color: "inherit", // Keeps the original text color
        "&:hover": {
          // Maintain colors on hover
          color: "inherit",
          borderColor: "inherit",
        },
      }}
    >
      <Stack
        direction="row"
        justifyContent="space-between"
        alignItems="center"
        sx={{ width: "100%" }}
      >
        <StatusIcon
          status={props.status}
          sx={{ ml: 0, mr: 1, ...selectedStatusBorder }}
        />
        <Box sx={{ flexGrow: 1, flexShrink: 1, minWidth: 0 }}>
          <Typography
            textAlign={"left"}
            sx={{
              whiteSpace: "nowrap",
              overflow: "hidden",
              textOverflow: "ellipsis",
            }}
          >
            {props.name}
          </Typography>
        </Box>
        <Box
          sx={{
            height: "1.25em",
            width: "1.25em",
            borderRadius: 10,
            bgcolor: "#ccc",
            alignContent: "center",
            margin: 0,
            padding: 0,
            flexShrink: 0,
          }}
        >
          <ChevronRightRounded sx={{ width: 1, height: 1, color: "black" }} />
        </Box>
      </Stack>
    </Button>
  );
}

export default NodeItem;
