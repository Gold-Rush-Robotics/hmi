import { ChevronRightRounded } from "@mui/icons-material";
import type { SxProps } from "@mui/material";
import {
  Box,
  Button,
  IconButton,
  Stack,
  Tooltip,
  Typography,
  useTheme,
} from "@mui/material";
import type { NodeItem } from "../../types/nodeManager";
import { requiresAttention } from "../../util/status";
import StatusIcon from "../StatusIcon";

/**
 * NodeItem component for displaying a node in a list.
 *
 * @param props.name The name of the node.
 * @param props.status The status of the node.
 * @param props.selection The currently selected node name.
 * @param props.setSelectedNode Function to set the selected node.
 * @param props.collapsed Whether the sidebar is collapsed (shows minimized version).
 */
function NodeItem({ ...props }: NodeItem) {
  const theme = useTheme();

  const selected = props.selection === props.name;
  let selectedStatusBorder: SxProps = {};
  if (selected) {
    selectedStatusBorder = {
      border: "2px solid",
      borderColor: "divider",
      borderRadius: 25,
    };
  }

  let attentionColor: string | undefined;
  let attentionColorHover: string | undefined;
  if (requiresAttention(props.status)) {
    attentionColor = theme.palette.error.light;
    attentionColorHover = "#ff8888";
  }

  if (props.collapsed) {
    return (
      <Tooltip title={props.name} placement="right">
        <IconButton
          onClick={() => props.setSelectedNode(props.name)}
          sx={{
            width: "100%",
            borderRadius: 25,
            border: "1px solid",
            borderColor:
              attentionColor ?? (selected ? "primary.main" : "divider"),
            bgcolor: selected
              ? (attentionColor ?? "primary.main")
              : "background.paper",
            color: selected ? "primary.contrastText" : "text.primary",
            "&:hover": {
              bgcolor: selected
                ? (attentionColorHover ?? "primary.dark")
                : "action.hover",
              borderColor: attentionColor ?? "primary.main",
            },
          }}
        >
          <StatusIcon
            status={props.status}
            sx={{ ml: "auto", mr: "auto", ...selectedStatusBorder }}
          />
        </IconButton>
      </Tooltip>
    );
  }

  return (
    <Button
      onClick={() => props.setSelectedNode(props.name)}
      sx={{
        width: "100%",
        borderRadius: 25,
        border: "1px solid",
        borderColor: attentionColor ?? (selected ? "primary.main" : "divider"),
        bgcolor: selected
          ? (attentionColor ?? "primary.main")
          : "background.paper",
        color: selected ? "primary.contrastText" : "text.primary",
        textTransform: "none", // Prevents text from being transformed to uppercase
        "&:hover": {
          bgcolor: selected
            ? (attentionColorHover ?? "primary.dark")
            : "action.hover",
          borderColor: attentionColor ?? "primary.main",
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
            height: "1.3em",
            width: "1.3em",
            borderRadius: 10,
            bgcolor: selected ? "primary.light" : "action.disabledBackground",
            alignContent: "center",
            margin: 0,
            padding: 0,
            flexShrink: 0,
          }}
        >
          <ChevronRightRounded
            sx={{
              width: 1,
              height: 1,
              color: selected ? "divider" : "text.secondary",
            }}
          />
        </Box>
      </Stack>
    </Button>
  );
}

export default NodeItem;
