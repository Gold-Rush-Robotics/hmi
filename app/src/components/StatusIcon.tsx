import { Circle } from "@mui/icons-material";
import { CircularProgress } from "@mui/material";
import type { SxProps } from "@mui/material";
import { Status, StatusColors, type StatusIcon } from "../types/status";

/**
 * Renders a colored circle icon corresponding with the given status. The colors can be
 * found in the StatusColors record in `../types/status.ts`.
 *
 * @param props.status The status to render an icon for.
 * @param props.sx Optional sx prop to override styling.
 */
function StatusIcon({ ...props }: StatusIcon) {
  const color = StatusColors[props.status];
  const defaultSx: SxProps = { height: ".5em", width: ".5em", ml: 1 };

  if (props.status === Status.Unknown || props.status === Status.Loading) {
    defaultSx.ml = 1.5;
    return (
      <CircularProgress
        thickness={5}
        size="0.7em"
        sx={{ color: color, ...defaultSx, ...props.sx }}
      />
    );
  }

  return <Circle htmlColor={color} sx={{ ...defaultSx, ...props.sx }} />;
}

export default StatusIcon;
