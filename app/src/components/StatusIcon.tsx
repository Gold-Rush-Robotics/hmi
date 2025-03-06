import { Circle } from "@mui/icons-material";
import { StatusColors, type StatusIcon } from "../types/status";
import { SxProps } from "@mui/material";

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
  return <Circle htmlColor={color} sx={{ ...defaultSx, ...props.sx }} />;
}

export default StatusIcon;
