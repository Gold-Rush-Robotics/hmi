import type { SxProps } from "@mui/material";

export enum Status {
  OK = "OK",
  Stopped = "Stopped",
  Unknown = "Unknown",
  Loading = "Starting...",
  Warning = "Warning",
  Error = "Error",
  Critical = "Critical Error",
}

export const StatusColors: Record<Status, string> = {
  [Status.OK]: "#00D331FF",
  [Status.Stopped]: "#AAAAAA",
  [Status.Unknown]: "#AAAAAA",
  [Status.Loading]: "#AAAAAA",
  [Status.Warning]: "#FFFF00",
  [Status.Error]: "#FF0000",
  [Status.Critical]: "#FF00BB",
};

export interface StatusIcon {
  status: Status;
  sx?: SxProps;
}
