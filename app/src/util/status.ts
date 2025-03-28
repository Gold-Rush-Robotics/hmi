import { Status } from "../types/status";

/**
 * Checks if a status represents a running state.
 * @param status The status to check.
 * @returns true if the status given is a running state, otherwise false.
 */
export function isRunning(status: Status) {
  if (status === Status.Unknown) return false;
  if (status === Status.Stopped) return false;
  return true;
}
