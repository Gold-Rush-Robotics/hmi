import { Status } from "../types/status";

/**
 * Checks if a status represents a running state.
 * @param status The status to check.
 * @returns true if the status given is a running state, otherwise false.
 */
export function isRunning(status: Status) {
  return ![Status.Unknown, Status.Stopped].includes(status);
}

/**
 * Checks if a status is in a state that warrants user attention (e.g. warning, error).
 * @param status The status to check.
 * @returns true if the status requires attention, otherwise false.
 */
export function requiresAttention(status: Status) {
  return [Status.Warning, Status.Error, Status.Critical].includes(status);
}
