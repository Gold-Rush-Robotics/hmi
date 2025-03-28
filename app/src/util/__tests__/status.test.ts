import { assert, describe, expect, it } from "vitest";
import { Status } from "../../types/status";
import { isRunning } from "../status";

describe("status", () => {
  it("should be correct", () => {
    const runningStates = [
      Status.OK,
      Status.Loading,
      Status.Warning,
      Status.Error,
      Status.Critical,
    ];
    const stoppedStates = [Status.Unknown, Status.Stopped];

    for (const status of Object.values(Status)) {
      if (runningStates.includes(status)) {
        expect(isRunning(status));
        continue;
      }
      if (stoppedStates.includes(status)) {
        expect(!isRunning(status));
        continue;
      }
      assert.fail("Uncategorized status");
    }
  });
});
