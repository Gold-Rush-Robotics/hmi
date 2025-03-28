import { act, cleanup, render } from "@testing-library/react";
import WS from "jest-websocket-mock";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import { Status } from "../../../types/status";
import ROSProvider from "../../Providers/ROSProvider";
import { ExposeROSProvider } from "../../Providers/__tests__/ExposeROSProvider";
import StartButton from "../StartButton";

describe("StartButton", () => {
  const mockNodeMsgData = {
    "/test_node": {
      publishers: [
        { topic: "/test_topic", type: "std_msgs/msg/String" },
        { topic: "/test_topic2", type: "std_msgs/msg/String" },
      ],
      subscribers: [],
      action_clients: [],
      action_servers: [],
      service_clients: [],
      service_servers: [],
    },
    "/another_node": {
      publishers: [],
      subscribers: [{ topic: "/test_topic", type: "std_msgs/msg/String" }],
      action_clients: [],
      action_servers: [],
      service_clients: [],
      service_servers: [],
    },
  };
  const mockNodes = {
    topic: "/node_info_publisher",
    msg: {
      data: JSON.stringify(mockNodeMsgData),
    },
  };
  let mockServer: WS;

  // Without this, every render() from previous tests stays in the environment
  afterEach(() => {
    cleanup();
    WS.clean();
  });

  beforeEach(() => {
    mockServer = new WS("ws://127.0.0.1:9090");
  });

  it("Sends a start message when clicked", async () => {
    const setMockStatus = vi.fn();
    const html = render(
      <ROSProvider>
        <ExposeROSProvider />
        <StartButton setStatus={setMockStatus} status={Status.Stopped} />
      </ROSProvider>,
    );
    await mockServer.connected;

    const buttons = html.getAllByRole("button");
    const startButton = buttons.find((button) => button.querySelector("svg")); // Check for the presence of an SVG

    act(() => startButton!.click());

    await vi.waitFor(() => {
      expect(mockServer.messages.length).toBeGreaterThan(0);
    });

    expect(setMockStatus).toHaveBeenCalled();
    expect(mockServer.messages).toContain(
      JSON.stringify({
        op: "publish",
        topic: "/hmi_start_stop",
        msg: { data: "start" },
      }),
    );
  });
});
