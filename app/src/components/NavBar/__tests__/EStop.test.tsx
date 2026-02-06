import { act, cleanup, render } from "@testing-library/react";
import WS from "jest-websocket-mock";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import config from "../../../util/config";
import ROSProvider from "../../Providers/ROSProvider";
import { ExposeROSProvider } from "../../Providers/__tests__/ExposeROSProvider";
import EStop from "../EStop";

describe("EStop", () => {
  const mockNodeMsgData = {
    "/test_node": {
      publishers: [
        { topic: "/test_topic", type: "std_msgs/msg/String" },
        { topic: "/test_topic2", type: "std_msgs/msg/String" },
      ],
      subscribers: [],
      service_clients: [],
      service_servers: [],
    },
    "/another_node": {
      publishers: [],
      subscribers: [{ topic: "/test_topic", type: "std_msgs/msg/String" }],
      service_clients: [],
      service_servers: [],
    },
  };
  const _mockNodes = {
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
    mockServer = new WS(config.rosIp);
  });

  it("Sends a stop message when clicked", async () => {
    const _setMockStatus = vi.fn();
    const html = render(
      <ROSProvider>
        <ExposeROSProvider />
        <EStop style="small" />
      </ROSProvider>,
    );
    await mockServer.connected;

    const button = html.getByText("EMERGENCY STOP");

    act(() => button.click());

    await vi.waitFor(() => {
      expect(mockServer.messages.length).toBeGreaterThan(0);
    });

    expect(mockServer.messages).toContain(
      JSON.stringify({
        op: "publish",
        topic: "/hmi_start_stop",
        msg: { data: "stop" },
      }),
    );
  });
});
