import {
  act,
  cleanup,
  getByTestId,
  render,
  waitFor,
  within,
} from "@testing-library/react";
import WS from "jest-websocket-mock";
import { afterEach, beforeEach, describe, expect, it, test, vi } from "vitest";
import ROSProvider from "../ROSProvider";
import { ExposeROSProvider } from "./ExposeROSProvider";

describe("ROSProvider", () => {
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

  it("connects to the WebSocket on mount and subscribes to '/node_info_publisher'", async () => {
    render(<ROSProvider />);
    await mockServer.connected;

    // Check that the client sent the subscribe message
    await expect(mockServer).toReceiveMessage(
      JSON.stringify({
        op: "subscribe",
        topic: "/node_info_publisher",
        type: "std_msgs/msg/String",
      }),
    );
  });

  it("subscribes to stuff the WebSocket tells it to, and adds messages from known nodes to context properly", async () => {
    const { container } = render(
      <ROSProvider>
        <ExposeROSProvider />
      </ROSProvider>,
    );
    await mockServer.connected;

    mockServer.send(JSON.stringify(mockNodes));

    // Make sure that stuff is being added to discovered nodes context
    const discoveredNodesEl = within(container).getByTestId("discovered-nodes");
    const discoveredNodes = JSON.parse(discoveredNodesEl.textContent || "");
    expect("/node_info" in discoveredNodes);
    expect("/test_node" in discoveredNodes);
    expect("/another_node" in discoveredNodes);
    expect(discoveredNodes["/test_node"].length === 2);
    expect(discoveredNodes["/another_node"].length === 1);
    expect(discoveredNodesEl.textContent).contains("/node_info_publisher");
    expect(discoveredNodesEl.textContent).contains("/test_topic");
    expect(discoveredNodesEl.textContent).contains("/test_topic2");

    // Make sure that messages are being logged to websocket history context
    const wsHistoryEl = within(container).getByTestId("ws-history");
    const wsHistory = JSON.parse(wsHistoryEl.textContent || "");
    expect("/node_info" in wsHistory);
    expect("/node_info_publisher" in wsHistory["/node_info"]);
    expect(wsHistory["/node_info"]["/node_info_publisher"].length === 1);
    expect(wsHistoryEl.textContent).contains("/test_node"); // shows that the actual message was logged
  });

  it("adds topics to '/unknown' node when not associated with a node", async () => {
    const { container } = render(
      <ROSProvider>
        <ExposeROSProvider />
      </ROSProvider>,
    );
    await mockServer.connected;

    mockServer.send(
      JSON.stringify({
        topic: "/test_topic",
        msg: { data: "YIPPEE!!" },
      }),
    );

    const wsHistoryEl = within(container).getByTestId("ws-history");
    const wsHistory = JSON.parse(wsHistoryEl.textContent || "");

    expect("/unknown" in wsHistory);
    expect("/test_topic" in wsHistory["/unknown"]);
    expect(wsHistoryEl.textContent).contains("YIPPEE!!");
  });

  it("throws an error when an invalid message is received", async () => {
    render(
      <ROSProvider>
        <ExposeROSProvider />
      </ROSProvider>,
    );
    await mockServer.connected;

    await expect(async () => {
      mockServer.send("This should not be a string!");
    }).rejects.toThrowError();

    await expect(async () => {
      mockServer.send(
        JSON.stringify({
          // has no data
        }),
      );
    }).rejects.toThrowError();

    await expect(async () => {
      mockServer.send(
        JSON.stringify({
          topic: "/node_info_publisher",
          msg: "should not be a string!",
        }),
      );
    }).rejects.toThrowError();
  });

  it("doesn't overwrite previous nodes when nodes aren't shown by '/node_info_publisher' anymore", async () => {
    const { container } = render(
      <ROSProvider>
        <ExposeROSProvider />
      </ROSProvider>,
    );
    await mockServer.connected;

    mockServer.send(
      JSON.stringify({
        topic: "/node_info_publisher",
        msg: { data: JSON.stringify({}) },
      }),
    );

    const discoveredNodesEl = within(container).getByTestId("discovered-nodes");
    const discoveredNodes = JSON.parse(discoveredNodesEl.textContent || "");

    expect("/node_info" in discoveredNodes);
    expect("/node_info_publisher" in discoveredNodes["/node_info"]);
  });

  it("sends whatever is sent in sendRaw", async () => {
    const { getByTestId } = render(
      <ROSProvider>
        <ExposeROSProvider />
      </ROSProvider>,
    );
    await mockServer.connected;

    act(() => {
      getByTestId("send-raw").click();
    });

    await vi.waitFor(() => {
      expect(mockServer.messages.length).toBeGreaterThan(0);
    });

    expect(mockServer.messages).toContain(JSON.stringify({ test: "data" }));
  });

  it("correctly parses and sends advertisements", async () => {
    const { getByTestId } = render(
      <ROSProvider>
        <ExposeROSProvider />
      </ROSProvider>,
    );
    await mockServer.connected;

    act(() => {
      getByTestId("advertise").click();
    });

    await vi.waitFor(() => {
      expect(mockServer.messages.length).toBeGreaterThan(0);
    });

    expect(mockServer.messages).toContain(
      JSON.stringify({
        op: "advertise",
        topic: "/advertise_topic",
        type: "std_msgs/msg/String",
      }),
    );
  });

  it("correctly parses and sends published messages", async () => {
    const { getByTestId } = render(
      <ROSProvider>
        <ExposeROSProvider />
      </ROSProvider>,
    );
    await mockServer.connected;

    act(() => {
      getByTestId("publish").click();
    });

    await vi.waitFor(() => {
      expect(mockServer.messages.length).toBeGreaterThan(0);
    });

    expect(mockServer.messages).toContain(
      JSON.stringify({
        op: "publish",
        topic: "/publish_topic",
        msg: { data: "test message" },
      }),
    );

    expect(mockServer.messages).toContain(
      JSON.stringify({
        op: "publish",
        topic: "/publish_topic",
        msg: "string_should_work_too",
      }),
    );
  });

  it("correctly parses and sends subscribe messages", async () => {
    const { getByTestId } = render(
      <ROSProvider>
        <ExposeROSProvider />
      </ROSProvider>,
    );
    await mockServer.connected;

    act(() => {
      getByTestId("subscribe").click();
    });

    await vi.waitFor(() => {
      expect(mockServer.messages.length).toBeGreaterThan(0);
    });

    expect(mockServer.messages).toContain(
      JSON.stringify({
        op: "subscribe",
        topic: "/subscribe_topic",
        type: "std_msgs/msg/String",
      }),
    );
  });
});
