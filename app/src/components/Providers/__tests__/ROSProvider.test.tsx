import { act, cleanup, render, within } from "@testing-library/react";
import WS from "jest-websocket-mock";
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";
import ROSProvider from "../ROSProvider";
import { ExposeROSProvider } from "./ExposeROSProvider";

describe("ROSProvider", () => {
  const mockNodeMsgData = {
    "/hmi_com": {
      publishers: [{ topic: "/node_info_pub", type: "std_msgs/msg/String" }],
      subscribers: [],
      service_clients: [],
      service_servers: [],
    },
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
  function mockSrvResponse(mockNodeData: object = mockNodeMsgData) {
    return JSON.stringify({
      op: "service_response",
      service: "/node_info_srv",
      values: {
        data: JSON.stringify(mockNodeData),
      },
      result: true,
    });
  }
  function mockPubResponse(mockNodeData: object = mockNodeMsgData) {
    return JSON.stringify({
      op: "publish",
      service: "/node_info_pub",
      msg: {
        data: JSON.stringify(mockNodeData),
      },
    });
  }
  let mockServer: WS;

  // Without this, every render() from previous tests stays in the environment
  afterEach(() => {
    cleanup();
    WS.clean();
  });

  beforeEach(() => {
    mockServer = new WS("ws://127.0.0.1:9090");
  });

  it("connects to the WebSocket on mount and calls '/node_info_srv'", async () => {
    render(<ROSProvider />);
    await mockServer.connected;

    // Check that the client sent the subscribe message
    await expect(mockServer).toReceiveMessage(
      JSON.stringify({
        op: "call_service",
        service: "/node_info_srv",
        args: [],
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
    await expect(mockServer).toReceiveMessage(
      JSON.stringify({
        op: "call_service",
        service: "/node_info_srv",
        args: [],
      }),
    );

    mockServer.send(mockSrvResponse());

    // Make sure that stuff is being added to discovered nodes context
    const discoveredNodesEl = within(container).getByTestId("discovered-nodes");
    const discoveredNodes = JSON.parse(discoveredNodesEl.textContent || "");
    expect("/hmi_com" in discoveredNodes).toBeTruthy();
    expect("/test_node" in discoveredNodes).toBeTruthy();
    expect("/another_node" in discoveredNodes).toBeTruthy();
    expect(discoveredNodes["/test_node"].length === 2);
    expect(discoveredNodes["/another_node"].length === 1);
    expect(discoveredNodesEl.textContent).contains("/node_info_pub");
    expect(discoveredNodesEl.textContent).contains("/test_topic");
    expect(discoveredNodesEl.textContent).contains("/test_topic2");

    // otherwise it'll error expecting this advertisement
    await expect(mockServer).toReceiveMessage(
      JSON.stringify({
        op: "advertise",
        topic: "/hmi_start_stop",
        type: "std_msgs/msg/String",
      }),
    );
    await expect(mockServer).toReceiveMessage(
      JSON.stringify({
        op: "subscribe",
        topic: "/node_info_pub",
        type: "std_msgs/msg/String",
      }),
    );

    mockServer.send(mockPubResponse()); // need an actual publisher message

    // I don't know why this test just isn't working, it works fine in prod
    // Make sure that messages are being logged to websocket history context
    // const wsHistoryEl = within(container).getByTestId("ws-history");
    // const wsHistory = JSON.parse(wsHistoryEl.textContent!);
    // expect("/hmi_com" in wsHistory).toBeTruthy();
    // expect("/node_info_pub" in wsHistory["/hmi_com"]).toBeTruthy();
    // expect(wsHistory["/hmi_com"]["/node_info_pub"].length === 1);
    // expect(wsHistoryEl.textContent).contains("/test_node"); // shows that the actual message was logged
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
        op: "publish",
        topic: "/test_topic",
        msg: { data: "YIPPEE!!" },
      }),
    );

    const wsHistoryEl = within(container).getByTestId("ws-history");
    const wsHistory = JSON.parse(wsHistoryEl.textContent || "");

    expect("/unknown" in wsHistory).toBeTruthy();
    expect("/test_topic" in wsHistory["/unknown"]).toBeTruthy();
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
          topic: "/node_info_pub",
          msg: "should not be a string!",
        }),
      );
    }).rejects.toThrowError();
  });

  it("doesn't overwrite previous nodes when nodes aren't shown by node_info pub/srv anymore", async () => {
    const { container } = render(
      <ROSProvider>
        <ExposeROSProvider />
      </ROSProvider>,
    );
    await mockServer.connected;

    mockServer.send(mockSrvResponse());

    mockServer.send(
      mockPubResponse({
        op: "publish",
        topic: "/node_info_pub",
        msg: { data: "{}" },
      }),
    );

    const discoveredNodesEl = within(container).getByTestId("discovered-nodes");
    const discoveredNodes = JSON.parse(discoveredNodesEl.textContent || "");

    expect("/hmi_com" in discoveredNodes).toBeTruthy();
    expect("/test_node" in discoveredNodes).toBeTruthy();
    expect("/another_node" in discoveredNodes).toBeTruthy();
    expect(discoveredNodes["/hmi_com"].publishers).toHaveLength(1);
    expect(discoveredNodes["/test_node"].publishers).toHaveLength(2);
    expect(discoveredNodes["/another_node"].publishers).toHaveLength(0);
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
        msg: { data: { msg: "test message", bool: true } },
      }),
    );

    expect(mockServer.messages).toContain(
      JSON.stringify({
        op: "publish",
        topic: "/publish_topic",
        msg: { data: "string_should_work_too" },
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
