import { createContext, useEffect, useRef, useState } from "react";
import type {
  DiscoveredNodes,
  DiscoveredTopic,
  GlobalStatus,
  GlobalStatusContextType,
  RosCommunicationContext,
  RosDashboardItemData,
  RosDashboardScreenItems,
  RosMessage,
  RosNodeInfo,
  RosPublishResponse,
  RosResponse,
  RosServiceRespone,
  RosType,
  WSHistory,
} from "../../types/rosProvider";
import { Status } from "../../types/status";
import config from "../../util/config";
import { isRunning } from "../../util/status";
import { deepCombineObjects } from "../../util/util";

export const WSHistoryContext = createContext<WSHistory>({});
export const DiscoveredNodesContext = createContext<DiscoveredNodes>({});
export const GlobalStatusContext = createContext<GlobalStatusContextType>({
  globalStatusHistory: [],
  setGlobalStatusHistory: () => {},
});
export const ROSCommunicationContext = createContext<RosCommunicationContext>({
  sendRaw: () => {},
  advertise: () => {},
  callService: () => {},
  publish: () => {},
  subscribe: () => {},
});
export const ROSDashboardDataContext = createContext<RosDashboardScreenItems>(
  {},
);

/**
 * A global state provider for subscribing to ROS topics and keeping track of received data.
 */
function ROSProvider({ ...props }) {
  const rosIP = config.rosIp;
  const wsRef = useRef<WebSocket | null>(null);
  const [wsHistory, setWSHistory] = useState<WSHistory>({});
  const [globalStatusHistory, setGlobalStatusHistory] = useState<
    GlobalStatus[]
  >([
    {
      timestamp: new Date(),
      status: Status.Unknown,
      extendedStatus: "Loading...",
    },
  ]);
  const [discoveredNodes, setDiscoveredNodes] = useState<DiscoveredNodes>({});
  const [dashboardData, setDashboardData] = useState<RosDashboardScreenItems>({
    Dashboard: {},
  });

  const [lastReceived, setLastReceived] = useState(new Date());
  const timeoutPollRate = 10000; // milliseconds; we should be receiving data from ROS every ~5 seconds at minimum

  /**
   * All publishers present in this app. Everything in here will be registered with ROS.
   */
  const publishers: DiscoveredTopic[] = [
    {
      topic: "/hmi_start_stop",
      type: "std_msgs/msg/String",
    },
  ];

  /**
   * Handles the main connection for the WebSocket (connections, errors, etc.).
   * Note that this has (or should have) no deps array - if there are deps then
   * it will be running handleWsMessage() from the old state until one of the
   * deps update.
   */
  useEffect(() => {
    if (wsRef.current === null) {
      wsRef.current = new WebSocket(rosIP);
      console.log(`Initializing new WebSocket connection to '${rosIP}'...`);
    }
    const ws = wsRef.current;

    ws.onopen = () => {
      console.log("Connected to ROS!");
      setLastReceived(new Date()); // to avoid timeout
      callService("/node_info_srv");
      for (const publisher of publishers) {
        advertise(publisher.topic, publisher.type);
      }
    };

    ws.onerror = () => {
      // I love how the message isn't exposed so I have to put something generic
      console.warn("An error occurred in the websocket connection.");
    };

    ws.onmessage = (event) => handleWsMessage(event, new Date());
  });

  /**
   * Polls to make sure the connection is stable, and will restart the connection or mark the
   * connection as unknown if nothing recent has happened.
   */
  // useEffect(() => {
  //   setTimeout(() => {
  //     const ws = wsRef.current;
  //     const now = new Date();
  //     const elapsed = now.getMilliseconds() - lastReceived.getMilliseconds();
  //     if (elapsed < timeoutPollRate) {
  //       return;
  //     }

  //     if (ws?.readyState === WebSocket.OPEN) {
  //       // mark status as warn
  //       if (elapsed > 2 * timeoutPollRate) {
  //         // mark status as error
  //       }
  //     }

  //     // if (ws?.readyState === WebSocket.)
  //   }, 3000);
  // }, [lastReceived]);

  /**
   * Parses a message received from the ROS Websocket.
   *
   * @param event The message event received from the ROS WebSocket.
   */
  function handleWsMessage(event: MessageEvent, timestamp: Date) {
    setLastReceived(timestamp);

    let data: RosResponse;
    try {
      data = JSON.parse(event.data) as RosResponse;
    } catch (e) {
      throw new Error(
        `Error parsing data. Data: '${JSON.stringify(event.data)}'`,
      );
    }

    // Handlers based on the operation type
    if (data.op === "service_response") {
      handleServiceResponse(data, timestamp);
    } else if (data.op === "publish") {
      handleTopicResponse(data, timestamp);
    } else {
      throw new Error(`Unknown data type! Data: '${JSON.stringify(data)}'`);
    }
  }

  /**
   * Parses a publisher response received from the ROS websocket.
   *
   * @param data The publisher response from ROS.
   * @param timestamp The timestamp the message was received at.
   */
  function handleTopicResponse(data: RosPublishResponse, timestamp: Date) {
    const node = findTopicParent(data.topic);
    const message: RosMessage = {
      message: data.msg.data,
      timestamp,
    };

    // Topic handlers
    if (data.topic === "/node_info_pub") {
      checkNodeUpdates(message);
    } else if (data.topic === "/hmi_start_stop") {
      handleHmiStartStop(message);
    } else if (data.topic === "/hmi_dashboard_data") {
      handleRosDashboardDataMessage(message);
    }

    setWSHistory((prev) => {
      // Make sure the value didn't already get added somehow due to react shenanigans
      // array.at(index) is the same as array[index] but allows for negative indexing
      if (prev[node] && prev[node][data.topic]?.at(-1) === message) return prev;

      // Merge new with old
      let update = { ...prev };
      update[node] = {
        ...prev[node],
        [data.topic]: [...(prev[node]?.[data.topic] ?? []), message],
      };

      return update;
    });
  }

  /**
   * Parses a service response received from the ROS websocket.
   *
   * @param response The service response from ROS.
   * @param timestamp The timestamp the response was received at.
   */
  function handleServiceResponse(response: RosServiceRespone, timestamp: Date) {
    const values: any = response.values ?? {};
    const data = values.data;
    const message: RosMessage = {
      timestamp,
      message: data,
    };

    if (response.service === "/node_info_srv") {
      checkNodeUpdates(message);
    }
  }

  /**
   * Processes updates containing ROS node and topic information.
   * Discovers new nodes and topics, updates internal state, and subscribes to new publishers.
   *
   * @param update The message from the node info source. Expected JSON payload in `update.message`.
   * @throws Error If the message payload cannot be parsed as JSON.
   */
  function checkNodeUpdates(update: RosMessage) {
    // Initialize global status if it's unknown
    if (globalStatusHistory.at(-1)?.status === Status.Unknown)
      setGlobalStatusHistory((prev) => {
        const stopped: GlobalStatus = {
          timestamp: new Date(),
          status: Status.Stopped,
          extendedStatus: "N/A",
        };
        return [...prev, stopped];
      });

    // Attempt to parse the incoming message string as JSON
    let data: RosNodeInfo;
    try {
      data = JSON.parse(update.message.toString());
    } catch (e) {
      throw new Error(
        `Error parsing node info. Data: '${JSON.stringify(update.message)}. Error: ${e}'`,
      );
    }

    // Create a set of nodes present in this update to track which ones are still online
    const activeNodes = new Set<string>(Object.keys(data));

    let newDiscovered: DiscoveredNodes = {};
    for (const node in data) {
      // Check if this node is completely new (not in our discoveredNodes state)
      if (!Object.hasOwn(discoveredNodes, node)) {
        newDiscovered[node] = { status: Status.OK, ...data[node] };
        continue;
      }

      // If the node already exists, check for new publishers on this node
      const pubUpdates = data[node].publishers;
      const discoveredPubs = discoveredNodes[node].publishers;
      let newDiscoveredPubs: DiscoveredTopic[] = [];
      for (const publisher of pubUpdates) {
        // Check if this specific publisher topic is already known for this node
        let found = false;
        for (const discoveredPub of discoveredPubs) {
          if (discoveredPub.topic === publisher.topic) {
            found = true;
            break;
          }
        }

        if (!found) {
          // If the publisher topic was not found in the discovered list, it's new
          newDiscoveredPubs.push({
            topic: publisher.topic,
            type: publisher.type,
          });
        }
      }

      // Now, check for new subscribers on this node (logic is similar to publishers)
      const subUpdates = data[node].subscribers;
      const discoveredSubs = discoveredNodes[node].subscribers;
      const newDiscoveredSubs: DiscoveredTopic[] = [];
      for (const subscriber of subUpdates) {
        // Check if this specific subscriber topic is already known for this node
        let found = false;
        for (const discoveredSub of discoveredSubs) {
          if (discoveredSub.topic === subscriber.topic) {
            found = true;
            break;
          }
        }
        if (!found) {
          // If the subscriber topic was not found in the discovered list, it's new
          newDiscoveredSubs.push({
            topic: subscriber.topic,
            type: subscriber.type,
          });
        }
      }

      // If any new publishers or subscribers were found for this existing node
      if (newDiscoveredSubs.length !== 0 || newDiscoveredPubs.length !== 0) {
        const nodeData = data[node];
        // Add the node to newDiscovered, including *only* the new pubs/subs found.
        // Service clients/servers are included from the latest update without diffing.
        newDiscovered[node] = {
          status: Status.OK,
          publishers: newDiscoveredPubs,
          subscribers: newDiscoveredSubs,
          service_clients: nodeData.service_clients,
          service_servers: nodeData.service_servers,
        };
      }
    }

    // Add new things and check for nodes that have gone offline
    setDiscoveredNodes((prev) => {
      let update: DiscoveredNodes = deepCombineObjects(prev, newDiscovered);

      // Check for nodes that went offline, and vice versa
      for (const node in prev) {
        if (isRunning(prev[node].status)) {
          if (activeNodes.has(node)) continue;

          // This node was online but is not in the current update
          update[node] = { ...update[node], status: Status.Stopped };
        } else {
          if (!activeNodes.has(node)) continue;

          // This node was offline but now it is included again in the current update
          update[node] = { ...update[node], status: Status.OK };
        }
      }

      return update;
    });

    // Subscribe to new stuffs
    for (const node in newDiscovered) {
      for (const pub of newDiscovered[node].publishers) {
        subscribe(pub.topic, pub.type);
      }
    }
  }

  /**
   * Processes start/stop updates. These will usually be sent out from StartButton, but
   * this also allows for the ROS server to send messages on this topic to update the client
   * status.
   *
   * @param update The message from the `/hmi_start_stop` topic. Expected JSON payload in
   * `update.message`.
   * @throws Error If the message payload cannot be parsed as JSON.
   */
  function handleHmiStartStop(update: RosMessage) {
    // Make sure this is a string
    const msg = update.message as unknown;
    if (typeof msg !== "string") {
      console.warn(
        "Warning: '/hmi_start_stop' message received that was not a string! Not processing.",
      );
      console.warn("Message received:", msg);
      return;
    }

    // Here is where we will ignore it if server-side status is setup

    // Handle operations
    switch (msg.toLowerCase()) {
      case "start":
        setGlobalStatusHistory((prev) => [
          ...prev,
          {
            timestamp: update.timestamp,
            status: Status.OK,
            extendedStatus: "Running",
          },
        ]);
        break;
      case "stop":
        setGlobalStatusHistory((prev) => [
          ...prev,
          {
            timestamp: update.timestamp,
            status: Status.Stopped,
            extendedStatus: "N/A",
          },
        ]);
        break;

      default:
        console.warn(
          "Skipping invalid operation received from '/hmi_start_stop':",
          msg,
        );
    }
  }

  /**
   * Processes dashboard data updates.
   * Updates the global context of dashboard data with the latest data received.
   *
   * @param message The raw message from the `/hmi_dashboard_data` topic.
   */
  function handleRosDashboardDataMessage(message: RosMessage) {
    const data = JSON.parse(message.message as string) as RosDashboardItemData;
    console.log("DASHBOARD DATA FROM ROS:", data);
    setDashboardData((prev) => ({
      ...prev,
      // Replace the item in the screen with what just came in
      [data.screen]: {
        ...prev[data.screen],
        [data.id]: data,
      },
    }));
  }

  /**
   * Subscribes to the specified topic.
   *
   * @param topic The name of a topic to subscribe to (e.g. "/my_topic").
   * @param type The type of the topic being subscribed to (e.g. "std_msgs/msg/String").
   */
  function subscribe(topic: string, type: RosType) {
    const message = {
      op: "subscribe",
      topic,
      type,
    };
    sendRaw(message);
  }

  /**
   * Calls the specified service with the provided args.
   *
   * @param service The name of the service to call.
   * @param args Optional. A list of json objects representing the arguments. See https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md#338-call-service
   */
  function callService(service: string, args?: object[]) {
    args ??= [];
    const message = {
      op: "call_service",
      service,
      args,
    };
    sendRaw(message);
  }

  /**
   * Sends a raw message to ROS.
   *
   * @param message The message object to stringify and send to ROS.
   */
  function sendRaw(message: object) {
    const socket = wsRef.current;
    if (!socket) {
      throw new Error(`Socket is null! Unable to send message:\n${message}.`);
    }
    socket.send(JSON.stringify(message));
  }

  /**
   * Sends a message to ROS that we are or will be publishing a topic.
   *
   * @param topic The name of the topic to advertise.
   * @param type The type of the advertized topic.
   */
  function advertise(topic: string, type: string) {
    const message = {
      op: "advertise",
      topic,
      type,
    };
    sendRaw(message);
  }

  /**
   * Publishes a message to ROS to the specified topic. Note that the topic
   * message must match the topic type.
   *
   * @param topic The name of the topic to send a message to.
   * @param msg The message to send.
   */
  function publish(topic: string, msg: object | string) {
    const message = {
      op: "publish",
      topic,
      msg: { data: msg },
    };
    sendRaw(message);
  }

  /**
   * Uses linear search to find the node belonging to a topic. It is possible that if multiple topics
   * have the same name, this could return the same one for both, but I think we have bigger issues if
   * we have multiple topics sharing a name.
   *
   * @param topic The topic to find
   * @returns The node containing the topic
   */
  function findTopicParent(topic: string) {
    for (const node in discoveredNodes) {
      for (const nodeTopic of discoveredNodes[node].publishers) {
        if (topic === nodeTopic.topic) return node;
      }
    }
    console.warn(
      `No node was found matching topic ${topic}! Using "/unknown".`,
    );
    return "/unknown";
  }

  return (
    <GlobalStatusContext
      value={{ globalStatusHistory, setGlobalStatusHistory }}
    >
      <WSHistoryContext.Provider value={wsHistory}>
        <DiscoveredNodesContext.Provider value={discoveredNodes}>
          <ROSCommunicationContext.Provider
            value={{ sendRaw, advertise, callService, publish, subscribe }}
          >
            <ROSDashboardDataContext.Provider value={dashboardData}>
              {props.children}
            </ROSDashboardDataContext.Provider>
          </ROSCommunicationContext.Provider>
        </DiscoveredNodesContext.Provider>
      </WSHistoryContext.Provider>
    </GlobalStatusContext>
  );
}

export default ROSProvider;
