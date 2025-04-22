import { createContext, useEffect, useRef, useState } from "react";
import type {
  DiscoveredNodes,
  DiscoveredTopic,
  GlobalStatusContextType,
  RosCommunicationContext,
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
import { deepCombineObjects } from "../../util/util";

export const WSHistoryContext = createContext<WSHistory>({});
export const DiscoveredNodesContext = createContext<DiscoveredNodes>({});
export const GlobalStatusContext = createContext<GlobalStatusContextType>({
  globalStatus: Status.Unknown,
  setGlobalStatus: () => {},
});
export const ROSCommunicationContext = createContext<RosCommunicationContext>({
  sendRaw: () => {},
  advertise: () => {},
  callService: () => {},
  publish: () => {},
  subscribe: () => {},
});

/**
 * A global state provider for subscribing to ROS topics and keeping track of received data.
 */
function ROSProvider({ ...props }) {
  const rosIP = config.rosIp;
  const wsRef = useRef<WebSocket | null>(null);
  const [wsHistory, setWSHistory] = useState<WSHistory>({});
  const [globalStatus, setGlobalStatus] = useState<Status>(Status.Unknown);
  const [lastReceived, setLastReceived] = useState(new Date());
  const timeoutPollRate = 10000; // milliseconds; we should be receiving data from ROS every ~5 seconds at minimum

  /**
   * Default discovered node list. Includes topics we are initially actively
   * using or could possibly use before receiving an initial response.
   */
  const [discoveredNodes, setDiscoveredNodes] = useState<DiscoveredNodes>({});

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
    console.log(data);

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

    if (data.topic == "/node_info_pub") {
      checkNodeUpdates(message);
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
   * Updates nodes based on nodes and topics received from /node_info_pub
   *
   * @param update The message from /node_info_pub or data from
   */
  function checkNodeUpdates(update: RosMessage) {
    if (globalStatus === Status.Unknown) setGlobalStatus(Status.Stopped);

    let data: RosNodeInfo;
    try {
      data = JSON.parse(update.message.toString());
    } catch (e) {
      throw new Error(
        `Error parsing node info. Data: '${JSON.stringify(update.message)}. Error: ${e}'`,
      );
    }

    let newDiscovered: DiscoveredNodes = {};
    for (const node in data) {
      if (!Object.hasOwn(discoveredNodes, node)) {
        newDiscovered[node] = data[node];
        continue;
      }

      const pubUpdates = data[node].publishers;
      const discoveredPubs = discoveredNodes[node].publishers;
      let newDiscoveredPubs: DiscoveredTopic[] = [];
      for (const publisher of pubUpdates) {
        let found = false;
        for (const discoveredPub of discoveredPubs) {
          if (discoveredPub.topic === publisher.topic) {
            found = true;
            break;
          }
        }
        if (!found) {
          newDiscoveredPubs.push({
            topic: publisher.topic,
            type: publisher.type,
          });
        }
      }

      const subUpdates = data[node].subscribers;
      const discoveredSubs = discoveredNodes[node].subscribers;
      const newDiscoveredSubs: DiscoveredTopic[] = [];
      for (const subscriber of subUpdates) {
        let found = false;
        for (const discoveredSub of discoveredSubs) {
          if (discoveredSub.topic === subscriber.topic) {
            found = true;
            break;
          }
        }
        if (!found) {
          newDiscoveredSubs.push({
            topic: subscriber.topic,
            type: subscriber.type,
          });
        }
      }

      if (newDiscoveredSubs.length !== 0 || newDiscoveredPubs.length !== 0) {
        const nodeData = data[node];
        newDiscovered[node] = {
          publishers: newDiscoveredPubs,
          subscribers: newDiscoveredSubs,
          service_clients: nodeData.service_clients,
          service_servers: nodeData.service_servers,
        };
      }
    }

    // Nothing new found
    if (Object.keys(newDiscovered).length === 0) return;

    // Add new things!
    setDiscoveredNodes((prev) => {
      let update: DiscoveredNodes = deepCombineObjects(prev, newDiscovered);
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
    <GlobalStatusContext value={{ globalStatus, setGlobalStatus }}>
      <WSHistoryContext.Provider value={wsHistory}>
        <DiscoveredNodesContext.Provider value={discoveredNodes}>
          <ROSCommunicationContext.Provider
            value={{ sendRaw, advertise, callService, publish, subscribe }}
          >
            {props.children}
          </ROSCommunicationContext.Provider>
        </DiscoveredNodesContext.Provider>
      </WSHistoryContext.Provider>
    </GlobalStatusContext>
  );
}

export default ROSProvider;
